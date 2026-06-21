#include "../include/obd2.h"
#include "../include/uart.h"
#include "../include/pids_obd2.h"

#include <cstddef>
#include <cstdint>

extern "C"
{
    #include "freertos/projdefs.h"
	#include "hal/twai_types_deprecated.h"
	#include "portmacro.h"
	#include "esp_rom_sys.h"
	#include "freertos/idf_additions.h"
	#include "soc/gpio_num.h"
	#include "esp_err.h"
	#include "esp_twai.h"
	#include "esp_twai_onchip.h"
	#include "esp_twai_types.h"
	#include "esp_log.h"
}

static void rx_task(void *arg);
static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx);

struct queued_twai_frame_t
{
	twai_frame_t frame;
	uint8_t data[8];
};

Obd2::Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart)
	: m_tx { tx }
	, m_rx { rx }
	, m_header_ide { false }
	, m_user_callbacks {  }
	, m_vin_in_progress { false }
	, m_vin_next_sn { 1 }
	, m_vin_len { 0 }
	, m_vin { 0 }
	, m_uart { uart }
{
	m_initialized = false;
}

static void rx_task(void *arg)
{
	Obd2 *self = static_cast<Obd2 *>(arg);
	queued_twai_frame_t queued_frame = {};

	while (1)
	{
		if (xQueueReceive(self->m_obd2_rx_queue_hdl, &queued_frame, portMAX_DELAY))
		{
			const twai_frame_t &rx_frame = queued_frame.frame;
			if (rx_frame.buffer == nullptr || rx_frame.buffer_len < 3)
			{
				continue;
			}

			if (self->handle_vin_response(rx_frame))
			{
				continue;
			}

			const uint8_t bytes_to_print = rx_frame.buffer_len > 8 ? 8 : rx_frame.buffer_len;

			uint8_t MODE = rx_frame.buffer[1];
			uint8_t PID  = rx_frame.buffer[2];
			const uint8_t base_mode = MODE >= 0x40 ? static_cast<uint8_t>(MODE - 0x40) : MODE;

			const uint8_t A = rx_frame.buffer_len > 3 ? rx_frame.buffer[3] : 0;
			const uint8_t B = rx_frame.buffer_len > 4 ? rx_frame.buffer[4] : 0;
			const uint8_t C = rx_frame.buffer_len > 5 ? rx_frame.buffer[5] : 0;
			const uint8_t D = rx_frame.buffer_len > 6 ? rx_frame.buffer[6] : 0;
			const uint8_t E = rx_frame.buffer_len > 7 ? rx_frame.buffer[7] : 0;

			int value;

			if (MODE == POSITIVE_TROUBLE_CODES_RESPONSE)
			{
			    self->m_uart.printf("ERR_CODE=");

				for (int i = 0; i < bytes_to_print; i++)
				{
				    self->m_uart.printf("%d ", rx_frame.buffer[i]);
				}

				self->m_uart.printf("\n");
			}
			else if (base_mode == INFORMATION_MODE)
			{
			    switch (PID)
    			{
        			case VIN:
                        self->m_uart.printf("VIN=Requested\n");
                        break;

                    default:
                        self->m_uart.printf("ERR=No matching for %X PID\n", PID);
                        break;
    			}
			}
			else if (base_mode == CURRENT_DATA_MODE)
			{
                switch (PID)
    			{
    				case CHECK_PIDS:
                        if (!self->m_initialized)
                        {
                            self->m_initialized = true;
                        }
    					break;

    				case ENGINE_SPEED:
    					value = (256*A + B) / 4;
    					self->m_uart.printf("ENGINE_SPEED=%d\n", value);
    					break;

    				case ENGINE_LOAD:
    					value = (100*A) / 255;
    					self->m_uart.printf("ENGINE_LOAD=%d\n", value);
    					break;

    				case VEHICLE_SPEED:
    					value = A;
    					self->m_uart.printf("VEHICLE_SPEED=%d\n", value);
    					break;

    				case COOLANT_TEMP:
    					value = A - 40;
    					self->m_uart.printf("COOLANT_TEMP=%d\n", value);
    					break;

    				case ENGINE_OIL_TEMP:
    					value = A - 40;
    					self->m_uart.printf("OIL_TEMP=%d\n", value);
    					break;

    				case FUEL_TRIM_STFT1:
    					value = (100*A) / 128 - 100;
    					self->m_uart.printf("STFT1=%d\n", value);
    					break;

    				case FUEL_TRIM_STFT2:
    					value = (100*A) / 128 - 100;
    					self->m_uart.printf("STFT2=%d\n", value);
    					break;

    				case FUEL_TRIM_LTFT1:
    					value = (100*A) / 128 - 100;
    					self->m_uart.printf("LTFT1=%d\n", value);
    					break;

    				case FUEL_TRIM_LTFT2:
    					value = (100*A) / 128 - 100;
    					self->m_uart.printf("LTFT2=%d\n", value);
    					break;

    				case ENGINE_PERECENT_TORQUE:
    					self->m_uart.printf("ENGINE_PERECENT_TORQUE=%d,%d,%d,%d,%d\n", A-125, B-125, C-125, D-125, E-125);
    					break;

    				default:
    					self->m_uart.printf("ERR=No matching for %X PID\n", PID);
    					break;
                }
			}
		}
	}
}

void Obd2::reset_vin_state()
{
	m_vin_in_progress = false;
	m_vin_next_sn = 1;
	m_vin_len = 0;
	m_vin[0] = '\0';
}

bool Obd2::handle_vin_response(const twai_frame_t &rx_frame)
{
	if (rx_frame.buffer == nullptr || rx_frame.buffer_len == 0)
	{
		return false;
	}

	const uint8_t pci_type = (rx_frame.buffer[0] >> 4) & 0x0F;

	// Single Frame: [PCI, MODE, PID, frame_idx, VIN...]
	if (pci_type == 0x0)
	{
		if (rx_frame.buffer_len < 5) return false;
		if (rx_frame.buffer[1] != static_cast<uint8_t>(INFORMATION_MODE + 0x40) || rx_frame.buffer[2] != VIN) return false;

		reset_vin_state();
		for (uint8_t i = 4; i < rx_frame.buffer_len && m_vin_len < 17; i++)
		{
			if (rx_frame.buffer[i] != 0x00)
			{
				m_vin[m_vin_len++] = static_cast<char>(rx_frame.buffer[i]);
			}
		}
		m_vin[m_vin_len] = '\0';
		if (m_vin_len == 17)
		{
			m_uart.printf("VIN=%s\n", m_vin);
		}
		return true;
	}

	// First Frame: [10 LL, total_len, MODE, PID, frame_idx, VIN...]
	if (pci_type == 0x1)
	{
		if (rx_frame.buffer_len < 6) return false;
		if (rx_frame.buffer[2] != static_cast<uint8_t>(INFORMATION_MODE + 0x40) || rx_frame.buffer[3] != VIN) return false;

		reset_vin_state();
		m_vin_in_progress = true;

		for (uint8_t i = 5; i < rx_frame.buffer_len && m_vin_len < 17; i++)
		{
			if (rx_frame.buffer[i] != 0x00)
			{
				m_vin[m_vin_len++] = static_cast<char>(rx_frame.buffer[i]);
			}
		}
		m_vin[m_vin_len] = '\0';

		uint32_t flow_control_id = IDE_11_BIT;
		if (rx_frame.header.ide)
		{
			flow_control_id = (rx_frame.header.id & 0xFFFF0000U) |
			                 ((rx_frame.header.id & 0x000000FFU) << 8) |
			                  0x000000F1U;
		}
		else if (rx_frame.header.id >= 0x8)
		{
			flow_control_id = rx_frame.header.id - 0x8;
		}

		const uint8_t flow_control[8] = { 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		const esp_err_t fc_err = send_with_id(flow_control_id, rx_frame.header.ide, flow_control, 8);
		if (fc_err != ESP_OK)
		{
			ESP_LOGW("CAN", "VIN flow-control failed: %s", esp_err_to_name(fc_err));
			reset_vin_state();
			return true;
		}

		if (m_vin_len == 17)
		{
			m_uart.printf("VIN=%s\n", m_vin);
			reset_vin_state();
		}

		return true;
	}

	// Consecutive Frame: [2N, VIN...]
	if (pci_type == 0x2 && m_vin_in_progress)
	{
		if (rx_frame.buffer_len < 2) return true;

		const uint8_t frame_sn = rx_frame.buffer[0] & 0x0F;
		if (frame_sn != m_vin_next_sn)
		{
			ESP_LOGW("CAN", "VIN sequence mismatch: got %u expected %u", frame_sn, m_vin_next_sn);
			reset_vin_state();
			return true;
		}

		for (uint8_t i = 1; i < rx_frame.buffer_len && m_vin_len < 17; i++)
		{
			if (rx_frame.buffer[i] != 0x00)
			{
				m_vin[m_vin_len++] = static_cast<char>(rx_frame.buffer[i]);
			}
		}
		m_vin[m_vin_len] = '\0';
		m_vin_next_sn = static_cast<uint8_t>((m_vin_next_sn + 1) & 0x0F);

		if (m_vin_len == 17)
		{
			m_uart.printf("VIN=%s\n", m_vin);
			reset_vin_state();
		}
		return true;
	}

	return false;
}

static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	(void)edata;
	uint8_t recv_buff[8] = {};

	twai_frame_t rx_frame = {};
	rx_frame.buffer = recv_buff;
	rx_frame.buffer_len = 8;

	Obd2 *self = static_cast<Obd2 *>(user_ctx);

	if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame))
	{
		if (rx_frame.header.id != 0x7E8 &&
		   (rx_frame.header.id < 0x18DAF100 ||
			rx_frame.header.id > 0x18DAF1FF))
		    return false;

		queued_twai_frame_t queued_frame = {};
		queued_frame.frame = rx_frame;
		const uint8_t copy_len = rx_frame.buffer_len > 8 ? 8 : rx_frame.buffer_len;
		for (uint8_t i = 0; i < copy_len; i++)
		{
			queued_frame.data[i] = recv_buff[i];
		}
		queued_frame.frame.buffer = queued_frame.data;
		queued_frame.frame.buffer_len = copy_len;

		BaseType_t higher_prio_woken = pdFALSE;
		xQueueSendFromISR(self->m_obd2_rx_queue_hdl, &queued_frame, &higher_prio_woken);

		return higher_prio_woken == pdTRUE;
  }

  return false;
}

bool Obd2::setup()
{
	m_node_hdl = NULL;

	m_node_config = {};
	m_node_config.io_cfg.tx = m_tx;             // TWAI TX GPIO pin
	m_node_config.io_cfg.rx = m_rx;             // TWAI RX GPIO pin
	m_node_config.bit_timing.bitrate = 500000;
	m_node_config.tx_queue_depth = 5;           // Transmit queue depth

	// TWAI controller driver instance
	ESP_ERROR_CHECK(twai_new_node_onchip(&m_node_config, &m_node_hdl));

	m_user_callbacks.on_rx_done = twai_rx_callback;

	ESP_ERROR_CHECK(twai_node_register_event_callbacks(m_node_hdl, &m_user_callbacks, this));

	// Start TWAI controller
	ESP_ERROR_CHECK(twai_node_enable(m_node_hdl));

	m_obd2_rx_queue_hdl = xQueueCreate(32, sizeof(queued_twai_frame_t)); // Create queue for receiving data

	xTaskCreate(rx_task, "obd2_RX", 4096, this, 5, NULL); // 4096 words -> 16KB

	Obd2::initialize_twai_frame_conf();

  	return true;
}

void Obd2::initialize_twai_frame_conf()
{
	struct can_protocol_t {
		bool ide;
		uint32_t bitrate;
	};

	constexpr can_protocol_t presets[] = {
		{true,  500000},
		{false, 500000},
		{true,  250000},
		{false, 250000},
	};
	constexpr size_t presets_len = sizeof(presets) / sizeof(presets[0]);

	constexpr uint8_t check_code[] = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint8_t index = 0;

	while (!m_initialized && index < presets_len)
	{
        if (uxQueueMessagesWaiting(m_obd2_rx_queue_hdl) != 0)
		{
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

        m_uart.printf("Queue = %d\n", uxQueueMessagesWaiting(m_obd2_rx_queue_hdl));

		ESP_ERROR_CHECK(Obd2::update_bitrate(presets[index].bitrate));

		m_header_ide = presets[index].ide;

		Obd2::send(check_code, 8);

		if (m_header_ide)
		{
			m_uart.printf("IDE 29-bit, bitrate = %d\n", presets[index].bitrate);
		}
		else
		{
			m_uart.printf("IDE 11-bit, bitrate = %d\n", presets[index].bitrate);
		}

		index++;

		vTaskDelay(pdMS_TO_TICKS(3000)); // Giving ECU time to response before sending another test
	}
}

esp_err_t Obd2::update_bitrate(uint32_t bitrate)
{
	// Disabling and uninstalling twai controller
	esp_err_t err = twai_node_disable(m_node_hdl);
	if (ESP_OK != err) return err;

	err = twai_node_delete(m_node_hdl);
	if (ESP_OK != err) return err;

	// bitrate change
	m_node_config.bit_timing.bitrate = bitrate;

	// Re-enabling twai controller and setting callbacks
	err = twai_new_node_onchip(&m_node_config, &m_node_hdl);
	if (ESP_OK != err) return err;

	m_user_callbacks.on_rx_done = twai_rx_callback;

	err = twai_node_register_event_callbacks(m_node_hdl, &m_user_callbacks, this);
	if (ESP_OK != err) return err;

	err = twai_node_enable(m_node_hdl);
	if (ESP_OK != err) return err;

	return ESP_OK;
}

esp_err_t Obd2::send_with_id(uint32_t id, bool ide, const uint8_t *buffer, uint8_t len)
{
	twai_frame_t msg = {};
	msg.header.ide = ide; // true = 29-bit, false = 11-bit
	msg.header.id = id;

	msg.buffer = (uint8_t *)buffer;
	msg.buffer_len = len;

	esp_err_t err = twai_node_transmit(m_node_hdl, &msg, 0);  // Timeout = 0: returns immediately if queue is full
	if (err == ESP_ERR_INVALID_STATE)
	{
		ESP_LOGW("CAN", "bus-off recover");
		esp_err_t reenable_err = twai_node_enable(m_node_hdl);
		if (reenable_err != ESP_OK)
		{
			ESP_LOGW("CAN", "recover failed: %s", esp_err_to_name(reenable_err));
		}
		return err;
	}
	if (err != ESP_OK)
	{
		return err;
	}

	const esp_err_t wait_err = twai_node_transmit_wait_all_done(m_node_hdl, 100);  // Avoid blocking forever on transient bus issues
	if (wait_err == ESP_ERR_TIMEOUT)
	{
		ESP_LOGW("CAN", "tx wait timeout for ID: 0x%03lX", id);
		return wait_err;
	}
	if (wait_err == ESP_ERR_INVALID_STATE)
	{
		ESP_LOGW("CAN", "tx wait invalid state, trying re-enable");
		esp_err_t reenable_err = twai_node_enable(m_node_hdl);
		if (reenable_err != ESP_OK)
		{
			ESP_LOGW("CAN", "recover failed: %s", esp_err_to_name(reenable_err));
		}
		return wait_err;
	}
	return wait_err;
}

void Obd2::send(const uint8_t *buffer, uint8_t len)
{
	const uint32_t id = m_header_ide ? IDE_29_BIT : IDE_11_BIT;
	const esp_err_t err = send_with_id(id, m_header_ide, buffer, len);
	if (err != ESP_OK && err != ESP_ERR_TIMEOUT && err != ESP_ERR_INVALID_STATE)
	{
		ESP_ERROR_CHECK(err);
	}
}
