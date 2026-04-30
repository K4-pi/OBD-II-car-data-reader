#include "../include/obd2.h"
#include "../include/uart.h"
#include "../include/pids_obd2.h"

#include <cstddef>
#include <cstdint>

extern "C"
{
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

Obd2::Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart)
	: m_tx { tx }
	, m_rx { rx }
	, m_header_ide { false }
	, m_uart { uart }
{
	m_initialized = false;
}

static void rx_task(void *arg)
{
	Obd2 *self = static_cast<Obd2 *>(arg);
	twai_frame_t rx_frame;

	while (1)
	{
		if (xQueueReceive(self->m_obd2_rx_queue_hdl, &rx_frame, portMAX_DELAY))
		{
			ESP_DRAM_LOGI("CAN", "ID: 0x%03lX len: %d", rx_frame.header.id, rx_frame.buffer_len);

			uint8_t A = rx_frame.buffer[3];
			uint8_t B = rx_frame.buffer[4];
			uint8_t C = rx_frame.buffer[5];
			uint8_t D = rx_frame.buffer[6];
			uint8_t E = rx_frame.buffer[7];

			int value; 

			switch (rx_frame.buffer[2]) 
			{
				case CHECK_PIDS:
					self->m_initialized = true;
					break;

				case ENGINE_SPEED:
					value = (256*A + B) / 4;
					self->m_uart.printf("ENGINE_SPEED=%d", value);
					break;

				case ENGINE_LOAD:
					value = (100*A) / 255;
					self->m_uart.printf("ENGINE_LOAD=%d", value);
					break;

				case VEHICLE_SPEED:
					value = A;
					self->m_uart.printf("VEHICLE_SPEED=%d", value);
					break;

				case COOLANT_TEMP:
					value = A - 40;
					self->m_uart.printf("COOLANT_TEMP=%d", value);
					break;

				case FUEL_PRESSURE:
					value = 3*A;
					self->m_uart.printf("FUEL_PRESSURE=%d", value);
					break;

				case FUEL_TRIM_STFT1:
					value = (100*A) / 128 - 100;
					self->m_uart.printf("STFT1=%d", value);
					break;

				case FUEL_TRIM_STFT2:
					value = (100*A) / 128 - 100;
					self->m_uart.printf("STFT1=%d", value);
					break;

				case FUEL_TRIM_LTFT1:
					value = (100*A) / 128 - 100;
					self->m_uart.printf("STFT1=%d", value);
					break;
	
				case FUEL_TRIM_LTFT2:
					value = (100*A) / 128 - 100;
					self->m_uart.printf("STFT1=%d", value);
					break;	

				case ENGINE_PERECENT_TORQUE:
					self->m_uart.printf("ENGINE_PERECENT_TORQUE=%d,%d,%d,%d,%d", A-125, B-125, C-125, D-125, E-125);	
					break;

				default:
					self->m_uart.printf("ERR=No matching PID");
					break;
			}
			self->m_uart.printf("\n");
		}
	}
}

static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	uint8_t recv_buff[8];

	twai_frame_t rx_frame = {};
	rx_frame.buffer = recv_buff;
	rx_frame.buffer_len = 8;

	Obd2 *self = static_cast<Obd2 *>(user_ctx);

	if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) 
	{
		if (rx_frame.header.id != 0x7E8 && 
			(rx_frame.header.id < 0x18DAF100 && rx_frame.header.id > 0x18DAF110)) return false;

		BaseType_t higher_prio_woken = pdFALSE;
		xQueueSendFromISR(self->m_obd2_rx_queue_hdl, &rx_frame, &higher_prio_woken);

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

	twai_event_callbacks_t user_cbs = {};
	user_cbs.on_rx_done = twai_rx_callback;

	ESP_ERROR_CHECK(twai_node_register_event_callbacks(m_node_hdl, &user_cbs, this));

	// Start TWAI controller
	ESP_ERROR_CHECK(twai_node_enable(m_node_hdl));	

	m_obd2_rx_queue_hdl = xQueueCreate(16, sizeof(twai_frame_t)); // Create queue for receiving data 	

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

	can_protocol_t presets[] = {
		{true,  500000},
		{false, 500000},
		{true,  250000},
		{false, 250000},
	};
	const size_t presets_len = sizeof(presets) / sizeof(presets[0]);

	const uint8_t check_code[] = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	uint8_t index = 0;

	while (!m_initialized && index < presets_len)
	{
		if (uxQueueMessagesWaiting(m_obd2_rx_queue_hdl) != 0) continue;

		Obd2::update_bitrate(presets[index].bitrate);
		
		m_header_ide = presets[index].ide;

		if (m_header_ide) 
		{
			Obd2::send(IDE_29_BIT, check_code, 8);
			m_uart.printf("IDE 29-bit, bitrate = %d", presets[index].bitrate);
		}
		else 
		{
			Obd2::send(IDE_11_BIT, check_code, 8);
			m_uart.printf("IDE 11-bit, bitrate = %d", presets[index].bitrate);
		}

		index++;
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

	twai_event_callbacks_t user_cbs = {};
	user_cbs.on_rx_done = twai_rx_callback;

	err = twai_node_register_event_callbacks(m_node_hdl, &user_cbs, this);
	if (ESP_OK != err) return err;

	err = twai_node_enable(m_node_hdl);
	if (ESP_OK != err) return err;

	return ESP_OK;
}

void Obd2::send(uint32_t id, const uint8_t *buffer, uint8_t len)
{
	twai_frame_t msg = {};
	msg.header.id = id;
	msg.header.ide = m_header_ide; // true = 29-bit, false = 11-bit
	msg.buffer = (uint8_t *)buffer;
	msg.buffer_len = len;

	esp_err_t err = twai_node_transmit(m_node_hdl, &msg, 0);  // Timeout = 0: returns immediately if queue is full
	if (err == ESP_ERR_INVALID_STATE)
	{
		ESP_LOGW("CAN", "bus-off recover");
		twai_node_disable(m_node_hdl);
		twai_node_enable(m_node_hdl);
		return;
	}
	ESP_ERROR_CHECK(err); 
												 
	ESP_ERROR_CHECK(twai_node_transmit_wait_all_done(m_node_hdl, -1));  // Wait for transmission to finish
}

