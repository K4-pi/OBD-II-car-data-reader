#include "../include/obd2.h"
#include "../include/uart.h"
#include "../include/pids_obd2.h"

#include <cstddef>
#include <cstdint>

extern "C"
{
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

Obd2::Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart)
  : m_tx { tx }
  , m_rx { rx }
	, m_uart { uart }
{}

static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  uint8_t recv_buff[8];

  twai_frame_t rx_frame = {};
  rx_frame.buffer = recv_buff;
  rx_frame.buffer_len = 8;

	Obd2 *self = static_cast<Obd2 *>(user_ctx);

	if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) 
	{
		if (rx_frame.header.id != 0x7E8) return false;

    BaseType_t higher_prio_woken = pdFALSE;
		xQueueSendFromISR(self->m_obd2_rx_queue_hdl, &rx_frame, &higher_prio_woken);

		return higher_prio_woken == pdTRUE;
  }
	 
  return false;
}

void Obd2::send(uint32_t id, const uint8_t *buffer, uint8_t len)
{
	// ESP_LOGI("CAN", "sending ID: 0x%03lX", id);

	twai_frame_t msg = {};
	msg.header.id = id;
	msg.header.ide = false; // true = 29-bit, false = 11-bit
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


static void rx_task(void *arg)
{
  Obd2 *self = static_cast<Obd2 *>(arg);
	twai_frame_t rx_frame;

	while (1)
	{
		if (xQueueReceive(self->m_obd2_rx_queue_hdl, &rx_frame, portMAX_DELAY))
		{
			ESP_DRAM_LOGI("CAN", "ID: 0x%03lX len: %d", rx_frame.header.id, rx_frame.buffer_len);

			uint8_t value;

			switch (rx_frame.buffer[2]) 
			{
				case ENGINE_SPEED[2]:
					//esp_rom_printf("ENGINE_SPEED");
					value = (256 * rx_frame.buffer[3] + rx_frame.buffer[4]) / 4;
					self->m_uart.printf("ENGINE_SPEED=%d", value);
					break;

				case VEHICLE_SPEED[2]:
					value = rx_frame.buffer[3];
					self->m_uart.printf("VEHICLE_SPEED=%d", value);
					break;

				default:
					value = 0;
					self->m_uart.printf("ERR=No matching PID");
					break;
			}
			self->m_uart.printf("\n");
		}
	}
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

	xTaskCreate(rx_task, "obd2_RX", 4096, this, 5, NULL);

  return true;
}

