#include "../include/obd2.h"
#include "../include/uart.h"

#include <cstdint>

extern "C"
{
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

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
  uint8_t recv_buff[8];

  twai_frame_t rx_frame = {};
  rx_frame.buffer = recv_buff;
  rx_frame.buffer_len = 8;

	Obd2 *self = static_cast<Obd2 *>(user_ctx);

	if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {

		if (rx_frame.header.id == 0x45A || rx_frame.header.id == 0x45E) return false;

		ESP_DRAM_LOGI("CAN", "ID: 0x%03lX len: %d", rx_frame.header.id, rx_frame.buffer_len);
   
		for (int i = 0; i < rx_frame.buffer_len; i++) {
        esp_rom_printf("%02X ", recv_buff[i]);
    }
    esp_rom_printf("\n");
  }
	 
  return true;
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
	user_cbs.on_rx_done = twai_rx_cb;

	ESP_ERROR_CHECK(twai_node_register_event_callbacks(m_node_hdl, &user_cbs, this));

	// Start TWAI controller
	ESP_ERROR_CHECK(twai_node_enable(m_node_hdl));	

  return true;
}

void Obd2::send(uint32_t id, uint8_t *buffer, uint8_t len)
{
	// ESP_LOGI("CAN", "sending ID: 0x%03lX", id);

	twai_frame_t msg = {};
	msg.header.id = id;
	msg.header.ide = false; // true = 29-bit, false = 11-bit
	msg.buffer = buffer;
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

