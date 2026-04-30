#ifndef OBD2_H
#define OBD2_H

#include "uart.h"

#include <cstdint>

extern "C"
{
  #include "esp_err.h"
  #include "freertos/FreeRTOS.h"
  #include "driver/gpio.h"
	#include "soc/gpio_num.h"
  #include "esp_twai.h" 
  #include "esp_twai_onchip.h"
}

class Obd2 
{
  private:
    gpio_num_t m_tx;
    gpio_num_t m_rx;

    bool m_header_ide;

		twai_onchip_node_config_t m_node_config;
		twai_node_handle_t m_node_hdl;

    esp_err_t update_bitrate(uint32_t bitrate);

  public:
		Uart m_uart;
    bool m_initialized;

    QueueHandle_t m_obd2_rx_queue_hdl;

		Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart);

    void initialize_twai_frame_conf();

    bool setup();

		void send(uint32_t id, const uint8_t *buffer, uint8_t len);
};

#endif
