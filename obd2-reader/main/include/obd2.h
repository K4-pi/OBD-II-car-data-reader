#ifndef OBD2_H
#define OBD2_H

#include "uart.h"

#include <cstdint>

extern "C"
{
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

		twai_onchip_node_config_t m_node_config;
		twai_node_handle_t m_node_hdl;

  public:
		Uart m_uart;
    QueueHandle_t m_obd2_rx_queue_hdl;

		Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart);

    bool setup();

		void send(uint32_t id, const uint8_t *buffer, uint8_t len);
};

#endif
