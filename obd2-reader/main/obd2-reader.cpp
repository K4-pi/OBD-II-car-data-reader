#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "include/uart.h"

#include <cstring>
#include <stdio.h>

extern "C" 
{
  #include "hal/uart_types.h"
	#include "driver/gpio.h"
  #include "soc/gpio_num.h"
	#include "esp_err.h"
}

constexpr gpio_num_t UART_TX = GPIO_NUM_1;
constexpr gpio_num_t UART_RX = GPIO_NUM_3;

extern "C"
void app_main(void)
{
	Uart uart(UART_TX, UART_RX);

	uart.setup
	(
		UART_NUM_0,
		115200,
		UART_DATA_8_BITS,
		UART_PARITY_DISABLE,
		UART_STOP_BITS_1,
		UART_HW_FLOWCTRL_DISABLE, // No RTS, CTS pins needed 
		122
	);

	// char buf[128];

	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));

		// uart.receive(buf);
		// uart.send((const char*)buf);

		uart.send("UART test!\n");
	}
}
