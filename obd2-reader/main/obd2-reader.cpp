#include "include/uart.h"
#include "include/obd2.h"

#include <cstdint>
#include <cstring>
#include <stdio.h>

extern "C" 
{
	#include "freertos/idf_additions.h"
	#include "freertos/projdefs.h"
	#include "hal/uart_types.h"
	#include "driver/gpio.h"
	#include "soc/gpio_num.h"
	#include "esp_err.h"
}

constexpr gpio_num_t UART_TX = GPIO_NUM_1;
constexpr gpio_num_t UART_RX = GPIO_NUM_3;

constexpr gpio_num_t CAN_TX = GPIO_NUM_17;
constexpr gpio_num_t CAN_RX = GPIO_NUM_16;

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

	Obd2 obd2(CAN_TX, CAN_RX, uart);

	if (!obd2.setup()) uart.send((const uint8_t *)("Failed to initalize CAN driver\n"), 31);
	else uart.send((const uint8_t *)("CAN initialized\n"), 16);

	while (1) 
	{
		vTaskDelay(pdMS_TO_TICKS(750));

		//uint8_t tester_present[] = { 0x02, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		//can.send(0x7DF, tester_present, 8);

		// then send your OBD-II request
		uint8_t send_buf[] = { 0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		obd2.send(0x7DF, send_buf, 8); // 11-bit ide 
		
		// can.send(0x18DB33F1, send_buf, 8); // 29-bit ide
   
	}
}
