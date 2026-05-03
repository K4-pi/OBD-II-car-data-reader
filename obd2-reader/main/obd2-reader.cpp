#include "include/uart.h"
#include "include/obd2.h"
#include "include/pids_obd2.h"

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

	if (!obd2.setup()) uart.printf("Failed to initalize CAN driver\n");
	else uart.printf("CAN initialized\n");

	while (1)
	{
		vTaskDelay(pdMS_TO_TICKS(500));

		for (int i = 0; i < sizeof(PIDS_CALLS)/sizeof(PIDS_CALLS[0]); i++)
		{
			obd2.send(PIDS_CALLS[i], 8);
		}
	}
}
