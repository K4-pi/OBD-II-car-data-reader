#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "include/uart.h"
#include "include/obd2.h"
#include "include/pids_obd2.h"
#include "include/gps.h"

#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C"
{
	#include "freertos/idf_additions.h"
	#include "freertos/projdefs.h"
	#include "hal/uart_types.h"
	#include "driver/gpio.h"
	#include "soc/gpio_num.h"
	#include "esp_err.h"
}

#define GPS_BUFFER_SIZE 1024

constexpr gpio_num_t UART_TX = GPIO_NUM_1;
constexpr gpio_num_t UART_RX = GPIO_NUM_3;

constexpr gpio_num_t UART_GPS_TX = GPIO_NUM_17;
constexpr gpio_num_t UART_GPS_RX = GPIO_NUM_16;

constexpr gpio_num_t CAN_TX = GPIO_NUM_21;
constexpr gpio_num_t CAN_RX = GPIO_NUM_22;

extern "C"
void app_main(void)
{
	Uart uart_pc(UART_TX, UART_RX);
	Uart uart_gps(UART_GPS_RX, UART_GPS_TX);

	uart_pc.setup
	(
		UART_NUM_0,
		115200,
		UART_DATA_8_BITS,
		UART_PARITY_DISABLE,
		UART_STOP_BITS_1,
		UART_HW_FLOWCTRL_DISABLE // No RTS, CTS pins needed
		// 122
	);

	uart_gps.setup
	(
		UART_NUM_2,
		9600,
		UART_DATA_8_BITS,
		UART_PARITY_DISABLE,
		UART_STOP_BITS_1,
		UART_HW_FLOWCTRL_DISABLE // No RTS, CTS pins needed
		// 122
	);

	Obd2 obd2(CAN_TX, CAN_RX, uart_pc);

	if (!obd2.setup()) uart_pc.printf("Failed to initalize CAN driver\n");
	else uart_pc.printf("CAN initialized\n");

	char gps_buffer[GPS_BUFFER_SIZE];

	for (int i = 0; i < ONE_TIME_PIDS_SIZE; i++)
    {
    	obd2.send(ONE_TIME_PIDS[i], 8);
    	vTaskDelay(pdMS_TO_TICKS(100));
    }

	while (1)
	{
        for (int i = 0; i < PIDS_CALLS_SIZE; i++)
    	{
    		obd2.send(PIDS_CALLS[i], 8);
    		vTaskDelay(pdMS_TO_TICKS(100));
    	}

        memset(gps_buffer, 0, GPS_BUFFER_SIZE); // zeroing buffer

        uart_gps.read(gps_buffer, &uart_pc);

        gps_gngll(gps_buffer, &uart_pc);
	}
}
