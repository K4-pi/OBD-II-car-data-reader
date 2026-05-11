#include "driver/uart.h"
#include "hal/gpio_types.h"
#include "include/uart.h"
#include "include/obd2.h"
#include "include/pids_obd2.h"

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

constexpr gpio_num_t UART_TX = GPIO_NUM_1;
constexpr gpio_num_t UART_RX = GPIO_NUM_3;

constexpr gpio_num_t UART_GPS_TX = GPIO_NUM_17;
constexpr gpio_num_t UART_GPS_RX = GPIO_NUM_16;

constexpr gpio_num_t CAN_TX = GPIO_NUM_21;
constexpr gpio_num_t CAN_RX = GPIO_NUM_22;

static float geo_to_float(char *str) // TODO: fix, wrong numbers
{
    float val = std::atof(str);
	int deg = (int)val / 100;
	float min = val - deg * 100;

	return deg + min / 60.0f;
}

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

	char gps_buffer[1024];

	for (int i = 0; i < 2; i++)
    {
    	obd2.send(ONE_TIME_PIDS[i], 8);
    	vTaskDelay(pdMS_TO_TICKS(100));
    }
	
	while (1)
	{
        for (int i = 0; i < 10; i++)
    	{
    		obd2.send(PIDS_CALLS[i], 8);
    		vTaskDelay(pdMS_TO_TICKS(100));
    	}

		memset(gps_buffer, 0, 1024); // zeroing buffer

		uart_gps.read(gps_buffer, &uart_pc);

		char *gngll = strstr(gps_buffer, "$GNGLL,");

		if (gngll)
		{
		    char *end = strchr(gngll, '\n');
			if (end) *end = '\0';

			char *s_ptr = strtok(gngll, ",");

			std::vector<char *> params;

			while (s_ptr)
			{
			    params.push_back(s_ptr);
                s_ptr = strtok(NULL, ",");
			}

			if (params.size() > 3)
			{
				float lat = geo_to_float(params.at(1));
				float lng = geo_to_float(params.at(3));

				uart_pc.printf("GPS=%f,%f\n", lat, lng);
			}
		}
	}
}
