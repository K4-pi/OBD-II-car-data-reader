#ifndef UART_H
#define UART_H

#include <cstdint>
#include <cstddef>

extern "C"
{
	#include "soc/gpio_num.h"
	#include "driver/gpio.h"
	#include "driver/uart.h"
	#include "freertos/idf_additions.h"
	#include "hal/uart_types.h"
}

class Uart
{
	private:
		static constexpr int UART_BUFFER_SIZE = 2048;
		gpio_num_t m_tx, m_rx, m_rts, m_cts;

		QueueHandle_t m_uart_queue;
		uart_port_t m_uart_num;
		uart_config_t m_uart_config;

	public:
		Uart
		(
			gpio_num_t tx,
			gpio_num_t rx,
			gpio_num_t rts = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE),
			gpio_num_t cts = static_cast<gpio_num_t>(UART_PIN_NO_CHANGE)
		);

		void setup
		(
			uart_port_t uart_num,
			int baudrate,
			uart_word_length_t data_bits,
			uart_parity_t parity,
			uart_stop_bits_t stop_bits,
			uart_hw_flowcontrol_t flow_ctrl
			// uint8_t rx_flow_ctrl_thresh
		);
		void printf(const char *fmt, ...) const;
		void read(char *buffer, Uart *args);
};

#endif
