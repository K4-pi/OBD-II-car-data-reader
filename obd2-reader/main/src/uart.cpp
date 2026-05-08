#include "../include/uart.h"
#include "freertos/projdefs.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstddef>

extern "C"
{
  #include "soc/gpio_num.h"
  #include "driver/gpio.h"
  #include "driver/uart.h"
  #include "freertos/idf_additions.h"
  #include "hal/uart_types.h"
  #include "esp_err.h"
}

// #define BUFFER_SIZE 1024

Uart::Uart(gpio_num_t tx, gpio_num_t rx, gpio_num_t rts, gpio_num_t cts)
	: m_tx { tx }
	, m_rx { rx }
	, m_rts { rts }
	, m_cts { cts }
{}

void Uart::setup
(
	uart_port_t uart_num,
	int baudrate,
	uart_word_length_t data_bits,
	uart_parity_t parity,
	uart_stop_bits_t stop_bits,
	uart_hw_flowcontrol_t flow_ctrl
	// uint8_t rx_flow_ctrl_thresh
)
{
	m_uart_num = uart_num;

	ESP_ERROR_CHECK(uart_driver_install(m_uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &m_uart_queue, 0));

	m_uart_config = {};
	m_uart_config.baud_rate = baudrate;
	m_uart_config.data_bits = data_bits;
	m_uart_config.parity = parity;
	m_uart_config.stop_bits = stop_bits;
	m_uart_config.flow_ctrl = flow_ctrl;
	// m_uart_config.rx_flow_ctrl_thresh = rx_flow_ctrl_thresh;

	ESP_ERROR_CHECK(uart_param_config(m_uart_num, &m_uart_config));

	ESP_ERROR_CHECK(uart_set_pin(m_uart_num, m_tx, m_rx, m_rts, m_cts));
}

void Uart::printf(const char *fmt, ...) const
{
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  uart_write_bytes(m_uart_num, buf, len);
}

void Uart::read(char *buffer, Uart *args)
{
	std::size_t length = uart_read_bytes(m_uart_num, buffer, 1024 - 1, pdMS_TO_TICKS(1000));

	buffer[length] = '\0';
}
