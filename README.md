# ESP32 OBD-II Car Data Reader

## Description

This project is an OBD-II (On-Board Diagnostics) data reader built for the ESP32 platform. It communicates with a vehicle's ECU (Engine Control Unit) over the CAN bus using ESP32's TWAI (Two-Wire Automotive Interface) controller to read real-time vehicle diagnostics data.

## What It Does

The ESP32 OBD-II reader performs the following functions:

- Initializes UART communication for serial output and debugging
- Sets up TWAI/CAN bus communication with the vehicle's OBD-II port
- Automatically detects the correct CAN protocol configuration by testing different combinations of:
  - Identifier types (11-bit standard vs 29-bit extended)
  - Bitrates (500kbps vs 250kbps)
- Periodically sends OBD-II PID requests to the vehicle's ECU
- Receives and decodes responses for various vehicle parameters
- Outputs real-time vehicle data via UART in a readable format

### Data Parameters Read

| Parameter | PID | Description |
|-----------|-----|-------------|
| ENGINE_SPEED | 0x0C | Engine RPM (revolutions per minute) |
| ENGINE_LOAD | 0x04 | Engine load value (%) |
| VEHICLE_SPEED | 0x0D | Vehicle speed (km/h) |
| COOLANT_TEMP | 0x05 | Engine coolant temperature (°C) |
| ENGINE_OIL_TEMP | 0x5C | Engine oil temperature (°C) |
| FUEL_TRIM_STFT1 | 0x06 | Short term fuel trim bank 1 (%) |
| FUEL_TRIM_STFT2 | 0x08 | Short term fuel trim bank 2 (%) |
| FUEL_TRIM_LTFT1 | 0x07 | Long term fuel trim bank 1 (%) |
| FUEL_TRIM_LTFT2 | 0x09 | Long term fuel trim bank 2 (%) |
| ENGINE_PERECENT_TORQUE | 0x64 | Engine percent torque |
| Trouble Codes | 0x03 | Diagnostic trouble codes (DTCs) |

## Core Functions

### Uart Class (`uart.h`, `uart.cpp`)
Handles UART serial communication for outputting diagnostic data.

- **`Uart(gpio_num_t tx, gpio_num_t rx, ...)`** - Constructor that initializes UART pin configuration
- **`setup(uart_port_t, int baudrate, ...)`** - Configures UART driver with specified parameters (baud rate, data bits, parity, stop bits, flow control)
- **`printf(const char *fmt, ...)`** - Formatted print to UART output
- **`read(uint8_t *buffer)`** - Reads buffered data from UART

### Obd2 Class (`obd2.h`, `obd2.cpp`)
Manages OBD-II communication over the CAN bus using ESP32's TWAI controller.

- **`Obd2(gpio_num_t tx, gpio_num_t rx, Uart uart)`** - Constructor that sets up CAN TX/RX pins and UART reference
- **`setup()`** - Initializes the TWAI/CAN driver, registers callbacks, creates receive queue and task
- **`send(const uint8_t *buffer, uint8_t len)`** - Sends OBD-II request frames to the ECU with proper ID (11-bit or 29-bit)
- **`initialize_twai_frame_conf()`** - Auto-detects CAN protocol by cycling through bitrate and identifier type combinations
- **`update_bitrate(uint32_t bitrate)`** - Dynamically changes CAN bus bitrate by reinitializing the TWAI driver

### FreeRTOS Tasks and Callbacks

- **`rx_task(void *arg)`** - FreeRTOS task that continuously receives CAN frames from a queue and decodes OBD-II responses, outputting formatted data via UART
- **`twai_rx_callback()`** - Interrupt service routine callback that receives CAN frames and posts them to the receive queue

### Main Application (`obd2-reader.cpp`)

- **`app_main()`** - Entry point that:
  - Initializes UART on GPIO 1 (TX) and GPIO 3 (RX) at 115200 baud
  - Sets up TWAI/CAN on GPIO 17 (TX) and GPIO 16 (RX)
  - Continuously sends OBD-II PID requests every 500ms

## Hardware Configuration

| Interface | GPIO Pin | Function |
|-----------|----------|----------|
| UART | GPIO 1 | TX (transmit to serial) |
| UART | GPIO 3 | RX (receive from serial) |
| CAN/TWAI | GPIO 17 | TX (to OBD-II port) |
| CAN/TWAI | GPIO 16 | RX (from OBD-II port) |

## Libraries and Dependencies

### ESP-IDF Framework
This project uses the ESP-IDF (Espressif IoT Development Framework) and the following components:

| Library | Purpose |
|---------|---------|
| `driver/uart.h` | UART communication driver |
| `driver/gpio.h` | GPIO pin control |
| `esp_twai.h` | TWAI/CAN controller driver (high-level API) |
| `esp_twai_onchip.h` | On-chip TWAI controller implementation |
| `esp_twai_types.h` | TWAI type definitions |
| `freertos/FreeRTOS.h` | FreeRTOS real-time operating system |
| `freertos/task.h` | FreeRTOS task creation and management |
| `freertos/queue.h` | FreeRTOS queue for inter-task communication |
| `freertos/projdefs.h` | FreeRTOS project definitions |

### Standard C/C++ Libraries
- `<cstdint>` - Fixed-width integer types
- `<cstddef>` - Standard definitions (size_t, nullptr_t)
- `<cstdio>` - Standard I/O functions (vsnprintf)

## Building and Flashing

This project uses ESP-IDF build system with CMake. To build and flash:

```bash
cd obd2-reader
idf.py build
idf.py flash
```
