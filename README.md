# espHosted


# Wi-Fi connectivity Setup over SPI
## 1. Setup

| Supported Targets | ESP32 | ESP32-S2 | ESP32-S3 | ESP32-C2 | ESP32-C3 | ESP32-C6 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

### 1.1 Hardware Setup/Connections
* In this setup, ESP board acts as a SPI peripheral and provides Wi-Fi capabilities to host. Please connect ESP board to STM32F769I board's CN12 Extension connecter with jumper cables as mentioned below. It may be good to use small length cables to ensure signal integrity.
* STM32F769I should be powered with correct incoming power rating. ESP peripheral can be powered through PC using micro-USB/USB-C cable. STM32 can be powered with mini-B cable. It is also used as USART connection for debug logs from host. Serial port communicaton program like tera term or minicom used to print the logs.
* BT/BLE support will be added in upcoming release.

#### Hardware connections for ESP32
| STM32 Pin | ESP32 Pin | ESP32-S2/S3 | ESP32-C2/C3/C6 | Function |
|:----------:|:---------:|:--------:|:--------:|:--------:|
| PB14 (D12) | IO19 | IO13 | IO2 | MISO |
| PA12 (D13) | IO18 | IO12 | IO6 | CLK  |
| PB15 (D11) | IO23 | IO11 | IO7 | MOSI |
| PA11 (D10) | IO5 | IO10 | IO10 | CS |
| GND (pin2) | GND | GND | GND | GND |
| PJ1 (D2) | IO2 | IO2 | IO3 | Handshake |
| PJ3 (D7) | IO4 | IO4 | IO4 | Data Ready from ESP |
| PJ0 (D4) | EN | RST | RST | Reset ESP |

- Use good quality extremely small (smaller than 10cm) jumper wires, all equal length
- Optionally, Add external pull-up of min 10k Ohm on CS line just to prevent bus floating
- In case of ESP32-S3, For avoidance of doubt, You can power using [UART port](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/hw-reference/esp32s3/user-guide-devkitc-1.html#description-of-components)

