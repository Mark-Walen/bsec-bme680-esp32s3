# BSEC IAQ Application for ESP32-S3

This project is an implementation of an Indoor Air Quality (IAQ) application using the Bosch Sensortec Environmental Cluster (BSEC) library on the ESP32-S3 microcontroller.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

## Introduction

This project combines the power of the BSEC library for accurate IAQ monitoring with the flexibility of MQTT for seamless data communication. Monitor and share IAQ data across devices using the ESP32-S3 microcontroller.

## Prerequisites

- [Visual Studio Code](https://code.visualstudio.com/)
- [Espressif IDF Plugin](https://marketplace.visualstudio.com/items?itemName=espressif.esp-idf-extension)
- Microcontroller ESP32S3
- [ESP IDF](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/index.html)
- BME680
- BSEC library v2.4.0.0

## Installation

Step-by-step instructions on how to install and set up your project using Visual Studio Code and the Espressif IDF Plugin.

1. Install Visual Studio Code from [https://code.visualstudio.com/](https://code.visualstudio.com/).
2. Set up the software development environment for the hardware based on the ESP32-S3 chip.
3. Install the Espressif IDF Plugin from the Extensions sidebar.
4. Clone the repository: `git clone https://github.com/Mark-Walen/bsec-bme680-esp32s3 --recursive` .
5. Open the project in Visual Studio Code.
6. Build and flash the code to your ESP32-S3.
7. Flash


## Usage

1. Connect the ESP32-S3 to your computer.
2. Open the Serial Monitor in Visual Studio Code.
3. Monitor the IAQ values and other relevant data from the BME680 sensor.
4. Set up an MQTT broker to receive and publish IAQ data.
5. Configure MQTT parameters in the code.
6. Observe data being transmitted via MQTT.

## Configuration

![image-20231226170550136](D:\git\esp32-s3\bsec-bme680-esp32\img\project_config.png)

Use ESP-IDF Terminal to config WiFi SSID m, WiFi Password and MQTT Broker URI.

```shell
idf.py menuconfig # Select BME680 Configuration
```

## TODO

- [ ] Use MQTTs
- [ ] Auto reconnection after WiFi diconnect

## Contributing

Provide guidelines for others who want to contribute to your project using Visual Studio Code, the Espressif IDF Plugin, MQTT, the BME680 sensor, and the submodule dependency.

1. Fork the project.
2. Create a new branch.
3. Make your changes and commit them.
4. Submit a pull request.

## License

This project is licensed under the MIT License - see the [LICENSE](https://github.com/Mark-Walen/bsec-bme680-esp32s3/LICENSE) file for details.
