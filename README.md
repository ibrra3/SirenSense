# SirenSense: early embedded prototype

[![Platform](https://img.shields.io/badge/platform-ESP32--S3-2C3E50)](https://www.espressif.com/en/products/socs/esp32-s3)
[![Framework](https://img.shields.io/badge/framework-PlatformIO%20%2B%20Arduino-2D8C8C)](https://platformio.org/)
[![Edge AI](https://img.shields.io/badge/edge%20AI-Edge%20Impulse-4B5563)](https://www.edgeimpulse.com/)

SirenSense is a driver-assistance project that detects emergency-vehicle sirens locally and warns the driver about their direction. Processing runs on the device, without requiring a cloud connection.

This repository preserves the **first working firmware prototype**. It combines four digital microphones, an Edge Impulse audio classifier, an ESP32-S3 and a small OLED interface. The project later evolved into a larger multi-node system with synchronized audio acquisition, GCC-PHAT direction finding, custom PCBs and a Raspberry Pi driver HUD.

Current project overview: [sirensense.es](https://sirensense.es)

Engineering case study: [SirenSense V2](https://github.com/ibrra3/SirenSense-V2)

Prototype video: [YouTube](https://www.youtube.com/watch?v=NpQYkkEUuDs)

## What this prototype demonstrates

* Continuous I2S audio capture from four INMP441 microphones
* On-device siren classification with an Edge Impulse model
* Use of ESP32-S3 PSRAM for audio buffers and model execution
* A 360-degree directional interface on an SSD1306 OLED
* Startup microphone diagnostics and persistent driver alerts
* A reproducible PlatformIO build for the ESP32-S3-N16R8

## Repository status

This is a historical prototype, not the current production architecture. Its energy-based direction estimate and independently configured I2S peripherals were useful for initial validation, but they were superseded by synchronized dual-I2S acquisition and GCC-PHAT/TDOA processing in the later system.

Keeping that distinction explicit is important: the code here documents an engineering iteration rather than claiming to be the finished product.

## Hardware used

| Part | Role |
| --- | --- |
| ESP32-S3 DevKitC, N16R8 | Main MCU with 16 MB flash and 8 MB PSRAM |
| 4 x INMP441 | Digital I2S microphones |
| SSD1306 OLED | Detection and direction display |
| Status LED | Visual warning output |

## Build

1. Install [PlatformIO](https://platformio.org/).
2. Clone this repository.
3. Open the project directory in VS Code or PlatformIO Core.
4. Connect an ESP32-S3-N16R8 board.
5. Build and upload:

```bash
pio run
pio run --target upload
pio device monitor
```

The pin mapping and library dependencies are defined in `platformio.ini` and `src/main.cpp`. Verify the microphone wiring against your own board before powering the array.

## Engineering lessons carried into V2

* Two microphones can share one I2S data line by using the INMP441 left and right channel slots.
* Direction finding requires deterministic timing between microphone channels. Independent peripheral startup is not sufficient.
* A system should suppress a low-quality bearing instead of displaying a confident but incorrect direction.
* Audio, power integrity and PCB layout have to be treated as one system; firmware alone cannot compensate for a noisy microphone rail.

## Recognition

SirenSense received first prize at Hack4Change 2026 at the ETSII, Universidad de Sevilla.

## Safety notice

SirenSense is an experimental prototype. It is not an automotive-certified safety device and must not replace normal observation, emergency-vehicle awareness or certified vehicle equipment.
