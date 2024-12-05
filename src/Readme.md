# Software Source Folder

## Folder structure

| **Folder** | **Description** |
|---|---|
| calibration_record | Recorder for the calibration data |
| cube_controller_v1_0 | Final cube controller software project (only MQTT credentials and name needs to be changed for every cube) |
| libraries | Contains addapted libraries |
| manufacturing | Test for tile verification after manufacturing the hardware |
| tile_v1_0 | Final tile software project (only I2C adresses need to be changed for every tile) |

# How to use the software sources

All the software fron this folder was developed within the ArduinoIDE. Therfore the setup instructions also base on this program.

## Setup Arduino IDE:

For the cube controller, we used an ESP32S3 development kit, as well as an XIAOESP32C6. The tile has an RP2040 on it.

### 1) ESP32S3 Development Kit:

- Board "ESP32S3 Dev Module" (https://dl.espressif.com/dl/package_esp32_index.json)
- ESP32 version v3.0.7 & v2.0.7
- PubSubClient library v2.8 (https://github.com/knolleary/pubsubclient?tab=readme-ov-file)
- WiFi library v3.0.7 (https://www.arduino.cc/reference/en/libraries/wifi/)
- Wire library v3.0.7 (https://www.arduino.cc/reference/en/language/functions/communication/wire/)

### 2) XIAOESP32C6:

- Board "XIAOESP32_C6"
- ESP32 version v3.0.7
- PubSubClient library v2.8 (https://github.com/knolleary/pubsubclient?tab=readme-ov-file)
- WiFi library v3.0.7 (https://www.arduino.cc/reference/en/libraries/wifi/)
- Wire library v3.0.7 (https://www.arduino.cc/reference/en/language/functions/communication/wire/)

### 3) RP2040:

- Board "Generic RP2040" (https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
- TS4231 library (https://www.arduino.cc/reference/en/libraries/ts4231-library/) - *care that we made changes in this library an that it has to be exchanged with ours (in library folder)*
- Adafruit NeoPixel library (https://www.arduino.cc/reference/en/libraries/adafruit-neopixel/)
- Wire library (https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- LittleFS library (https://arduino-pico.readthedocs.io/en/latest/fs.html)


## How to setup the system:

- perform calibration_record for every new base stations once and addapt the calibration_data.h file of the final tile software accordingly - if not already done (only required once or if index number of stations gets changed)
- set WIFI_SSID, WIFI_PASSWORD and MQTT_SERVER on cube controller software to connect to right mqtt server where topics cube_position, cube_forward_vector, cube_up_vector and cube_mode get published
- flash the tiles with their individual I2C_ADDRESS_OFFSET (change in config.h) top tile: index 10, front tile: index 11, right tile: index 12, back tile: index 13, left tile: index 14
- configuration via python scripts are now possible (different scripts depending on how to configure) - requirements.txt file contains required pip installations


## Further things to know:

- tiles need to receive light in order to start (sensor activation needs light in setup-phase)
- calibration_record contains older receiving toplogy (optimal for 1m placement)
- pio pins must be in ascending order (pio function can only aquire ascending order pins)

- pioasm.exe in arduino directory for compiling changed or final .pio to header files that later gets compiled to uf2 file for rp
- cmd call: `pioasm.exe differential_manchester.pio differential_manchester.pio.h`

- if configuration square is not measured precisely and is a bit like a paralellogram, z-coordinate of positioning gets worse
- do not copy data encapsulated by `[]` into configuration file (program skips the line then)
- reflections of wall are rare but possible if hardware is too close to it
- spi-slave on rp2040 is broken - does not tristate (free) data line when communication is to other slave (at least for the first version during development)
- use receiving hardware at least 15 cm below station
