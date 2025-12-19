# Smart-Home-Hub
Implemented a modular IoT smart home system using a TM4C123GH6PM microcntroller as a wireless sensor node and an ESP32 Wi-Fi gateway to monitor environmental and security data and publish it to the cloud via MQTT.

<p align="center">
  <img src="Images/TM4C123 Wireless Node.jpeg" width="45%" />
  <img src="Images/ESP32 Wi-fi Gateway.jpeg" width="45%" />
</p>

# Parts
|                                |
|--------------------------------|
| TM4C123GH6PM microcontroller |
| ESP32 microcontroller |
| BME280 sensor |
| OLED Display |
| LDR (Light Dependent Resistor) |
| Servo Motor |
| 4x4 Keypad |
| PIR sensor |
| Speaker |
| nrf24l01+ wireless module (2) |
| White LED (1) |

# Peripherals/Software
|           |
|-----------|
| GPIO |
| UART |
| I2C |
| SPI |
| PWM |
| TIMERS |
| FreeRTOS |

# Features
* `RTOS`: TM4C123 wireless sensor node tasks managed using FreeRTOS. Also used to monitor system status. 
* `Environmental Monitoring`: Monitors temperature, humidity, atmospheric pressure, and ambient light. Sensor readings are processed locally and transmitted using an nrf24l01+ wireless module.
* `Security Monitoring`: checks for motion detection and door status. Door opens when password inputted using 4x4 keypad is correct. If password is incorrect after three attempts, door remains closed and triggers an alarm. Servo motor used for simulation of door opening and closing.
* `Automatic Lights`: A white LED is used for turining on lights only when it's dark and there's motion detected. Brightness and motion detection are determined using an LDR and PIR sensor respectively. Lights (LED) turn off after 3 minutes if there's no motion detected.
* `Wi-fi and MQTT Gateway`: utilizes ESP32 as a Wi-fi gateway for connecting to a local Wi-fi network and publish data to an MQTT Broker (HiveMQ).
* `Mobile Data Visualization (iOS/Android)`: The data can be visualized remotely using mobile or web-based MQTT dashboards such as IoT MQTT Panel. The following picture represents data being visualized using the IoT MQTT Panel mobile application on an Iphone.

![IoT MQTT Panel ](https://github.com/user-attachments/assets/eb1b02e7-81f8-4447-8138-1e1d1948a48f)

# MQTT Topics
* `home/temperature`: topic for measured temperature.
* `home/humidity`: topic for measured humidity.
* `home/pressure`: topic for measured atmospheric pressure.
* `home/motion/status`: topic for monitoring motion detection.
* `home/door/status`: topic for monitoring door status whether it's locked or unlocked.
* `home/brightness`: topic for detecting whether it's bright or dark using LDR.
