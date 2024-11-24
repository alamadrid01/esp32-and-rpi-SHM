# ESP32 IoT-Based Home Automation and Energy Monitoring System  

This project is an IoT-based home automation and energy monitoring system built using an ESP32 microcontroller. It integrates various sensors and relays to automate devices, monitor environmental conditions, and track energy consumption in real-time using the MQTT protocol.  

## Features  

1. **Home Automation**  
   - Control devices such as lights, fans, sockets, and extensions via MQTT messages.  
   - Motion-based automation using a PIR sensor to detect activity and toggle lights automatically.  

2. **Environmental Monitoring**  
   - Measure and publish temperature and humidity data using a DHT11 sensor.  

3. **Energy Monitoring**  
   - Real-time calculation of power consumption for up to 4 devices using ACS712 current sensors.  
   - Publish energy data to the MQTT broker for logging and analysis.  

4. **MQTT Communication**  
   - Publish sensor data (temperature, humidity, motion, power) to specific MQTT topics.  
   - Subscribe to a control topic to receive commands for device management.  

5. **Efficient Power Summation**  
   - Calculate and publish hourly energy usage to the MQTT broker for long-term analysis.  

## Components Used  

- **ESP32**: Microcontroller for data processing and communication.  
- **DHT11 Sensor**: For temperature and humidity measurements.  
- **PIR Sensor**: For motion detection.  
- **ACS712 Current Sensors**: For real-time power consumption monitoring.  
- **Relays**: For controlling devices like lights, fans, and sockets.  
- **Wi-Fi**: For MQTT-based communication.  

## MQTT Topics  

- `home/livingroom/temperature`: Publishes temperature data.  
- `home/livingroom/humidity`: Publishes humidity data.  
- `home/livingroom/motion`: Publishes motion status (1 for detected, 0 for none).  
- `home/livingroom/power_1`: Publishes power consumption of device 1.  
- `home/livingroom/power_2`: Publishes power consumption of device 2.  
- `home/livingroom/power_3`: Publishes power consumption of device 3.  
- `home/livingroom/power_4`: Publishes power consumption of device 4.  
- `home/livingroom/summation`: Publishes accumulated energy usage.  
- `home/livingroom/control`: Subscribed topic for controlling devices.  

## How It Works  

1. **Wi-Fi Setup**  
   - Connects to a Wi-Fi network using SSID and password.  
   - Establishes a connection to the MQTT broker.  

2. **Data Collection**  
   - Reads temperature and humidity from the DHT11 sensor.  
   - Measures current and calculates power usage for connected devices.  
   - Detects motion using the PIR sensor.  

3. **Data Publishing**  
   - Publishes sensor readings and power consumption to corresponding MQTT topics.  
   - Sends motion status updates and hourly energy consumption to the MQTT broker.  

4. **Device Control**  
   - Listens to commands from the MQTT broker to toggle devices via relays.  

## Installation  

1. Clone the repository:  
   ```bash  
   git clone https://github.com/yourusername/esp32-iot-home-automation.git  
   cd esp32-iot-home-automation  
   ```  

2. Install required libraries in Arduino IDE:  
   - [Adafruit Sensor Library](https://github.com/adafruit/Adafruit_Sensor)  
   - [DHT Sensor Library](https://github.com/adafruit/DHT-sensor-library)  
   - [PubSubClient Library](https://github.com/knolleary/pubsubclient)  

3. Configure the code:  
   - Replace Wi-Fi credentials (`ssid` and `wifi_password`) with your network details.  
   - Replace `mqtt_server` and `mqtt_port` with your MQTT broker details.  

4. Upload the code to ESP32 using Arduino IDE.  

## Usage  

- **Monitoring**: YOu can connect this response to Grafana or Node red graph  
- **Automation**: Send MQTT commands to control devices. Example commands:  
  - `"LIGHT"` to turn on the light.  
  - `"OFFLIGHT"` to turn off the light.  
  - `"FAN"` to turn on the fan.  
  - `"OFFFAN"` to turn off the fan.  


## Future Enhancements  

- Integrate AI for power recommendation usage


------
Feel free to replace the placeholder links or customize further!
