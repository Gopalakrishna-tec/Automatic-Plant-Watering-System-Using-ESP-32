# Automatic Plant Watering System Using ESP32

## Overview
The Automatic Plant Watering System Using ESP32 is an IoT-based smart irrigation solution designed to monitor soil moisture and automatically control a water pump. The system helps conserve water, reduces manual effort, and ensures plants receive adequate water based on real-time soil conditions.

## Features
- Automatic soil moisture monitoring
- Automatic water pump control
- Rain detection to prevent overwatering
- Temperature and humidity monitoring using DHT11
- Real-time clock (RTC) based scheduling
- OLED display for live system status
- GSM SIM900A SMS notifications and remote control
- Low-cost and energy-efficient design

## Hardware Components
- ESP32-WROOM-32
- Soil Moisture Sensor
- Rain Sensor
- DHT11 Temperature & Humidity Sensor
- SIM900A GSM Module
- DS1302 RTC Module
-  OLED Display 
- 5V Relay Module
- Water Pump
- LM2596 Power Supply Module

## System Architecture
The ESP32 collects data from the soil moisture, rain, and DHT11 sensors. Based on the sensor readings and irrigation logic, the ESP32 controls a relay that switches the water pump ON or OFF. System status is displayed on the OLED screen, while SMS alerts are sent through the GSM module.

## Pin Configuration

| Component | ESP32 GPIO |
|------------|------------|
| DHT11 | GPIO 25 |
| Relay | GPIO 26 |
| Soil Moisture Sensor | GPIO 32 |
| Rain Sensor | GPIO 33 |
| GSM RX | GPIO 16 |
| GSM TX | GPIO 17 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |
| RTC SDA | GPIO 21 |
| RTC SCL | GPIO 22 |

## Working Principle
1. Read soil moisture level.
2. Detect rainfall status.
3. Monitor temperature and humidity.
4. Compare moisture level with predefined thresholds.
5. Turn the pump ON when soil becomes dry.
6. Turn the pump OFF when sufficient moisture is reached or rain is detected.
7. Display real-time information on the OLED screen.
8. Send SMS alerts through the GSM module.

## Applications
- Agriculture
- Smart Farming
- Greenhouses
- Home Gardens
- Plant Nurseries

## Future Enhancements
- Mobile App Integration
- Cloud Data Logging
- Weather Forecast Integration
- Solar Power Support
- AI-Based Irrigation Scheduling

## Safety Precautions

⚠️ **Warning: This project uses AC mains power as the input source. Improper handling can cause electric shock, fire, equipment damage, or serious injury.**

### Precautions
- Always switch OFF the AC supply before wiring or maintenance.
- Use a properly rated and isolated AC-to-DC power supply module.
- Never touch exposed AC wires while the system is powered.
- Ensure all AC connections are insulated and enclosed in a protective case.
- Use appropriate fuses, circuit breakers, and earthing/grounding for safety.
- Keep the system away from water, moisture, and wet surfaces.
- Verify voltage levels before connecting components to the ESP32.
- Use relays rated for the pump's voltage and current requirements.
- Test the circuit with low voltage before connecting the AC-powered pump.
- Children and untrained users should not operate or service the system.

### Disclaimer
This project is intended for educational and research purposes. The authors are not responsible for any damage, injury, or loss resulting from improper assembly, installation, or use of this system.

## Project Team

### Team Members
1. **H. Gopalakrishna Prabhu**
2. **D. Chinmayraju** 
3. **Deekshith** 
4. **Deekshitha N**
5. **Deepika D** 

KVG College of Engineering, Sullia, Karnataka, India

## License
This project is released under the MIT License.

---
🌱 Smart Irrigation for Efficient Water Management using ESP32
