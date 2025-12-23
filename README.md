Biometric Fitness Tracker
A compact, embedded fitness tracking device that monitors both physical activity (step counting) and cardiovascular health (heart rate monitoring) with real-time LCD display feedback.
Project Overview
This project implements a wearable fitness tracker using low-cost microcontrollers (ESP8266/RP2040) with integrated sensors for step detection and pulse monitoring. The device provides real-time biometric data visualization through an LCD interface, making it suitable for personal fitness tracking and health monitoring applications.
Features

Step Counter: Accurate pedometer functionality using accelerometer-based motion detection
Heart Rate Monitor: Real-time pulse rate measurement using optical pulse sensor
LCD Display: 16x2 LCD interface showing steps and heart rate simultaneously
Multi-Platform Support: Compatible with both ESP8266 (NodeMCU) and RP2040 (Raspberry Pi Pico)
Low Power Design: Optimized for portable, battery-operated use
Data Processing: Python-based analytics for historical data tracking

Hardware Requirements
Components

Microcontroller: ESP8266 (NodeMCU) or RP2040 (Raspberry Pi Pico)
Sensors:

ADXL345 or MPU6050 Accelerometer (for step counting)
MAX30100/MAX30102 Pulse Oximeter Sensor (for heart rate)


Display: 16x2 I2C LCD Module
Power Supply: 5V USB or 3.7V Li-Po battery with voltage regulator
Miscellaneous: Jumper wires, breadboard/PCB, resistors, capacitors

Pin Connections
Refer to Schematic_Step_Counter_Prototype_mk2_2025-04-14.png for detailed circuit connections.
Software Requirements

Embedded Development:

Arduino IDE (for ESP8266) or Platform IO
Raspberry Pi Pico SDK (for RP2040)


Python 3.x (for data processing script)
Libraries:

Wire.h (I2C communication)
LiquidCrystal_I2C.h (LCD control)
Adafruit sensor libraries (accelerometer, pulse sensor)



Installation & Setup
1. Hardware Assembly

Connect the accelerometer to I2C pins (SDA, SCL) of your microcontroller
Connect the pulse sensor to the ADC pin and I2C pins
Wire the LCD module to I2C pins (same bus as sensors)
Provide appropriate power supply (3.3V or 5V depending on components)
Refer to the schematic diagram for complete wiring details

2. Software Setup
For ESP8266:
# Install Arduino IDE and ESP8266 board support
# Open pulse+LCD (ESP8266) file in Arduino IDE
# Select Board: NodeMCU 1.0 (ESP-12E Module)
# Select appropriate COM port
# Upload the code

For RP2040:
# Install Raspberry Pi Pico SDK
# Compile and flash using:
cd DSD_PROJ
mkdir build && cd build
cmake ..
make
# Copy .uf2 file to Pico in bootloader mode

Usage

Power On: Connect the device to power source (USB or battery)
Initialization: LCD will display "Fitness Tracker" during startup
Normal Operation:

Top row shows: Steps: XXXX
Bottom row shows: HR: XXX bpm


Wear Device: Attach securely to wrist or ankle for accurate readings
Data Logging: Connect via USB to computer and run main.py for data visualization

How It Works
Step Detection Algorithm:

Continuously samples accelerometer data
Calculates magnitude of acceleration vector
Applies peak detection with adaptive threshold
Filters false positives using time-gating

Heart Rate Measurement:

Optical sensor detects blood volume changes
Signal processing identifies heartbeat peaks
Calculates BPM using inter-beat intervals
Applies moving average filter for stability

Technical Highlights

Real-time Signal Processing: Efficient filtering algorithms for noise reduction
Multi-platform Firmware: Portable code architecture supporting multiple MCUs
I2C Protocol Implementation: Multi-device communication on shared bus
Interrupt-driven Design: Low-latency sensor data acquisition
Python Data Analytics: Statistical analysis and visualization capabilities

Future Enhancements

 Add Bluetooth connectivity for smartphone app integration
 Implement calorie estimation algorithm
 Add sleep tracking functionality
 Design custom PCB for compact form factor
 Integrate GPS for distance tracking
 Add water resistance enclosure

Author
Ramakrishnan S

GitHub: @Ramakrishnan-S7
