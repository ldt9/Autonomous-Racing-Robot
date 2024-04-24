# Autonomous Robot with MQTT Communication
ECE 1188 Cyber Physical Systems Autonomous Racing Challenge Repository

## Overview
This is an autonomous robot capable of following walls using proximity sensors and controlled by MQTT messages. This README provides instructions on how to set up and the requirements to run the project.

## Features
- Autonomous wall-following behavior
- MQTT communication for remote control and status updates
- Integration with proximity sensors for obstacle detection
- Real-time monitoring of motor RPMs

## Hardware Requirements
- **Robot Chassis**: A robot chassis equipped with two DC motors and proximity sensors.
- **CC3100 Wi-Fi Module**: For MQTT communication.
- **Reflectance Sensors**: To detect the presence of a wall.
- **Bump Sensors**: For manual control and emergency stop.
- **Tachometer Sensors**: To measure motor RPMs.
- **Distance Sensor**: For accurate distance measurements.

## Software Requirements
- **TI-RTOS**: Real-time operating system for embedded systems.
- **SimpleLink Wi-Fi CC3100/CC3200 SDK**: SDK for Wi-Fi module.
- **MQTT Client Library**: For MQTT communication.
- **Optical Distance Sensor Library**: For distance measurements.
- **Energia IDE**: For programming the microcontroller.

## Setup Instructions
1. **Hardware Assembly**:
   - Assemble the robot chassis with the motors, sensors, and Wi-Fi module according to the provided instructions.

2. **Software Installation**:
   - Install Code Composer Studio IDE on your development machine.
   - Set up a server on Azure to host the NodeRed application to deploy the flow for the robot's dashboard
   - Install the MQTT client library and Optical Distance Sensor library.

3. **Project Configuration**:
   - Configure the project settings in Energia IDE to use the appropriate board and compiler options.
   - Make sure to include all necessary libraries and dependencies in your project.

4. **Connectivity Setup**:
   - Configure the Wi-Fi module to connect to your local Wi-Fi network. Update the SSID and password in the source code.

5. **Compile and Upload**:
   - Compile the source code in Code Composer Studio IDE.
   - Upload the compiled binary to the microcontroller.

6. **Run the Program**:
   - Power on the robot and monitor the serial output for debugging information.
   - Use an MQTT client to send commands to control the robot remotely.

## Usage
- Use the provided MQTT topics to send commands and receive status updates from the robot.
- Monitor the serial output for debugging information and sensor readings.

## Troubleshooting
- If the robot is not responding to MQTT commands, check the Wi-Fi connectivity and MQTT broker configuration.
- Verify the sensor connections and calibration for accurate distance measurements.
- Ensure that all required libraries and dependencies are properly installed and configured.

## Contributors
- [Lucas Troy](https://github.com/ldt9)
- [John Klamut](https://github.com/jmklamut)
