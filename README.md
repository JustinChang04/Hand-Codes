# Hand Codes for Krysallis Hand
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## Setup
Requires installation of the [Arduino IDE](https://www.arduino.cc/en/software/). Programming and communicating with the Raspberry Pi Pico requires a machine running Ubuntu 22.04 and installation of [ROS 2 Humble](https://docs.ros.org/en/humble/index.html#). Refer to the [RPI_Master](https://github.com/JustinChang04/RPI_Master) folder for more instructions about progamming and communicating with the Raspberry Pi Pico.

# Usage

## About
The Krysallis Hand system is split into MCP, PIP, DIP, wrist, and abduction modules, all programmable via the Arduino IDE. These modules are all controlled over I2C by a Raspberry Pi Pico with code from the RPI_Master repository. Each folder is an Arduino project for each module, titled "PID-[module]". PID_Master is unused code that an Arduino can use to control the modules over I2C.

## How the Code Works
The DC motors have two offset hall effects sensors and an magnetic encoder that spins with the motor. The Arduinos trigger an interrupt on the rising edge of one of the hall effect sensors, and in the interrupt service routine, the Arduino checks the value of the other hall effect sensor and increments or decrements the motor's position depending on the value read. This allows the position of the motor to be updated in real-time.

Each Arduino module drives two AT8236 2-channel motor controller, each of which controlls 2 DC motors running at 6V. The thumb module drives two AT8236 motor controller, one running at 6V to control the MCP and IP joints and one running at 12V to control the CMC joint. The speed and direction of the motors is specified by a value between -255 to 255. Each channel of the motor controller takes two PWM values and uses the difference to determine the speed and direction of the motors.

Each motor has a target position specified in number of pulses. This goal is set over I2C. The position of each motor is controlled by a position-error controller, although a PID controller could easily be implemented. The position controller calculates the position error of the motor, multiplies it by a constant, and sets that at the speed of the motor, capped between -255 and 255.

## Programming the Arduinos
To program a module, open the corresponding folder in the Arduino IDE and connect the Arduino Mega. Select the connected Arduino Mega under the device tree, and program the device.

## Pinout
The Arduino Megas all follow a [standardized pinout](https://docs.google.com/spreadsheets/d/1TUzqdXqWe3b5YbMJ5jpp_UFLg8gyYuG0tuyAA3czh9w/edit?usp=sharing) for their four motors, except the thumb module, which doesn't use the last motor's pinout.
