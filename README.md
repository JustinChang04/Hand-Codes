# Hand Codes for Krysallis Hand
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## About
The Krysallis Hand system is split into MCP, PIP, DIP, wrist, and abduction modules, all programmable via the Arduino IDE. These modules are all controlled over I2C by a Raspberry Pi Pico with code from the RPI_Master repository. Each folder is an Arduino project for each module, titled "PID-[module]". PID_Master is unused code that an Arduino can use to control the modules over I2C.

## Programming the Arduinos
To program a module, open the Arduino IDE and connect the Arduino Mega. Select the connected Arduino Mega under the device tree, and program the device.

## Pinout
The Arduino Megas all follow a [standardized pinout](https://docs.google.com/spreadsheets/d/1TUzqdXqWe3b5YbMJ5jpp_UFLg8gyYuG0tuyAA3czh9w/edit?usp=sharing) for their four motors, except the thumb module, which doesn't use the last motor's pinout.
