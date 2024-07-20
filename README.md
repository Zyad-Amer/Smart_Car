# Smart Car System

## Overview
The Smart Car System is an advanced autonomous vehicle control system featuring multiple driving modes and intelligent sensors for precise navigation.

## Features
- **Manual Mode**: Control the car's movements (forward, backward, left, right, stop) via Bluetooth commands.
- **Automatic Mode**: Autonomous navigation using four ultrasonic sensors to detect obstacles and adjust the path.
- **Line Follower Mode**: Follow a predefined line path using three IR sensors.
- **Speed Control**: Variable speed settings (minimum, average, maximum) controlled via PWM signals.
- **Secure Communication**: UART communication for receiving commands and configuring the system.
- **Motor Control**: Dual DC motors for both left and right wheels, allowing precise movement control.
- **Interrupt-Driven Design**: Efficient handling of real-time control and sensor feedback.

## Setup and Configuration

### Hardware Requirements
- Microcontroller (e.g., ATmega328)
- Bluetooth Module (e.g., HC-05)
- Ultrasonic Sensors (x4)
- IR Sensors (x3)
- DC Motors (x4)
- Motor Driver (e.g., L298N)
- PWM Controllers (Timer0 and Timer2)
- Miscellaneous: Resistors, Capacitors, Wires, etc.

## Pin Configuration

### Ultrasonic Sensors
- **Front Sensor**: TRIGGER - `PORTx_PINy`, ECHO - `PORTx_PINy`
- **Back Sensor**: TRIGGER - `PORTx_PINy`, ECHO - `PORTx_PINy`
- **Right Sensor**: TRIGGER - `PORTx_PINy`, ECHO - `PORTx_PINy`
- **Left Sensor**: TRIGGER - `PORTx_PINy`, ECHO - `PORTx_PINy`

### IR Sensors
- **Center Sensor**: OUTPUT - `PORTx_PINy`
- **Right Sensor**: OUTPUT - `PORTx_PINy`
- **Left Sensor**: OUTPUT - `PORTx_PINy`

### DC Motors
- **Left Motors**: `PWM_PIN1`, `PWM_PIN2`
- **Right Motors**: `PWM_PIN3`, `PWM_PIN4`

### Bluetooth Module
- **RX**: `PORTx_PINy`
- **TX**: `PORTx_PINy`

## Code Overview

### Files
- `Smart_Car.c`: Main file containing the control logic for the smart car.
- `Motor.c`: Motor control functions.
- `Smart_Car.h`: Header file with function declarations and macros.
- `Makefile`: Makefile for compiling and uploading the code.

### Key Functions
- `Move_Forward()`: Operate all motors to move the car forward.
- `Move_Backward()`: Operate all motors to move the car backward.
- `Move_Left()`: Operate motors to turn the car left.
- `Move_Right()`: Operate motors to turn the car right.
- `Stop()`: Stop all motors.

### Modes
- **Manual Mode**: Control via Bluetooth commands.
- **Automatic Mode**: Use ultrasonic sensors for autonomous navigation.
- **Line Follower Mode**: Follow a line using IR sensors.
