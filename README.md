# 2023-Romi
# WPILib Romi Example Projects
This repo provides a list of additional WPILib Romi example projects beyond those supplied directly in WPILib. For more information on the core WPILib examples, plese see [WPILib Programming the Romi](https://docs.wpilib.org/en/latest/docs/romi-robot/programming-romi.html). The examples projects in this repo showcase ways to use various off-the-shelf sensors and actuators in conjunction with the base WPILib Romi kit. You can simply clone this repository, open these projects in VS Code, and build.

## List of Example Projects

### WPILib Features
1. [Robot Characterization with frc-characterization](romi-characterization/)
2. [Trajectory Follower](romi-trajectory-ramsete/)

### COTS Mechanisms
1. [Simple Robot Arm](simpleArm/)

<hr>

# 2022-Romi
**Romi Robot** [documentation from WPILib](https://docs.wpilib.org/en/stable/docs/romi-robot/index.html)

## Wireless Network Setup
1. Perform the following steps to get your Raspberry Pi recady to use with the Romi:
Tucpr3663!n the Romi on by sliding the power switch on the Romi 32U4 board to the on position. The first time it is started with a new image it will take approximately 2-3 minutes to boot while it resizes the file system and reboots. Subsequent times it will boot in less than a minute.

2. Using your computer, connect to the Romi WiFi network using the SSID `WPILibPi-<number>` (where `<number>` is based on the Raspberry Pi serial number) with the WPA2 passphrase `WPILib2021!`. 
> For CPR Team 3663: 
> * Romi SSID is: `WPILibPi3663A` `3663BlueRomi` etc
> * Passphrase is: `Cpr3663!`

3. Open a web browser and connect to the Raspberry Pi dashboard at either http://10.0.0.2/ or http://wpilibpi.local/

## The Romi has 2 microprocessor boards:
A Raspberry Pi that handles high level communication with the robot program running on the desktop and
A Romi 32U4 Control Board that handles low level motor and sensor operation.

## Hardware, Sensors and GPIO
The Romi has the following built-in hardware/peripherals:
- 2x geared motors with encoders
- 1x Inertial Measurement Unit (IMU)
- 3x LEDs (green, yellow, red)
- 3x pushbuttons (marked A, B, and C)
- 5x configurable GPIO channels

## Motors, Wheels and Encoders
The motors used on the Romi have a 120:1 gear reduction, and a no-load output speed of 150 RPM at 4.5V. The free current is 0.13 amps and the stall current is 1.25 amps. Stall torque is 25 oz-in (0.1765 N-m) but the built-in safety clutch might start slipping at lower torques.
The wheels have a diameter of 70mm (2.75”). They have a trackwidth of 141mm (5.55”).
The encoders are connected directly to the motor output shaft and have 12 Counts Per Revolution (CPR). With the provided gear ratio, this nets 1440 counts per wheel revolution.

## Inertial Measurement Unit
The Romi includes an STMicroelectronics LSM6DS33 Inertial Measurement Unit (IMU) which contains a 3-axis gyro and a 3-axis accelerometer.
The accelerometer has selectable sensitivity of 2G, 4G, 8G, and 16G. The gyro has selectable sensitivity of 125 Degrees Per Second (DPS), 250 DPS, 500 DPS, 1000 DPS and 2000 DPS.
The Romi Web UI also provides a means to calibrate the gyro and measure its zero-offsets before use with robot code.

## LEDs and Push Buttons
The Romi 32U4 control board has 3 push buttons and 3 LEDs that are exposed as Digital IO channels to robot code. The section on GPIO Mapping has more information on how to use the built in LEDs and buttons in robot code.

## Digital IO
| DIO Channel | Romi Hardware Component |
|-------------|-------------------------|
DIO 0         | Button A (input only)
DIO 1         | Button B (input), Green LED (output)
DIO 2         | Button C (input), Red LED (output)
DIO 3         | Yellow LED (output only)
DIO 4         | Left Encoder Quadrature Channel A
DIO 5         | Left Encoder Quadrature Channel B
DIO 6         | Right Encoder Quadrature Channel A
DIO 7         | Right Encoder Quadrature Channel B

## PWM Output
| PWM Channel | Romi Hardware Component |
|-------------|-------------------------|
PWM 0         | Left Motor
PWM 1         | Right Motor
