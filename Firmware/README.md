# Rotator Firmware

This firmware is based on [SatNOGS Rotator Controller](https://gitlab.com/librespacefoundation/satnogs/satnogs-rotator-firmware) and is in a very alpha state: use it with caution!
A lot of safety features are still not implemented (watchdog, overheating protection).

## Instructions

In order to use this code, you need to install:
 * [AccelStepper library](http://www.airspayce.com/mikem/arduino/AccelStepper/index.html)
 * [TMCStepper library](https://github.com/teemuatlut/TMCStepper)
 * Download Arduino IDE (tested with 1.8.5)

#### Burn optiboot

Only with SWD programming:

* Jlink like programmer
* Arduino Micro as DAP device

Connect the programmer to the SWD port on the board. Using Arduino IDE, select Arduino Zero (programming port), then burn bootloader. Wait while flashing the bootloader and disconnect the programmer.

#### Upload the code

Connect a USB cable to the controller board. Select Arduino Zero (native port) in Arduino IDE. Open the main firmware file (.ino) in Arduino IDE and press Upload. Wait while flashing the firmware. Disconnect USB cable.

## Testing with hamlib - rotctl or with Serial Monitor

Connect the PC with contreller via RS485 to USB by using the right converter.
Use commands of rotctl:

```
rotctl -m 204 -s 19200 -r /dev/ttyUSB1 -vvvvv
```

Replace the /dev/ttyUSB1 with the device which is connected to PC.

Use commands of easycomm 3:

Send directly commands of easycomm 3 as described in Easycomm implemantation section.

## Use with Gpredict
Configure Gpredict and launch hamlib deamon with: 
```
rotctld -m 202 -r COM7 -s 19200 -T 127.0.0.1 -t 4533 -C timeout=500 -C retry=0 > pause
```
Replace the COM7 with the device which is connected to PC.

## Easycomm implemantation

* AZ, Azimuth, number - 1 decimal place [deg]
* EL, Elevation, number - 1 decimal place [deg]
* SA, Stop azimuth moving
* SE, Stop elevation moving
* RESET, Move to home position
* PARK, Move to park position
* IP, Read an input, number
    * Temperature = 0
    * SW1 = 1
    * SW2 = 2
    * Encoder1 = 3
    * Encoder2 = 4
    * Load of M1/AZ = 5
    * Load of M2/EL = 6
    * Speed of M1/AZ (DPS) = 7
    * Speed of M2/EL (DPS) = 8
* VE, Request Version
* GS, Get status register, number
    * idle = 1
    * moving = 2
    * pointing = 4
    * error = 8
* GE, Get error register, number
    * no_error = 1
    * sensor_error = 2
    * homing_error = 4
    * motor_error = 8
    * over_temperature = 12
    * wdt_error = 16
* VL, Velocity Left ,number [mdeg/s]
* VR, Velocity Right, number [mdeg/s]
* VU, Velocity Up, number [mdeg/s]
* VD, Velocity Down, number [mdeg/s]
* CR, Read config, register [0-x]
    * Gain P for M1/AZ = 1
    * Gain I for M1/AZ = 2
    * Gain D for M1/AZ = 3
    * Gain P for M2/EL = 4
    * Gain I for M2/EL = 5
    * Gain D for M2/EL = 6
    * Azimuth park position = 7
    * Elevation park position = 8
    * Control mode (position = 0, speed = 1) = 9
* CW, Write config, register [0-x]
    * Gain P for M1/AZ = 1
    * Gain I for M1/AZ = 2
    * Gain D for M1/AZ = 3
    * Gain P for M2/EL = 4
    * Gain I for M2/EL = 5
    * Gain D for M2/EL = 6
    * Azimuth park position = 7
    * Elevation park position = 8
    * This reg is set from Vx commands control mode (position = 0, speed = 1) = 9
* RB, command to reboot controller

## Pins Configuration

```
M1IN1 10, Step or PWM1
M1IN2 9, Direction or PWM2
M1SF  7, Status flag
M1FB  A1, Load measurment

M2IN1 11, Step or PWM1
M2IN2 3, Direction or PWM2
M2SF  6, Status flag
M2FB  A0, Load measurment

MOTOR_EN 8, Enable/Disable motors

SW1 5, Endstop for axis 1
SW2 4, Endstop for axis 2

RS485_DIR 2, RS485 Half Duplex direction pin

SDA_PIN 3, Data I2C pin
SCL_PIN 4, Clock I2C pin

PIN12 12, Digital output pin
PIN13 13, Digital output pin
A2    A2, Analog input pin
A3    A3, Analog input pin
```

## TODO

* Implement overheating protection and overheating warning
* Implement watchdog
* Replace blocking delay with non-blocking delay in easycomm implementation.
* Remove PID related stuff (not used with steppers)