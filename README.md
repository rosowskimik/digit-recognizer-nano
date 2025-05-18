# Digit Recognizer for Arduino Nano 33 BLE with Zephyr

This is a simple TensorFlow digit recognition application targeting Arduino Nano BLE + Arducam Mini.
Board firmware is written in ZephyrRTOS.

## Project setup

To fetch all the necessary Zephyr modules run:

```sh
west init -l .
west update
```

## Building the app

To build the application run:

```sh
west build -b arduino_nano_33_ble app
```

## Flashing the board

In order to flash the board, you will need Arduino variant of `bossac`.
You can get it from [lastest release](https://github.com/arduino/BOSSA/releases) from Arduino source tree.

To flash the board run:

```sh
west flash --bossac="<path_to_arduino_bossac>"
```
