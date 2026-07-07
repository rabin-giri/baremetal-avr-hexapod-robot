# Bare-Metal AVR ATmega32 Hexapod Controller

> Developed in 2013, this project is a low level, bare-metal robotics controller for a 12 DoF hexapod walking robot with a 2 DoF pan/tilt face-tracking system. Built entirely from scratch for the AVR ATmega32 microcontroller, it utilizes optimized timeline sorting algorithms and 16 bit hardware timers to multiplex a custom software PWM engine across multiple GPIO ports.

## 🧠 The Engineering Challenge: Custom Software PWM

The ATmega32 microcontroller only has four hardware PWM channels, but this robot required 14 independent RC servos (12 for the legs, 2 for the camera head). Instead of relying on external I2C servo drivers, this project features a custom built, event driven software PWM engine that uses a single 16 bit timer (`TIMER1`) to drive all 14 servos simultaneously without jitter.

**How the PWM Multiplexer Works:**
1. **State Sorting (`arrange_servo`):** Every loop, the desired pulse widths (in microseconds) for all 14 servos are captured and sorted from shortest to longest using a bubble sort algorithm.
2. **Event Queue Generation:** The algorithm generates an array of unique timer comparison values and precalculates bitmasks for `PORTA`, `PORTB`, and `PORTC`.
3. **Interrupt Engine:** 
   * `TIMER1_COMPA` fires every 20ms to reset the 50Hz cycle, pulling all active servo pins `HIGH`.
   * `TIMER1_COMPB` fires iteratively throughout the cycle. At each interrupt, it pulls the specific pins `LOW` whose pulse widths have elapsed, then immediately loads the *next* shortest pulse width into the hardware timer comparator.

This timeline sorting approach provides highly efficient, non blocking PWM generation across arbitrary GPIO pins with minimal CPU overhead.

## ⚙️ Core Features

* **14 Channel Multiplexing:** Drives 14 independent SG90/MG90S servos directly from GPIO pins.
* **Tripod Gait Kinematics:** Implements a statically stable walking sequence (two interlocking triangles of support) allowing forward, backward, and rotational movement.
* **UART Telemetry & Control:** Non blocking state execution with serial command parsing for remote control at 9600 baud.
* **Face Tracking & Inverse Kinematics:** Parses 8 bit X/Y coordinate streams to drive a 2 DoF pan/tilt head. Includes a kinematic `tilt()` function that adjusts the hexapod's body stance to lean into directional movements.
* **No High Level Libraries:** Written entirely in baremetal C using direct AVR register manipulation.

## 🛠️ Hardware Requirements

* **Microcontroller:** AVR ATmega32 (running at 8MHz via internal/external oscillator)
* **Actuators:** 14x Micro RC Servos (e.g., TowerPro SG90)
* **Power:** Dedicated 5V/6V high-current buck converter for the servo rails (logic and motor power must be isolated to prevent brownouts).
* **Communication:** UART to USB/Bluetooth module (for PC/joystick commands and face-tracking data).

## 🚀 Compiling and Flashing

This project was built using the standard AVR-GCC toolchain. 

1. Ensure you have `avr-gcc` and `avrdude` installed.
2. Compile the source code specifying the `atmega32` target.
3. Flash via an AVR ISP programmer (like the USBasp):

```bash
avr-gcc -mmcu=atmega32 -Os -o hexapod.elf main.c
avr-objcopy -j .text -j .data -O ihex hexapod.elf hexapod.hex
avrdude -c usbasp -p m32 -U flash:w:hexapod.hex:i
```

*(Note: The JTAG interface is explicitly disabled in the code via `MCUCSR = (1 << JTD);` to free up `PORTC` pins for servo control.)*

## 📜 Serial Command Reference

The controller accepts single-byte ASCII characters over UART (9600 baud) for manual control:

| Command | Action | Command | Action |
| :---: | :--- | :---: | :--- |
| `F` | Walk Forward | `O` / `P` | Twist Body CW / CCW |
| `B` | Walk Backward | `U` | Stand Up (Raise Z  height) |
| `L` | Turn Left | `V` | Stand Down (Lower Z height) |
| `R` | Turn Right | `1` | Reset to Neutral Position |
| `M` / `N` | Lean Forward / Backward | `2` | Toggle PWM Engine (Motors ON/OFF) |
| `232` | Enter tracking mode (expects incoming 8-bit X/Y stream) | | |

## 📅 Historical Context

This codebase was originally developed as a college robotics project in 2013. It remains a fantastic demonstration of low-level embedded systems
programming, interrupt handling, and overcoming hardware limitations through creative software architecture.

<p align="center">
  <img  src="robot.png" alt="Motion planning car clip">
</p>
