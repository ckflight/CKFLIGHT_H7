# CKFLIGHT H7 Advanced Flight Controller Firmware (Light Version) as a STM32CubeIDE Project

<img width="2027" height="1139" alt="Image" src="https://github.com/user-attachments/assets/83101031-c83c-4fed-b8a0-d6efd2ead19e" />

I have decided to upload the lighter version of my flight controller firmware: High-performance bare-metal 16K loop time capable STM32H7 flight controller firmware close to betaflight in flight performance developed for educational, hobbyist, and academic use by me. 

STM32CubeIDE is a great environment for easy debugging without getting into unnecessary details. 

Commercially available boards can be used with this software: Check Boad Porting and Configuration Guide below.

CKFLIGHT H7 is designed as a simplified and readable version of the full CKFLIGHT firmware architecture while preserving modern flight-controller concepts including high-rate control loops, sensor fusion, digital ESC protocols, SD logging, USB communication, advanced PID algorithms, and real-time embedded control systems.

The firmware targets STM32H743-based flight controllers and is implemented using STM32CubeIDE without an RTOS.

The project is intended to provide both:

- A real usable flight-controller firmware
- A readable educational embedded systems reference

The architecture focuses on exposing low-level implementation details rather than hiding functionality behind large abstraction layers.

---

# Features

- Bare-metal STM32H743 firmware architecture
- High-rate real-time flight control loops
- Advanced PID controller implementation
- IMU sensor fusion and attitude estimation
- DSHOT ESC protocol support
- RC receiver protocols
  - SBUS
  - CRSF
- USB communication interface
- MSP communication support
- SD card flight data logging
- GPS support
- Barometer support
- Magnetometer support
- OSD support
- SmartAudio VTX control
- DMA-based peripheral handling
- SPI / UART / I2C driver stack
- Blackbox-style logging infrastructure
- Runtime telemetry and configuration systems
- Modular firmware structure for learning and development
- STM32CubeIDE project structure
- Real-time fault detection and debugging infrastructure

---

# Flight Control Firmware Details

The firmware contains a complete modern flight-control stack including:

- Real-time PID control system
- Attitude estimation and sensor fusion
- Gyroscope and accelerometer filtering
- RC input processing
- Motor mixer implementation
- DSHOT motor output generation
- Failsafe handling
- Arming and safety logic
- Navigation and altitude control infrastructure
- GPS integration
- Blackbox-style SD card logging
- USB telemetry and configuration interface
- Runtime communication systems
- Fault detection and debugging infrastructure

The control architecture is designed for:

- Low latency
- High loop-rate operation
- Deterministic timing
- Minimal software overhead
- Direct hardware-level control

The project focuses on readable implementation of real flight-controller concepts rather than excessive abstraction layers.

The firmware performance and architecture are designed to be conceptually comparable to modern high-performance open-source flight-controller firmware while remaining easier to study and modify.

---

# Real Flight Video

https://github.com/user-attachments/assets/4c7df17b-40bc-41f8-9e52-42024572642d

---

# Terminal Interface

https://github.com/user-attachments/assets/535e39e9-0e48-4be0-abaf-20bba0b0e894

<img width="1526" height="844" alt="Image" src="https://github.com/user-attachments/assets/935bc549-ba67-491f-a17a-4a270db213b6" />

---

# Supported Hardware

Primary target:

- STM32H743 flight controller platforms

Supported sensors currently present in firmware:

## IMUs
- ICM20602_GYRO,
- ICM45686_GYRO,
- ICM42688P_GYRO,
- L3GD20H_GYRO,
- ICM42605_GYRO,
- IIM42652_GYRO,
- MPU6000_GYRO,
- ICM20602_ACC,
- ICM45686_ACC,
- ICM42688P_ACC,
- IIM42652_ACC,
- LSM303D_ACC,
- FXOS8700CQ_ACC,
- ICM42605_ACC,
- MPU6000_ACC,
- BNO055_IMU

## Magnetometers
- MAG3110_MAGNETO,
- LSM303D_MAGNETO,
- FXOS8700CQ_MAGNETO,
- HMC5983_MAGNETO,
- QMC5883L_MAGNETO,
- MLX90393_MAGNETO,

## Barometers
- MS5607_BAROMETER,
- BMP280_BAROMETER,
- MS5611_BAROMETER,
- DPS310_BAROMETER,

## GPS
- GPS_UBLOX7,
- GPS_UBLOX8,
- M10 Series

## SPI Flash and SPI/SDIO SD Card

Use this log analysis tool i have written for flight log analiysis: https://github.com/ckflight/Flight_Log_Tools

<img width="1440" height="900" alt="Image" src="https://github.com/user-attachments/assets/c2dae7aa-cf1c-408a-8b66-1965b458bf14" />
<img width="1440" height="900" alt="Image" src="https://github.com/user-attachments/assets/2bfab410-94e1-4786-b5f5-ae395c77af66" />
<img width="1440" height="900" alt="Image" src="https://github.com/user-attachments/assets/65a67220-2bfc-44e3-8488-375f693efad6" />

---

# Board Porting and Configuration Guide

CKFLIGHT H7 is designed so that custom STM32H7 flight-controller boards and commercially available Betaflight-style flight controllers can be adapted through configuration headers.

The main configuration files are:

```text
Core/Inc/CK_DEFINITIONS.h
Core/Inc/CK_SETTINGS.h
Core/Inc/config/
```

---

## Board Selection

<img width="2494" height="1404" alt="Image" src="https://github.com/user-attachments/assets/e4420c43-7bd6-4e89-9a34-cfe242fd0568" />
<img width="2494" height="1404" alt="Image" src="https://github.com/user-attachments/assets/2db4f87e-cab1-49af-92f2-14391fc41286" />

`CK_DEFINITIONS.h` selects the target board and MCU family.

Example:

```c
#define CKFLIGHT_F4 false
#define CKFLIGHT_H7 false
#define KAKUTE_H7_1v3 true
#define MATEKH743_SLIMV3 false
#define RF_REVOLT false
```

Only one target board should be enabled at a time.

Currently included board targets include:

```text
CKFLIGHT_H7
KAKUTE_H7_v1.3
MATEKH743_SLIMV3
CKFLIGHT_F4
RF_REVOLT
```

Each board target includes its own pin/peripheral configuration from:

```text
Core/Inc/config/
```

To port the firmware to another Betaflight-compatible STM32H7 flight controller:

1. Add a new board definition in `CK_DEFINITIONS.h`
2. Create a matching board config header in `Core/Inc/config/`
3. Map the correct SPI, UART, I2C, timer, motor, receiver, OSD, LED, buzzer, ADC, and chip-select pins
4. Select the required sensors and peripherals in `CK_SETTINGS.h`
5. Verify interrupt priorities, DMA usage, and timer assignments
6. Build and flash using STM32CubeIDE

---

## Feature and Peripheral Selection

`CK_SETTINGS.h` enables or disables firmware modules depending on the hardware.

Examples:

```c
#define GYRO1_SPI_ 1
#define ACC1_SPI_ 1
#define GYRO2_SPI_ 0
#define ACC2_SPI_ 0

#define USE_BARO_ 0
#define USE_MAG_ 0
#define GPS_ 1

#define LOG_SPI_ 1
#define LOG_SDIO_ 0
#define LOG_FLASH_ 0

#define SBUS_ 0
#define CRSF_ 1

#define OSD_DJI_ 1
#define RGB_ 1
#define LED1_ 1
```

This allows the same firmware architecture to be adapted to different flight-controller layouts.

---

## Flight Performance Options

Advanced flight-performance features are enabled from `CK_SETTINGS.h`.

Currently supported options include:

```c
#define USE_DSHOT
#define USE_RC_SMOOTHING_FILTER
#define USE_AIRMODE_LPF
#define USE_ITERM_RELAX
#define USE_ABSOLUTE_CONTROL
#define USE_DYN_LPF
#define USE_INTEGRATED_YAW_CONTROL
#define USE_AIRMODE
#define USE_GYRO_OVERFLOW_CHECK
#define USE_THROTTLE_BOOST
#define USE_FEEDFORWARD
#define USE_TPA_MODE
#define USE_D_MAX
#define USE_ACC
#define USE_THRUST_LINEARIZATION
#define USE_SIMPLIFIED_TUNING
```

These features provide a modern control architecture similar in concept to high-performance open-source flight-controller firmware.

---

## Sensor Orientation and Mixer Setup

Sensor orientation and motor mixer configuration are also controlled from `CK_SETTINGS.h`.

Example:

```c
#define GYRO_ORIENTATION_X_SIGN 1
#define GYRO_ORIENTATION_Y_SIGN 1
#define GYRO_ORIENTATION_Z_SIGN -1
#define GYRO_ORIENTATION_SWAP_XY false

#define ACC_ORIENTATION_X_SIGN 1
#define ACC_ORIENTATION_Y_SIGN 1
#define ACC_ORIENTATION_Z_SIGN 1

#define MIXER_ORIENTATION 3
#define MIXER_ESC_REVERSED true
```

These settings must be checked carefully when using a different frame layout, IMU orientation, or ESC mounting direction.

---

## Important Porting Notes

Before flying on a new board:

- Verify all motor outputs with props removed
- Confirm gyro and accelerometer orientation
- Confirm receiver channel mapping
- Confirm arming, failsafe, and motor stop behavior
- Confirm battery voltage/current ADC scaling
- Confirm DSHOT/timer/DMA assignments
- Confirm SD card or flash logging configuration
- Confirm OSD, GPS, buzzer, LED, and telemetry peripherals
- Test sensor data over USB before enabling flight

This firmware is intended as a technical and educational reference. Hardware-specific validation is required before flight use.

---

# Firmware Architecture

```text
Core/
├── Inc/
│   ├── COMMON/
│   ├── COMMUNICATION/
│   ├── DRIVERS/
│   ├── FLASH/
│   ├── FLIGHT/
│   ├── MOTION/
│   ├── OSD/
│   ├── SENSORS/
│   ├── config/
│   ├── CK_DEFINITIONS.h
│   ├── CK_SETTINGS.h
│   ├── NOTES.txt
│   ├── firmware_notes.txt
│   ├── git_commit_hash.h
│   ├── git_hash.py
│   ├── main.h
│   ├── stm32h7xx_hal_conf.h
│   └── stm32h7xx_it.h
│
├── Src/
│   ├── COMMUNICATION/
│   ├── DRIVERS/
│   ├── FLASH/
│   ├── FLIGHT/
│   ├── MOTION/
│   ├── OSD/
│   ├── SENSORS/
│   ├── FATFS/
│   ├── USB_DEVICE/
│   ├── main.c
│   ├── stm32h7xx_hal_msp.c
│   ├── stm32h7xx_it.c
│   └── system_stm32h7xx.c
│
└── Startup/
    └── startup_stm32h743xx.s
```

---

## Main Firmware Modules

```text
FLIGHT/         -> PID, mixer, RC processing, navigation, failsafe
SENSORS/        -> IMU, barometer, magnetometer drivers
DRIVERS/        -> SPI, UART, I2C, ADC, DMA, GPIO
COMMUNICATION/  -> MSP, USB CDC, telemetry, configuration
OSD/            -> On-screen display
FLASH/          -> Flash memory handling
MOTION/         -> Motion estimation and filtering
FATFS/          -> SD card filesystem and logging
```

The project prioritizes:

- Readability
- Real-time determinism
- Low-level hardware understanding
- Educational value
- Minimal abstraction overhead

---

# Development Environment

- STM32CubeIDE
- STM32 HAL drivers
- STM32H7 series MCU
- Bare-metal architecture (no RTOS)

---

# Design Goals

This project was developed to provide:

- A readable modern flight-controller firmware
- A learning-oriented STM32H7 embedded architecture
- A practical reference for:
  - Sensor fusion
  - PID control
  - Real-time embedded systems
  - DMA-driven communication
  - Digital motor protocols
  - Flight logging systems

Unlike highly abstracted production firmware stacks, this repository focuses on exposing low-level implementation details for educational and research purposes.

---

# Current Status

The firmware includes:

- Real flight-control infrastructure
- Sensor drivers
- Communication stacks
- Logging systems
- USB interfaces
- Real-time control architecture

Development is ongoing.

---

# Repository Purpose

This repository is published as:

- Educational reference
- Embedded systems learning resource
- Flight-controller architecture example
- STM32H7 real-time firmware reference
- Personal engineering portfolio project

---

# License

This project is licensed under the GNU General Public License v3.0 (GPLv3).

You are free to use, modify, and distribute this software under the terms of the GPLv3 license. Any distributed modifications or derivative works must also be released under the same license and include source code.

For full license text:
https://www.gnu.org/licenses/gpl-3.0.html

---

# Author

Cenk Keskin

GitHub:
https://github.com/ckflight/CKFLIGHT_H7