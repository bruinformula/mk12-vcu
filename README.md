# Mk.11 VCU Firmware

Firmware for the Mk.11 vehicle control unit, running on an STM32 H723ZGT microcontroller.

## Overview

The VCU manages the main drive-state flow for the vehicle:

- Reads pedal sensors through ADC + DMA.
- Calculates and transmits torque requests to a Cascadia Motion CM200DX Inverter.
- Handles precharge sequencing and response tracking.
- Plays the ready-to-drive sound over I2S.
- Processes BMS and Inverter temperatures to send cooling commands over CAN to control fans.

## Key Modules

- `Core/Src/main.c` initializes hardware, configures FDCAN filters, starts ADC DMA, and runs the main firmware loop.
- `Core/Src/motor_control.c` handles pedal processing, plausibility checks, torque calculation, and inverter CAN messages.
- `Core/Src/prchg.c` manages the precharge request and response flow with timeout handling.
- `Core/Src/cooling.c` sends cooling control frames.
- `Core/Src/audio.c` handles ready-to-drive sound playback over I2S.
- `Core/Src/fdcan.c` configures FDCAN peripherals and routes received CAN frames.
- `Core/Src/vcu_state.c` manages the VCU state accordingly in case of loss of shutdown power.