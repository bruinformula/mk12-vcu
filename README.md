# MK12 VCU Firmware

Firmware for the Bruin Formula MK12 vehicle control unit: an STM32H725RGV (VFQFPN68) driving a Cascadia Motion CM200DX inverter over CAN.

The VCU:

- reads the accelerator (APPS1, APPS2) and brake (BSE) sensors through ADC1 + DMA,
- runs a 1 kHz control loop that checks the pedals against the FSAE rules and turns pedal travel into a torque request,
- sends the torque request to the inverter over CAN at 100 Hz,
- handles the precharge request and the ready-to-drive (RTD) sequence, including the RTD sound over I2S,
- sends cooling commands and VCU diagnostics over CAN.

How the torque path works, and which rule each check satisfies: [docs/torque-control.md](docs/torque-control.md).
How to retune it and what to check on the bench: [docs/tuning-and-bench.md](docs/tuning-and-bench.md).
The full bench test plan, phase by phase with edge cases: [docs/bench-plan.md](docs/bench-plan.md).
What to demo at tech inspection, and the open items to close before it: [docs/tech-inspection.md](docs/tech-inspection.md).
Regen (this branch only, `REGEN_ENABLED 1`): design, car speed, and its bench checks: [docs/regen.md](docs/regen.md).

## Drive state flow

```
IDLE ──hold PRCHG 1 s──▶ precharge request (0x6B3) ──BMS replies OK within 6 s──▶ PRECHARGED
PRECHARGED ──hold RTD 1 s with brake pressed (valid BSE reading)──▶ RTD sound (~2.6 s) ──▶ up to 3 × 0 Nm frames, 150 ms lockout ──▶ DRIVE
any state ──shutdown power lost (0x6B6)──▶ resetVCU() ──▶ IDLE
```

- If the BMS never answers the precharge request, the VCU goes to `VCU_CAN_FAULT` and halts in `Error_Handler()` (sends 0 Nm, inverter disabled).
- Leaving drive (`resetVCU()`) is the only thing that clears a latched pedal sensor fault.

## Repository layout

| Path | What's there |
|---|---|
| `Core/Src/main.c` | Init, button/timer callbacks, the 1 kHz `controlLoopTick()` (TIM6) |
| `Core/Src/pedal_checks.c` | Pedal scaling and all plausibility checks. No HAL, host-tested |
| `Core/Src/torque_map.c` | Pedal map table, regen, filter, fault cut. No HAL, host-tested |
| `Core/Src/torque_diag.c` | Packing for the `0x502` torque diagnostics frame. No HAL, host-tested |
| `Core/Src/motor_control.c` | Glue between the pure files above and the hardware; sends `0x0C0` and `0x502` |
| `Core/Src/vcu_state.c` | `resetVCU()` and the VCU diagnostics frame |
| `Core/Src/prchg.c` | Precharge request/response and the 6 s timeout |
| `Core/Src/cooling.c` | Fan/pump commands from inverter and BMS temperatures |
| `Core/Src/audio.c` | RTD sound over I2S |
| `Core/Src/fdcan.c` | FDCAN setup and the RX message router |
| `mk12-vcu.ioc` | CubeMX config. Generated files (`tim.c`, `stm32h7xx_it.c`, ...) come from here |
| `tests/` | Host unit tests for `pedal_checks.c`, `torque_map.c` and `torque_diag.c` |
| `.github/workflows/tests.yml` | CI: runs the host tests with GCC and Clang on every push and PR |
| `tools/` | Torque map sweep + plot, Live Expressions guide generator |

## Building and flashing

1. In STM32CubeIDE: **File → Import → Existing Projects into Workspace**, and select the **repo root** (the project is `mk12-vcu`).
2. Build the `Debug` configuration and flash with an ST-Link as usual.

Notes:

- `STM32CubeIDE/` holds a second, older project with the same name. Don't import both into one workspace; use the root one.
- `mk11-vcu Debug.launch` is left over from MK11 and points at the wrong `.elf`. Make a new debug configuration for `mk12-vcu`.
- If you regenerate code from `mk12-vcu.ioc`, keep your changes inside `USER CODE BEGIN/END` blocks. The CubeMX-generated files use CRLF line endings; keep them that way so diffs stay small.

## Tests

The pedal checks, torque map and diagnostics packing compile on a laptop and are tested there:

```bash
make -C tests
```

Run this before every commit that touches `pedal_checks.*`, `torque_map.*` or `torque_diag.*`. GitHub Actions also runs it (with GCC and Clang) and builds the torque sim on every push and pull request; a red check means don't merge. Tests marked `CURRENT BEHAVIOR` pin known limitations on purpose. If you change that behavior, change the test in the same commit.

## Tools

```bash
clang -O2 -ICore/Inc tools/torque_curve_sim.c Core/Src/torque_map.c -lm -o torque_curve_sim
./torque_curve_sim > sweep.csv
python3 tools/plot_torque_curve.py sweep.csv
```

The sweep uses the real `torque_map.c`, so the plot is exactly what the car runs.

## CAN messages

FDCAN1, 500 kbit/s, standard IDs. FDCAN2 is configured but nothing uses it yet.

| ID | Dir | Rate | Content |
|---|---|---|---|
| `0x0C0` | TX | 100 Hz in drive | Inverter torque command (layout in [torque-control.md](docs/torque-control.md#inverter-torque-frame-0x0c0)) |
| `0x500` | TX | 20 Hz | VCU diagnostics: car speed, torque, APPS1/APPS2/BSE %, status flags |
| `0x502` | TX | 100 Hz in drive | Torque diagnostics: driver request, final command, APPS %, fault flags (layout in [torque-control.md](docs/torque-control.md#torque-diagnostics-frame-0x502)) |
| `0x501` | TX | 2 Hz | Cooling command: enable, tractive fan, pump, accessory fan PWM |
| `0x6B3` | TX | on request | Precharge request to BMS |
| `0x6B4` | RX | | BMS precharge response (byte 0 = 1 means success) |
| `0x6B5` | RX | | BMS ignored the precharge request |
| `0x6B6` | RX | | Shutdown power lost: reset to IDLE |
| `0x6B1` | RX | | BMS temperatures |
| `0x0A0` | RX | | Inverter temperatures |
| `0x0A5` | RX | | Inverter motor speed |
| `0x0A7` | RX | | Inverter DC bus voltage |

## Known issues

- **Car speed constants are not verified on the car** (`feature/regen`): gear ratio 59/12, 16 in tire, rpm sign. The old factor read ~48× high; it now divides by the gear ratio and uses the smallest plausible tire, so any error reads low (safe for EV.3.3.3). See [docs/regen.md](docs/regen.md).
- **BSE open/short thresholds (250 / 2780) are not yet verified on the car.** See the bench checklist.
- **Fan PWM timers (TIM1, TIM8) are configured but never started** (`HAL_TIM_PWM_Start` is never called).
- **No power limit in the VCU yet** (EV.3.3.1, 80 kW). Needed before full-power runs.
