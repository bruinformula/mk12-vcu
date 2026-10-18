# Bench test plan

A step-by-step plan for the first bench sessions with the MK12 VCU, from a VCU alone on a desk to a rehearsal of tech inspection. Every test has an ID, what to do, and what should happen, so results can be logged and compared between sessions. Rule numbers refer to the FSAE Rules 2027 V1.0.

The plan is ordered so that most of the logic is proven **before any high voltage**: a CAN tool can stand in for the BMS and the inverter, so phases 0 and 1 need no Tractive System.

Tests marked **⚠ expected to fail today** reproduce problems found in the code review of 2026-10-18 (summary under [Known problems to confirm](#known-problems-to-confirm)). Run them anyway: confirming the failure on real hardware is what justifies the fix.

Related docs: [torque-control.md](torque-control.md) (how the code works, frame layouts), [tuning-and-bench.md](tuning-and-bench.md) (calibration), [tech-inspection.md](tech-inspection.md) (what inspectors will ask).

## Contents

- [Safety rules](#safety-rules)
- [Equipment](#equipment)
- [Before each session](#before-each-session)
- [Phase 0: VCU on the desk](#phase-0-vcu-on-the-desk-lv-only)
- [Phase 1: in the car, LV only](#phase-1-in-the-car-lv-only)
- [Phase 2: Tractive System active, wheels off the ground](#phase-2-tractive-system-active-wheels-off-the-ground)
- [Phase 3: tech inspection rehearsal](#phase-3-tech-inspection-rehearsal)
- [Edge cases](#edge-cases)
- [Known problems to confirm](#known-problems-to-confirm)
- [Decisions waiting on bench results](#decisions-waiting-on-bench-results)
- [Results log](#results-log)

## Safety rules

These apply to every phase. Phase 2 also follows the team's own high-voltage procedures; this plan doesn't replace them.

1. **Never pause the debugger while the motor can spin** until test 1.6 has confirmed the inverter's CAN timeout. A breakpoint or "Suspend" stops the VCU sending `0x0C0`, and with the default EEPROM settings the inverter keeps applying the last torque it received.
2. **First Tractive System runs use reduced torque.** Build with `MAX_TORQUE` set to 10 in `torque_map.h` for phase 2, and raise it in steps (2.10). Put it back to 80 before committing.
3. **Wheels off the ground** for all of phase 2, with the drivetrain guarded and one person whose only job is the shutdown button.
4. **Regen must be off.** `0x502` byte 5 bit 7 must never be set. If it ever is, stop: the firmware isn't the expected build.
5. **Flash a clean build of a known commit.** Write the commit hash in the results log. Phase 0 and 1 can use `fix/fan-pwm` (it contains all the torque-mapping work plus the fan fix).

## Equipment

| Item | Used for |
|---|---|
| ST-LINK + STM32CubeIDE | Flashing, Live Expressions (`vcu_state`, `ADC_VAL`, `pedal_percents`, `plausibility_checks`, `requestedTorque`, `torque_filter`) |
| CAN adapter that can **log and send** (PCAN, CANable, Kvaser…) | Logging, and faking the BMS and inverter in phases 0–1. FDCAN1 runs at **500 kbit/s** |
| Frame decoding for `0x0C0`, `0x502`, `0x500`, `0x501` | Layouts in [torque-control.md](torque-control.md#torque-diagnostics-frame-0x502) and the README |
| Pedal box (or three potentiometers wired like the sensors) | Pedal inputs |
| Jumper leads with a 1 kΩ resistor, and access to each sensor connector | Unplugging sensors, shorting a signal to 3.3 V or to GND |
| A way to offset one APPS: breakout box, or a potentiometer in place of one sensor | APPS disagreement tests |
| Oscilloscope | Fan PWM, button bounce |
| Bench supply with current limit | Phase 0 power |
| Phone sound meter app | RTD sound level (≥ 80 dBA at 2 m) |
| Stopwatch, or timestamps in the CAN log | Timing checks |

## Before each session

- [ ] Build the commit you mean to test, note the hash, and flash it.
- [ ] Run `make -C tests` on that commit: all builds pass.
- [ ] CAN logger recording to a new file named after the date and phase.
- [ ] Results log ready (template at the end).
- [ ] Phase 2 only: `MAX_TORQUE` at the reduced value, the team's HV checklist done, shutdown-button person in place.

## Phase 0: VCU on the desk (LV only)

The VCU, its pedal inputs and buttons, and the CAN adapter. The CAN adapter fakes the BMS: precharge OK is a `0x6B4` frame with byte 0 = 1, and shutdown power lost is a `0x6B6` frame. No inverter is needed: the `0x0C0` frames on the log are the output under test.

### 0.1 Boot and idle traffic

| ID | Do | Expect |
|---|---|---|
| 0.1.1 | Power up with pedals released | No halt in `Error_Handler()`. `vcu_state` = 0 (IDLE). `0x500` at 20 Hz, `0x501` at 2 Hz. **No `0x0C0`** in IDLE |
| 0.1.2 | Power up with the throttle pressed | Same as above. Nothing happens until RTD |
| 0.1.3 | Power up with the CAN adapter unplugged (no other node to ACK) | VCU keeps running (Live Expressions update). Plug the adapter in: `0x500`/`0x501` appear within a second |
| 0.1.4 | Watch `ADC_VAL[0..2]` for 10 s with pedals at rest, then held at full | Note the noise in counts. It should be small next to the margins: APPS 10% ≈ 108 counts (APPS1) / 150 (APPS2), BSE "pressed" at 1000 |

### 0.2 Precharge request

Precharge always waits the full 6 s (TIM4) before deciding, even if the BMS answers sooner.

| ID | Do | Expect |
|---|---|---|
| 0.2.1 | Hold PRCHG for less than 1 s | Nothing sent |
| 0.2.2 | Hold PRCHG for 1 s | One `0x6B3` (byte 0 = 1) |
| 0.2.3 | After 0.2.2, send `0x6B4` byte 0 = 1 within 6 s | 6 s after the request, `vcu_state` = 1 (PRECHARGED). `0x500` byte 7 bit 7 ("looking for RTD") set |
| 0.2.4 | Repeat, but reply `0x6B4` byte 0 = 0 | Stays IDLE (precharge failure). Holding PRCHG again sends a new request |
| 0.2.5 | Repeat, but send `0x6B5` (BMS ignore) | Precharge request dropped, the 6 s timer stops, stays IDLE. PRCHG works again |
| 0.2.6 | Repeat, but **send no reply** | After 6 s: `vcu_state` = 3 (CAN_FAULT), `Error_Handler()`: 1–3 lockout frames (`0x0C0` 0 Nm, enable 0), then silence. **The VCU stays halted until power-cycled.** Decide if that's wanted (see decisions) |
| 0.2.7 | Send `0x6B4` while not waiting (IDLE, nothing requested) | Ignored |
| 0.2.8 | Hold PRCHG while already PRECHARGED | Ignored, no `0x6B3` |
| 0.2.9 | Tap PRCHG quickly several times, then hold 1 s | One request only |
| 0.2.10 | Reply `0x6B4` OK at 0.5 s, then at 5.9 s, then at 6.1 s (three runs) | 0.5 s and 5.9 s: PRECHARGED at 6 s. **6.1 s: the VCU has already halted** (as 0.2.6) |
| 0.2.11 | After `0x6B4` OK, hold PRCHG a second time before the 6 s are up | ⚠ Expected to fail today: a new `0x6B3` goes out and the earlier OK is forgotten; without a second reply the VCU halts at 6 s |

### 0.3 Ready to Drive

Start each test from PRECHARGED (repeat 0.2.3).

| ID | Do | Expect |
|---|---|---|
| 0.3.1 | Hold RTD 1 s, no brake | Nothing. No sound, stays PRECHARGED |
| 0.3.2 | Press the brake, then hold RTD 1 s | RTD sound (~2.6 s), then **1–3** `0x0C0` lockout frames (0 Nm, enable 0), then **~150 ms with no `0x0C0`**, then `0x0C0` every 10 ms (enable 1), each followed by `0x502` |
| 0.3.3 | Release RTD at about 0.9 s | Nothing |
| 0.3.4 | Hold RTD 1 s, press the brake only in the last ~0.2 s | **Today this passes** (brake checked once at the end). Record it for the drivers' decision |
| 0.3.5 | Unplug the BSE, press the "brake", hold RTD | Nothing (a failed BSE never counts as pressed) |
| 0.3.6 | Short the BSE signal to 3.3 V through 1 kΩ, hold RTD | Nothing |
| 0.3.7 | Hold RTD in IDLE (not precharged), with brake | Nothing |
| 0.3.8 | Brake + RTD with the throttle held at ~50% | Enters drive, but `0x0C0` torque stays 0 and `0x502` bits 3, 4, 6 stay set until the throttle drops below 5% |
| 0.3.9 | Press RTD again while the sound is playing | Nothing extra (buttons are disabled during the sound) |
| 0.3.10 | Measure the sound | 1–3 s long (expect ~2.6 s). Level is checked in phase 2 |
| 0.3.11 | Release the brake during the sound | Note the result: today the VCU still enters DRIVE (the brake is only checked before the sound) |
| 0.3.12 | Tap RTD repeatedly, releasing right around 1 s | Never enters RTD on a press shorter than 1 s |

### 0.4 Torque output

In DRIVE, watching `0x0C0` bytes 0–1 (Nm × 10) and `0x502`.

| ID | Do | Expect |
|---|---|---|
| 0.4.1 | Pedal at rest (≤ 5%) | 0 Nm |
| 0.4.2 | Hold the pedal at ~50% | Settles at 378 (37.8 Nm): the map is a straight line from 0 Nm at 5% to 80 Nm at 100%. The filter settles a hair under its target and the frame truncates, so values read one count low |
| 0.4.3 | Full pedal | Settles at **799** (79.9 Nm), not 800, for the same reason. Harmless, never above the request |
| 0.4.4 | Snap from 0 to full | Rises smoothly: about 63% (50 Nm) after 50 ms, ~95% after 150 ms. Never above the driver request (`0x502` bytes 0–1) |
| 0.4.5 | Snap from full to 0 | **0 Nm in the next frame** (EV.3.1.3) |
| 0.4.6 | Hold at 5%, wiggle around it | Torque stays at or very near 0, no jumps |
| 0.4.7 | Check the log over 60 s of mixed pedal | `0x0C0` every 10 ms (±1 ms). `0x502` counter goes up by 1 every frame, no gaps. `0x502` bytes 2–3 always equal `0x0C0` bytes 0–1 |
| 0.4.8 | Check `0x0C0` byte 4 (direction) and 5 (enable) | Byte 4 = 0 on every frame, byte 5 = 1 in drive |
| 0.4.9 | Lift off with the fake speed above 5 km/h (send `0x0A5` with a large rpm in bytes 2–3) | Torque never goes negative, `0x502` bit 7 never set (regen off) |

### 0.5 Pedal faults

In DRIVE with the throttle at ~30% unless stated. "Cut" means `0x0C0` torque 0 in the next frame and `0x502` bit 6 set. Bits: 0 APPS1, 1 APPS2, 2 BSE out of range, 3 disagreement, 4 brake + throttle, 5 latched.

| ID | Do | Expect | Rule |
|---|---|---|---|
| 0.5.1 | Unplug APPS1 and replug within ~50 ms (a quick flick) | Cut while unplugged, **torque comes back** after replug (no latch, bit 5 clear) | T.4.2.9 |
| 0.5.2 | Unplug APPS1 for 1 s, replug | Cut; bit 0, then bit 5 after 100 ms. **Stays cut** after replug | T.4.2.9, T.4.2.10 |
| 0.5.3 | Send `0x6B6` to reset, precharge, RTD again | Latch cleared, torque available | — |
| 0.5.4 | Repeat 0.5.1–0.5.2 with APPS2 | Same, bit 1 | T.4.2.9 |
| 0.5.5 | Short APPS1 signal to 3.3 V through 1 kΩ | Cut, bit 0, latches after 100 ms | T.4.2.10 |
| 0.5.6 | Short APPS1 signal to GND | Cut, bit 0 (reads like an open circuit) | T.4.2.10 |
| 0.5.7 | Repeat 0.5.5–0.5.6 with APPS2 and with the BSE | Same, bits 1 / 2 | T.4.2.10, T.4.3.4 |
| 0.5.8 | Offset one APPS by ~8% of travel | No cut | T.4.2.4 |
| 0.5.9 | Offset one APPS by ~15% | Torque continues for 100 ms, then cut (bit 3). Removing the offset brings torque back at once | T.4.2.4, T.4.2.5 |
| 0.5.10 | Offset by ~15% in pulses shorter than 100 ms | No cut (the timer restarts) | T.4.2.5 |
| 0.5.11 | Throttle above 25%, press the brake | Cut (bit 4) | EV.4.7 |
| 0.5.12 | Release the brake, stay on the throttle | Stays cut | EV.4.7.2 |
| 0.5.13 | Lift the throttle below 5% | Bit 4 clears; pressing again ramps up | EV.4.7.2 |
| 0.5.14 | Throttle at ~20% with the brake pressed | No cut (below 25%) | EV.4.7 |
| 0.5.15 | Jumper the APPS1 and APPS2 signal wires together | Cut within 100 ms (out of range or > 10% apart). The margin is small (10.7% at the worst point), so try several pedal positions | T.4.2.3 |
| 0.5.16 | Press the brake as hard as possible | No BSE fault (bit 2 clear) | T.4.3.4 |
| 0.5.17 | Offset one APPS to sit right at 10% apart, so noise crosses the line | ⚠ Watch bits 3 and 6: today one agreeing sample restarts the 100 ms timer, so a borderline fault can keep torque on for long stretches | T.4.2.5 |
| 0.5.18 | Flick a connector on and off repeatedly in bursts shorter than 100 ms | Torque cut on every bad burst. ⚠ Today the latch (bit 5) never sets for this pattern, although the docs say it should | T.4.2.9 |
| 0.5.19 | Throttle exactly at 25% / release to exactly 5% during a crosscheck | Note the boundaries. EV.4.7.2 says the cut stays until APPS is **less than** 5%; the code restores at 5% | EV.4.7 |
| 0.5.20 | Unplug a sensor during the 150 ms lockout right after RTD | Torque stays 0 when frames start, fault bits set | — |

### 0.6 Reset (shutdown power lost)

| ID | Do | Expect |
|---|---|---|
| 0.6.1 | In DRIVE with torque applied, send `0x6B6` | 1–3 lockout frames, then no more `0x0C0`. `vcu_state` = 0. Precharge and RTD work again |
| 0.6.2 | Send `0x6B6` about 1 s into the RTD sound, once with other traffic on the bus (e.g. `0x0A5` at 100 Hz) and once with a silent bus | Must end in **IDLE with lockout frames last**, never DRIVE. ⚠ Expected to fail with traffic: CAN RX can't run during the sound and the 3-frame RX FIFO drops the `0x6B6` |
| 0.6.3 | Send `0x6B6` while waiting for precharge | IDLE, the 6 s timer doesn't fire later |
| 0.6.4 | Send `0x6B6` in IDLE, and several times in a row | Harmless, stays IDLE |
| 0.6.5 | After one RTD (sound played), with **no other traffic**, send a single `0x6B6` in DRIVE | Reset at once. ⚠ Expected to fail today: the RX FIFO keeps 2 stale frames and only one frame is read per new arrival, so the `0x6B6` waits until two more frames arrive (forever on a silent bus) |
| 0.6.6 | Send `0x6B6` at random moments in DRIVE, 20 times | Lockout frames always last on the log |

### 0.7 CAN robustness

| ID | Do | Expect |
|---|---|---|
| 0.7.1 | In DRIVE, flood the bus from the adapter (e.g. `0x0A5` at 1 kHz plus other IDs) | `0x0C0` still every 10 ms. Only `0x502`, `0x500` or `0x501` may skip frames, never `0x0C0` |
| 0.7.2 | In DRIVE, unplug the VCU from the bus for 1 s, then replug | VCU keeps running. Frames resume after replug. Note what the log shows (`0x502` counter jump) |
| 0.7.3 | Briefly short CANH to CANL (bus-off) | ⚠ Expected: the VCU never sends or receives again (no bus-off recovery in the code). The inverter's CAN timeout is then the only thing stopping the motor, so 1.6 matters even more |
| 0.7.4 | In DRIVE with torque applied, unplug the VCU from the bus for 2 s, then replug | Look at the first frames after replug: up to 3 stale `0x0C0` (enable 1, old torque) can go out first |
| 0.7.5 | Send a `0x6B6` with an **extended** ID (`0x000106B6`), and a `0x6B4` with 0 data bytes right after a frame whose byte 0 was 1 | Both should be ignored. ⚠ Today the extended frame resets the VCU and the empty `0x6B4` counts as OK |
| 0.7.6 | Watch the `0x500` rate in IDLE and DRIVE | 20 Hz. Lower means the main loop is starved (the ADC DMA interrupts fire ~400k times per second) |

### 0.8 Fans (`fix/fan-pwm` build)

Scope on PA8 (tractive fan, TIM1) and PC6 (accumulator fan, TIM8).

| ID | Do | Expect |
|---|---|---|
| 0.8.1 | IDLE | 25 kHz, 10% duty (high 4 µs of 40 µs) on both pins |
| 0.8.2 | DRIVE, send `0x0A0` with all three phase temps at 35.0 °C (350 in bytes 0–1, 2–3, 4–5) | Tractive fan ~50% within 0.5 s |
| 0.8.3 | `0x0A0` at 60 °C | Tractive fan 100%: **steady high**, no pulses |
| 0.8.4 | `0x0A0` at 10 °C | Tractive fan 0%: steady low |
| 0.8.5 | DRIVE, send `0x6B1` with BMS average temp 55.00 °C (5500 in bytes 2–3) | Accumulator fan 75% |
| 0.8.6 | Leave DRIVE (`0x6B6`), then send one more `0x0A0` and `0x6B1` | Both back to 10%, but only after the next temperature frame: duty is only recalculated when one arrives. On the car these frames come continuously |
| 0.8.7 | DRIVE, send `0x0A0` with a negative temperature (e.g. −5.0 °C = 0xFFCE) | Note the result: today it reads as ~6500 °C and the fan goes to 100% |
| 0.8.8 | With a fan connected | Fan speed rises with duty. If it **falls**, the fan driver inverts the signal: change the output polarity in CubeMX |

### 0.9 Without the debugger

| ID | Do | Expect |
|---|---|---|
| 0.9.1 | Disconnect the ST-LINK, power-cycle, repeat 0.2.3, 0.3.2, 0.4.4, 0.5.2, 0.6.1 | Same results. Timing in the log matches the debug runs |

## Phase 1: in the car, LV only

GLV on, Tractive System off. The real pedal box, buttons and harness. The real BMS may answer precharge, or keep using the CAN adapter.

| ID | Do | Expect / record |
|---|---|---|
| 1.1 | Calibrate the pedals (read `ADC_VAL`, not `pedal_percents`, which only updates in DRIVE) ([tuning-and-bench.md](tuning-and-bench.md#pedal-calibration)) | Released and full readings for APPS1, APPS2, BSE. Update `pedal_checks.h`, run `make -C tests` (the calibration tests catch a bad set) |
| 1.2 | Unplug the BSE | `ADC_VAL[2]` < 250 |
| 1.3 | Brake as hard as possible (two people if needed) | `ADC_VAL[2]` well below 2780 |
| 1.4 | Press the brake slowly; note `ADC_VAL[2]` when the pads first clamp the rotor | Must be **above 1000** by then, and a resting foot must stay below 1000 (EV.4.7.1, EV.10.6.2) |
| 1.5 | Repeat 0.1.4 on the car harness, with the LV system and fans running | Noise still small. Watch for jumps when fans/pump switch |
| 1.6 | Connect the RMS GUI to the inverter and read: CAN Command Message Active, **CAN Timeout**, Motor Torque Limit, REGEN Torque Limit, direction setting | Record every value. Set CAN Timeout short (team decision). This must be done before phase 2. **Keep it above 150 ms for now:** the VCU sends nothing during the 150 ms lockout after RTD, so a shorter timeout may fault the inverter on every RTD (check in 1.7) |
| 1.7 | Inverter LV on, VCU goes PRECHARGED → RTD (real BMS or CAN adapter) | Inverter's own state messages show it leaving lockout after our enable-0 frames, then accepting enable 1. No "CAN command lost" fault during the 150 ms gap |
| 1.8 | Repeat phase 0.5 faults on the car harness, unplugging at the real connectors | Same results. Each connector can be unplugged without disturbing the others (T.4.2.7, T.4.3.2) |
| 1.9 | Repeat 0.3 RTD tests with the real buttons | Same results. Note any bounce (scope on the button pins if RTD fires oddly) |

## Phase 2: Tractive System active, wheels off the ground

`MAX_TORQUE` reduced (safety rule 2). Team HV procedures in force.

| ID | Do | Expect / record |
|---|---|---|
| 2.1 | Precharge with the real BMS | PRECHARGED 6 s after the request. Record actual BMS reply time on the log |
| 2.2 | RTD with brake held | Sound, lockout frames, drive. Measure the sound at 2 m: ≥ 80 dBA (EV.10.7.2, IN.10.3) |
| 2.3 | Small throttle | **Wheels turn forward** (EV.3.1.1). If backward: shut down, then fix the direction byte before anything else |
| 2.4 | Throttle sweep and lift-off | Motor torque feedback from the inverter follows `0x0C0`. Lift-off drops torque at once |
| 2.5 | Wheels spinning at low torque, unplug the VCU's CAN connector | The inverter stops applying torque within its CAN timeout. Record the time from the last `0x0C0` to the inverter disabling |
| 2.6 | Inspection demos from [tech-inspection.md](tech-inspection.md#demos) with wheels spinning: unplug APPS1, APPS2, BSE; brake + throttle | Motor torque stops. Record the time from fault to the 0 Nm frame (≤ 10 ms) |
| 2.7 | Open the shutdown circuit while spinning | `0x6B6` → lockout frames → IDLE. Tractive System goes off |
| 2.8 | Spin up, lift off fully | No negative torque command, `0x502` bit 7 never set |
| 2.9 | Hold a steady speed; measure wheel speed (optical tachometer or phone slow-motion video) and compare with the VCU's car speed and the inverter rpm | Data for the speed fix on `feature/regen` (gear ratio, tire size) |
| 2.10 | Raise `MAX_TORQUE` in steps (10 → 20 → 40), repeating 2.4 and 2.6 at each | Same behavior at each step. Full 80 Nm only once the team is happy |
| 2.11 | Run for 10+ minutes | Fans/pump follow real inverter and BMS temps. No unexpected cuts in the log |

## Phase 3: tech inspection rehearsal

Run [tech-inspection.md](tech-inspection.md) from start to finish, with someone playing the inspector who hasn't seen the plan. Time it, note every hesitation, and fix the doc where it was unclear.

## Edge cases

Cases worth deliberately provoking. Most are already in the phases above (ID given); the rest are one-off checks.

| Case | How to provoke | Expect | Why it matters | Test |
|---|---|---|---|---|
| Throttle held at power-up or at RTD | Hold the pedal while booting / entering drive | No torque until released below 5% | Unexpected motion at RTD | 0.1.2, 0.3.8 |
| Brake tapped at the very end of the RTD hold | Press the brake in the last ~0.2 s | Passes today | EV.10.6.2 "pressed and held" decision | 0.3.4 |
| Failed BSE at RTD | Unplug, or short to 3.3 V | RTD refused | A failed sensor must not satisfy a safety check | 0.3.5, 0.3.6 |
| Fault shorter than 100 ms | Flick a connector | Cut, then recovers, no latch | A noisy connector mustn't strand the car, but must still cut | 0.5.1 |
| Connector flickering back after a real fault | Unplug > 100 ms, replug | Stays cut until reset | A flickering connector must not give torque back | 0.5.2 |
| Signal shorted to GND | Jumper to GND | Same as open circuit | Both failure directions are caught | 0.5.6 |
| APPS signals shorted together | Jumper the two signals | Cut within 100 ms | T.4.2.3; thin 10.7% margin | 0.5.15 |
| Disagreement pulses under 100 ms | Pulse the offset | No cut | Timer restarts correctly | 0.5.10 |
| Brake released during a crosscheck cut | 0.5.11 then 0.5.12 | Still cut until APPS < 5% | EV.4.7.2 "with or without brake" | 0.5.12 |
| Pedal hovering at 5% | Hold right at the inflection | No jumping torque | Driveability, filter restart | 0.4.6 |
| Lift-off at speed | Lift fully above 5 km/h | 0 Nm, never negative | Regen must be off | 0.4.9, 2.8 |
| Shutdown during the RTD sound | `0x6B6` mid-sound | Ends in IDLE, lockout frames last | CAN RX is blocked during the sound | 0.6.2 |
| Shutdown during precharge wait | `0x6B6` within 6 s of the request | IDLE, no late timeout | Stale timers | 0.6.3 |
| BMS never answers precharge | No `0x6B4` | VCU halts in `Error_Handler()` until power-cycled | Is a full halt the right reaction? | 0.2.6 |
| Precharge reply out of turn | `0x6B4` when not waiting | Ignored | Stray frames mustn't skip precharge | 0.2.7 |
| Button bounce | Fast taps, then a hold | One action only | Double requests | 0.2.9, 1.9 |
| RTD pressed during the sound | Press again mid-sound | Nothing | Buttons are disabled during the sound | 0.3.9 |
| Heavy bus load | Flood the bus | `0x0C0` never skipped | T.4.2.12 | 0.7.1 |
| CAN cable unplugged in drive | Pull the connector | Inverter stops within its timeout | T.4.2.12; also covers a halted VCU | 0.7.2, 2.5 |
| CAN bus-off | Short CANH–CANL briefly | Note if the VCU recovers | Without recovery only the inverter timeout protects the car | 0.7.3 |
| Debugger halt in drive | (Only after 1.6, wheels off the ground) Suspend in CubeIDE | Inverter stops within its timeout | Same as a crashed VCU | 2.5 |
| No debugger attached | Run the key tests standalone | Same behavior | Debug builds and SWD can hide timing issues | 0.9.1 |
| Fan at 0% and 100% | Inject temperatures | Steady low / steady high | PWM edge values | 0.8.3, 0.8.4 |
| LV dip during the RTD sound | Watch LV voltage with the scope while the sound plays | No VCU reset (it would show as boot traffic in the log) | The speaker draws current | — |
| Inverter faults while the VCU is in drive | Trigger or wait for an inverter fault (e.g. RMS GUI) | Note what the VCU does: today it **doesn't read inverter faults** and keeps sending torque requests | Gap to decide on | — |
| Ground offset on the pedal sensors | Wiggle the pedal sensor ground while watching `ADC_VAL` | No false faults; a lost ground should read out of range | Harness quality | — |

## Known problems to confirm

From the code review of 2026-10-18 (three independent reviewers, the top items rechecked against the code). None are fixed yet. Ordered by severity.

| # | Problem | Effect | Test |
|---|---|---|---|
| 1 | CAN RX can't run during the 2.6 s RTD sound, and the RX FIFO (3 frames, all IDs accepted, drops new frames when full) overflows | A shutdown frame (`0x6B6`) during the sound is lost; the VCU enters DRIVE with shutdown power gone | 0.6.2 |
| 2 | The RX interrupt reads only one frame per new arrival | After any burst, 2 stale frames stay queued forever; every later frame is handled two frames late, or never on a quiet bus | 0.6.5 |
| 3 | No CAN bus-off recovery | After bus-off the VCU is deaf and mute until power-cycled; only the inverter timeout stops the motor | 0.7.3 |
| 4 | 150 ms with no `0x0C0` after the lockout frames | A short inverter CAN timeout may fault on every RTD | 1.6, 1.7 |
| 5 | One agreeing APPS sample restarts the 100 ms disagreement timer | A borderline APPS fault can keep torque on | 0.5.17 |
| 6 | Intermittent faults (< 100 ms bursts) never latch | Each burst still cuts, but the latch the docs describe never happens | 0.5.18 |
| 7 | Precharge: a second PRCHG hold forgets an OK; a missing or late reply halts the VCU | VCU stuck or halted with HV possibly up | 0.2.6, 0.2.10, 0.2.11 |
| 8 | Extended-ID and empty frames are treated as normal ones | A stray frame can reset the VCU or fake a precharge OK | 0.7.5 |
| 9 | `0x500` pedal values only update in DRIVE and overflow on fault readings; signed inverter temperatures read as unsigned | Misleading telemetry; fan at 100% below 0 °C | 0.7.6, 0.8.7 |

## Decisions waiting on bench results

| Decision | Bench input | Where it's tracked |
|---|---|---|
| RTD brake "pressed and held" for the whole 1 s | 0.3.4, driver feedback | [tech-inspection.md](tech-inspection.md#before-the-event), stash "RTD brake held for whole 1 s hold" |
| Send lockout frames before the RTD sound | 0.6.2, 1.7 | Deferred |
| What a missing precharge reply should do (halt vs return to IDLE) | 0.2.6 | — |
| `PEDAL_MODE` (`ONLY_APPS2` vs `MIN_APPS`) | 0.1.4, 1.1, 1.5 noise and agreement | [torque-control.md](torque-control.md#pedal-scaling) |
| BSE thresholds and "pressed" threshold | 1.2–1.4 | [tech-inspection.md](tech-inspection.md#before-the-event) |
| Inverter CAN timeout value | 1.6, 2.5 | [tech-inspection.md](tech-inspection.md#before-the-event) |
| Direction byte | 2.3 | [tech-inspection.md](tech-inspection.md#before-the-event) |
| Fan PWM polarity | 0.8.7 | README known issues |
| Gear ratio and tire size for car speed | 2.9 | `feature/regen` |
| Whether the VCU should react to inverter faults | Edge cases table | — |

## Results log

Copy this table into the session notes (one per session).

| Field | Value |
|---|---|
| Date | |
| People | |
| Phase | |
| Commit flashed | |
| `MAX_TORQUE` | |
| CAN log file(s) | |

| Test ID | Pass / fail | Measured value | Notes |
|---|---|---|---|
| | | | |
