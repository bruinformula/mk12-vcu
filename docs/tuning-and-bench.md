# Tuning and bench checks

How to change the torque behavior safely, and what to check on the car before driving new firmware. For how the torque path works, see [torque-control.md](torque-control.md).

## Before and after any tuning change

1. Make the change in one place (the constants below).
2. Run the tests:
   ```bash
   make -C tests
   ```
3. If you changed the pedal map, plot it (see [README → Tools](../README.md#tools)).
4. Build in CubeIDE (0 errors, 0 warnings), flash, and run the bench checks below that cover what you changed.
5. Commit the change and any test you deliberately updated together, with the reason in the commit message.

## Tuning knobs

| Constant | File | Default | What it does | Watch out for |
|---|---|---|---|---|
| `torqueMap_table` | `torque_map.c` | Straight line | Pedal feel: fraction of `MAX_TORQUE` at 11 points from 5% to 100% pedal | Must start at 0, end at 1 and never decrease, or the VCU halts at boot. `test_half_travel_gives_half_torque` fails for any non-straight map; update it with the new value |
| `MAX_TORQUE` | `torque_map.h` | 80 Nm | Torque at full pedal. The whole map scales with it | No test changes. Lower it for events or first drives |
| `REGEN_ENABLED` | `torque_map.h` | 0 (off) | 0: below 5% pedal coasts at 0 Nm. 1: regen | Keep 0 on `feature/torque-mapping`. Regen work happens on `feature/regen`. `0x502` bit 7 stays 0 while off |
| `REGEN_MAX_TORQUE` | `torque_map.h` | −30 Nm | Regen at 0% pedal when moving | Larger regen = more phase current; raise slowly |
| `REGEN_MIN_CARSPEED` | `torque_map.h` | 5 km/h | No regen below this | **EV.3.3.3: never below 5.** `test_regen_cutoff_meets_ev_3_3_3` enforces this. Also see the speed conversion issue in the README |
| `APPS_INFLECTION_PERCENT` | `torque_map.h` | 5% | Where coast (or regen) ends and drive torque starts | Dead band between regen and drive. Separate from the crosscheck's 5% restore point (`CROSSCHECK_RESTORATION_APPS_PERCENT`) |
| `RC_TIME_CONSTANT` | `torque_map.h` | 50 ms | Smoothing when torque rises. Lift-off is never smoothed (EV.3.1.3) | Below ~40 ms the 2000 Nm/s slew limit starts to engage, and 5 filter tests change expectations. That's expected; check the new numbers make sense |
| `SLEW_RATE_LIMIT` | `torque_map.h` | 2000 Nm/s | Max torque rise per second | Currently never engages (the LPF is slower). Only matters with a smaller τ |
| `*_ADC_MIN_VAL` / `*_ADC_MAX_VAL` | `pedal_checks.h` | See [calibration](#pedal-calibration) | Pedal calibration | Re-measure after any pedal box or sensor change |
| `BSE_ADC_OPEN_CIRCUIT_VAL`, `BSE_ADC_SHORT_CIRCUIT_VAL` | `pedal_checks.h` | 250 / 2780 | BSE wiring fault thresholds | **2780 is not verified yet.** See bench check 3 |

Don't change the rule-defined values without reading the rule first:
- 10% and 100 ms (APPS disagreement, T.4.2.4/5),
- 25% and 5% (crosscheck, EV.4.7),
- 100 ms sensor latch (T.4.3.3).

## Pedal calibration

Use Live Expressions in CubeIDE while debugging. `tools/build_live_expressions_guide.py` generates a step-by-step guide (`STM32CubeIDE_Live_Expressions_Operation_Guide.docx`).

1. Watch `ADC_VAL[0]` (APPS1), `ADC_VAL[1]` (APPS2) and `ADC_VAL[2]` (BSE).
2. Note each value with the pedal released and fully pressed (APPS against the stop, BSE at a firm, hard press).
3. Put them in `pedal_checks.h`:
   - **APPS:** `*_ADC_MIN_VAL` = released, `*_ADC_MAX_VAL` = full. APPS read lower when pressed, so MIN > MAX.
   - **BSE:** `BSE_ADC_MIN_VAL` = released, `BSE_ADC_MAX_VAL` = full. BSE reads higher when pressed.
4. Check `pedal_percents` reads about 0 released and about 1 full, and that APPS1 and APPS2 agree to well within 10% across the whole travel.

An APPS reading more than 125 counts past either end counts as a wiring fault. If normal pedal travel gets close to that, the calibration is off.

## Bench checklist

Do these with the car on stands or wheels off the ground, and a CAN logger on FDCAN1. Watch `vcu_state`, `requestedTorque`, `plausibility_checks` and `pedal_percents` in Live Expressions, and frames `0x0C0` and `0x502` on the bus.

1. **Boot.** The VCU doesn't halt in `Error_Handler()`. A halt right at boot with a new map means `torqueMap_tableIsValid()` rejected it.
2. **Pedal readings.** Released and full readings match the calibration (above).
3. **BSE thresholds** (still open):
   - Unplug the BSE: `ADC_VAL[2]` must read **below 250**.
   - Brake as hard as you can: `ADC_VAL[2]` must stay **well below 2780**.

   If either fails, fix `BSE_ADC_OPEN_CIRCUIT_VAL` / `BSE_ADC_SHORT_CIRCUIT_VAL` first.
4. **Entering drive.** Precharge, then hold RTD with the brake pressed. On the bus:
   - up to 5 frames on `0x0C0` with 0 Nm and enable = 0,
   - then about 150 ms with no `0x0C0`,
   - then frames every **10 ms** with enable = 1, each followed by a `0x502` whose counter (byte 7) goes up by 1 and whose bytes 2–3 match the `0x0C0` torque.
5. **Throttle held at RTD.** Enter drive with the throttle pressed: torque stays 0 until you release it.
6. **Normal throttle.** Torque rises smoothly when you press. When you lift, it drops to 0 in the next frame (EV.3.1.3).
7. **APPS unplugged** (T.4.2.9/10), one sensor at a time while holding throttle:
   - Torque goes to 0 in the next frame.
   - `0x502` byte 5 shows that sensor's out-of-range bit (bit 0 or 1) and bit 6 (torque cut).
   - Unplug for more than 100 ms, then replug: torque **stays 0** and bit 5 (`sensor_fault_latched`) stays set until shutdown power is cycled (`0x6B6` → IDLE).
8. **BSE unplugged** (T.4.3.3/4): same as 7, with the BSE (bit 2).
9. **Brake + throttle** (EV.4.7):
   - Hold throttle above 25%, then press the brake: torque goes to 0.
   - Release the brake while staying on the throttle: torque stays 0.
   - Lift below 5%: torque comes back.
10. **APPS disagreement** (T.4.2.4/5). If you have a breakout box, offset one APPS by more than 10%: torque stays on for 100 ms, then cuts. It comes back once the sensors agree.
