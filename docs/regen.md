# Regen (`feature/regen`)

Lift-off regen, kept on its own branch until it passes the bench and car checks below. `feature/torque-mapping` ships with `REGEN_ENABLED 0`; this branch ships with 1. Bug fixes go torque-mapping → regen, never back, until then.

Stages C1 and C2 of the plan are done (host-tested, not yet run on the car). C3–C5 are later.

## What it does

| Pedal | Torque |
|---|---|
| 8–100% | Drive, from `torqueMap_table` |
| 3–8% | 0 Nm (coast). A resting foot neither drives nor regens |
| 0–3% | Regen: 0 Nm at 3% down to −5 Nm at 0%, times the speed fade |

| Car speed | Regen fade |
|---|---|
| ≤ 8 km/h (and backward) | 0 (no regen) |
| 8–20 km/h | Linear 0 → 1 |
| ≥ 20 km/h | 1 (full −5 Nm at full lift) |

On top of that:
- **Brake past 5%:** no regen (large phase currents trip the inverter).
- **Ramp:** regen builds at most 40 Nm/s (−5 Nm takes 125 ms), on top of the 50 ms filter. Releasing regen (pedal back into coast, brake, slowing down) is instant.
- **Hard limit:** nothing goes past −30 Nm (`REGEN_MAX_TORQUE`), whatever the tuning.
- **EV.3.1.3:** the output still stays between 0 Nm and the driver request every tick.

All constants are in `Core/Inc/torque_map.h`; see the tuning table in [tuning-and-bench.md](tuning-and-bench.md).

**Feel change vs. torque-mapping:** drive torque now starts at 8% pedal instead of 5%, so there's 3% more dead travel before the car pulls. That's what makes room for the coast band. Tell the drivers before they drive this branch.

## Car speed (C1)

`torqueMap_carspeedKph(rpm)` = rpm × `MOTOR_FORWARD_RPM_SIGN` / `GEAR_RATIO` × π × `TIRE_DIAMETER_IN` × 60 / 39370.1. With 59/12 and 16 in, 1000 motor rpm = 15.6 km/h, so regen starts above ~510 rpm.

- **The old factor read ~48× high:** it multiplied by 59/12 instead of dividing and used a 32 in tire, so regen was allowed from ~0.1 km/h.
- **Signed:** rolling backward reads negative and never allows regen (EV.3.1.1). If the sign is wrong, forward reads negative and regen never engages, which is safe and obvious on the bench.
- **Stale guard:** `processInverter_RPM()` records the `0x0A5` time. If no frame arrives for 100 ms, the control loop uses 0 km/h, which ends regen on that tick.
- **Worst-case braking between frames:** a frame's speed is already old when the loop uses it, so `torqueMap_trustedCarspeed` assumes the car has been braking as hard as it can since then (`CARSPEED_MAX_DECEL_KPH_S`, 80 km/h/s, about 2.3 g) and lowers the speed to match, never below 0. At 100 Hz that costs at most 0.8 km/h. Without it, the Monte Carlo test (`tests/mc`) found regen at 3.4 km/h: the bus went quiet for 80 ms during a hard stop, and the driver let go of the brake while the VCU still believed 9.1 km/h.
- **VERIFY ON CAR:** gear ratio, tire diameter, rpm sign (bench plan 2.9). The 16 in tire is the smallest plausible one, so a bigger real tire makes speed read low: regen starts later, never earlier.

## Bench checks (before merging)

This branch's build differs from [bench-plan.md](bench-plan.md) in a few places: drive starts at 8% (0.4.2 settles lower: ~365 at 50% pedal), and regen is on, so 0.4.9 and the "Regen must be off" rule don't apply. Run these instead:

| # | Do | Expect |
|---|---|---|
| R.1 | Fake `0x0A5` at 0, 300, 500 rpm; full lift | 0 Nm, `0x502` bit 7 never set (≤ 7.8 km/h) |
| R.2 | Fake `0x0A5` at 2000 rpm (31 km/h) every 10 ms; full lift | Ramps to −5 Nm (`0x0C0` −50) in about 125–200 ms, `0x502` bit 7 set |
| R.3 | Same, pedal at 5% | 0 Nm (coast band) |
| R.4 | Same, press brake past 5% | 0 Nm on the next frame |
| R.5 | Same, stop sending `0x0A5` | 0 Nm within ~100 ms |
| R.6 | Same, send −2000 rpm | 0 Nm (backward) |
| R.7 | Sweep fake rpm 2000 → 0 over ~5 s, full lift | Regen fades to 0 by ~515 rpm (8 km/h), well before 5 km/h |
| R.8 | On the car (2.9): compare VCU speed with measured wheel speed | Within a few %, positive going forward. Fix `TIRE_DIAMETER_IN` / `MOTOR_FORWARD_RPM_SIGN` if not |
| R.9 | On the car: coast-downs with the drivers | Lift-off feel acceptable; car stable. Raise `REGEN_LIFTOFF_TORQUE` in small steps only after this |
| R.10 | Fake `0x0A5` at 700 rpm (10.9 km/h), full lift: first every 10 ms, then every 50 ms | 10 ms: steady regen, about −0.9 Nm (`0x0C0` −9). 50 ms: weak and pulsing, never past −0.3 Nm and at 0 Nm about a quarter of the time: the speed is trusted only down to 80 km/h/s × its age, so it reads below 8 km/h late in each gap |
| R.11 | Offset one APPS by ~15% (shim) **before** RTD, then go to DRIVE | Cut from the first frame (`0x502` bit 3, `0x0C0` torque 0), no 100 ms of torque. Removing the offset brings torque back. The `HAL_GetTick()` wrap version can't be benched; `test_reset_disagreement_cuts_across_tick_wrap` covers it |

## Later

- **C3 battery protection:** prefer Cascadia's BMS current limit (BMS sends `0x202`, `CAN_BMS_Limit_Enable` in EEPROM), no VCU code. EV.3.3.2 (600 V max) applies to regen voltage spikes.
- **C4 brake-pedal regen** (T.3.2.4), with the brakes team: rear-only regen shifts brake bias.
- **C5 driver regen knob** (off / low / high), once there's a dash.
