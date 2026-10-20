# Monte Carlo tests (`tests/mc`)

Thousands of random 2 s drives through the real firmware code (`pedal_checks.c`, `torque_map.c`), with every 1 ms tick checked against fixed rules. The unit tests check the cases we thought of; this looks for the ones we didn't. It runs as part of `make -C tests`, in about 2 s per build.

## What it runs

Two levels, both on by default:

| Level | Path | Finds |
|---|---|---|
| `map` | Random pedal, brake, speed and torque-cut inputs straight into `torqueMap_request()`, values often right on a threshold | Map and filter bugs: bounds, EV.3.1.3, ramps, regen gates, overtravel |
| `chain` | True pedal positions → sensor model → raw ADC counts → `pedalChecks_*` → `torqueMap_request()`, same calls in the same order as `calculateTorqueRequest()`. Car speed comes from a car model, through a lossy, late `0x0A5` | Pedal-check timing, faults, EV.4.7, false cuts, regen vs. the car's real speed |

The `chain` models, all random per run:

- **Sensors** ([sensors.c](../tests/mc/sensors.c)): calibration offset and gain error up to 1.5%, noise up to 4 counts RMS (clipped at 3σ). 0–2 wiring faults per run: open, short, stuck, offset (15–50% of travel), intermittent. About 40% are short glitches (1–200 ms), so both sides of the 100 ms timeouts get tested.
- **Car** ([vehicle.c](../tests/mc/vehicle.c)): point mass driven by the commanded torque, brake (1–1.8 g), drag and rolling resistance. Mass, tire size (16–19 in) and drag area vary. No wheel slip, no hills.
- **`0x0A5`** ([inverter_can.c](../tests/mc/inverter_can.c)): period 3–20 ms with jitter, 0.1–1.5 ms latency, up to 20% frame loss, sometimes a 20–500 ms outage. A frame carries the speed from when it was sent.
- **Clock:** `HAL_GetTick()` starts anywhere, sometimes right before it wraps.

**These numbers are guesses** until bench data replaces them. `no_false_cut` depends most on them: worse real noise could cause cuts this test can't predict.

## Rules

[oracle.h](../tests/mc/oracle.h) has the full list. They check limits, not exact values, so they hold however the map is tuned.

- **Map:** finite output; within `REGEN_MAX_TORQUE`..`MAX_TORQUE`; between 0 and the driver request (EV.3.1.3); cuts are instant and reset the filter; regen only with regen on, pedal in the regen band, ≥ 5 km/h and brake off; drive and regen ramp limits; more pedal never less torque; pedal below the inflection never drives; overtravel acts like the pedal's end stop.
- **Chain**, against the truth:
  - `range_fault`: a sensor out of range > 100 ms cuts torque and stays cut (T.4.2.9, T.4.3.3).
  - `range_instant`: ...and on the same tick. Firmware choice, stricter than the rules.
  - `apps_disagree`: APPS more than 10% apart for > 100 ms cuts torque (T.4.2.4, T.4.2.5).
  - `ev4.7`: brake + APPS > 25% cuts torque until APPS ≤ 5%.
  - `no_false_cut`: healthy sensors never trip a check.
  - `regen_true_speed`: regen only while the car really does ≥ 5 km/h forward (EV.3.3.3).

Sensor noise is bounded, so each reading lies in a known span. A chain rule demands a cut only when the whole span breaks it, and `no_false_cut` only judges ticks where nothing in any span could justify one. Borderline ticks aren't judged, so the test never fails on noise.

A coverage check fails the run if some rule never had anything to check (a generator that never reaches a case proves nothing about it).

## Running it

From the repo root, after `make -C tests`:

```bash
tests/build/test_mc_regen --runs 100000 --seed 7
```

- `--level map|chain`: one level only.
- `--seed S`: a different set of runs. The CI run uses seed 1, 5000 runs.
- `--replay RUN_SEED --level chain --trace trace.csv`: one run, tick by tick. A failure prints this command with its run seed.
- `--nonfinite` (map level): also feed NaN/Inf. Off in CI: the firmware's inputs can't be NaN/Inf. It shows the filter would stay NaN until the next cut if they ever were.

Builds: `test_mc` (regen off), `test_mc_regen`, `test_mc_min_apps` (regen on, `PEDAL_MODE=MIN_APPS`).

The summary also prints the lowest true speed at which regen was commanded: the margin to the 5 km/h limit.

## When a sweep fails

Replay the run and read the trace before touching the firmware. Of the first four sweep failures, two were harness bugs (overlapping seeds, and the oracle not clamping readings to the ADC range like the real ADC does).

Found and fixed so far:
- **Regen at 3.4 km/h** (`regen_true_speed`): `0x0A5` went quiet for 80 ms during a hard stop, and the driver let go of the brake while the VCU still believed 9.1 km/h. Fix: `torqueMap_trustedCarspeed` assumes the car braked as hard as it can since the last frame (`CARSPEED_MAX_DECEL_KPH_S`). Lowest regen speed went from 6.3 to 8.0 km/h.
- **APPS disagreement at drive entry** (`apps_disagree`): the no-grace cut after `pedalChecks_reset` relied on a 0 timestamp, which let torque through for 100 ms when `HAL_GetTick()` wrapped to 0 (49.7 days of uptime). Fix: the `apps_no_grace` flag.

## Mutation checks

To check the rules themselves, break a copy of a firmware file on purpose, build the test against it, and see that the right rule fails. Every deliberate bug that changes what the car does has been caught. The ones that weren't changed nothing (e.g. a looser slew limit that the low-pass filter never reaches). One gap: breaking `apps_no_grace` is only caught by sweeps of about 100k runs (the wrap is rare), so `test_reset_disagreement_cuts_across_tick_wrap` covers it.

## Not done yet

- **Calibrated noise and `0x0A5` timing** from bench logs (APPS/BSE at rest, at a few pedal positions and while driving; real `0x0A5` period and gaps).
- **Plots** of traces and sweep histograms.
- **Power limit (EV.3.3.1, 80 kW):** blocked. There's no power limit in `Core/` yet, and the rule is measured at the accumulator (pack V × I), so it needs pack and inverter models.
