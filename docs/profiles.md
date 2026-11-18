# Driver profiles

How a tune goes from a YAML file to the sim (and, later, the car). Profiles, schema, generator and reference outputs are done. The firmware doesn't read profiles yet, and the sim side is on `feature/vcu-profiles` in `mk12-assetto-sim` (see below).

## Profile files

`profiles/base.yaml` holds every tunable at the firmware's current values. Any other profile sets `extends: base` and lists only what it changes:

```yaml
name: endurance
extends: base

torque:
  max_nm: 60

regen:
  liftoff_nm: -12
```

The editor (`tools/profile_editor/`) reads and writes this format. Write it by hand too: `profiles/schema.json` gives autocomplete and range errors in VS Code with the YAML extension (add `# yaml-language-server: $schema=schema.json` as the first line). The format is a small YAML subset: `name:`, `extends:`, and sections of `key: number` or `key: [numbers]`, with `#` comments. Anything else is rejected rather than guessed at.

| Key | Firmware name | Range |
|---|---|---|
| `torque.max_nm` | `MAX_TORQUE` | 20–150 Nm, whole number |
| `torque.pedal_table` | `torqueMap_table` | 11 points from 0 to 1, never decreasing |
| `pedal.coast_end` | `REGEN_END_PERCENT` | 0.005–0.10, below `drive_start` |
| `pedal.drive_start` | `APPS_INFLECTION_PERCENT` | 0.02–0.20 |
| `regen.liftoff_nm` | `REGEN_LIFTOFF_TORQUE` | `REGEN_MAX_TORQUE` (−30) to 0 Nm |
| `regen.fade_lo_kph` | `REGEN_FADE_LO_KPH` | 0–40, below `fade_hi_kph` |
| `regen.fade_hi_kph` | `REGEN_FADE_HI_KPH` | 5–60 |
| `regen.ramp_nm_s` | `REGEN_RAMP_NM_S` | 5–200 Nm/s |
| `filter.tau_s` | `RC_TIME_CONSTANT` | 0.005–0.2 s |
| `filter.slew_nm_s` | `SLEW_RATE_LIMIT` | 200–10000 Nm/s |

Car-fixed values (regen hard limit, regen cutoff speed, brake cancel, power cap, gear ratio, tire) are not in profiles. The generator reads them from `torque_map.h` and `power_limit.h`. The 150 Nm ceiling is a team setting in `tools/gen_profile.py` (`MAX_NM_CEILING`), not a firmware limit.

With base's 50 ms filter, `slew_nm_s` never binds: the filter alone moves at most about 1.6 Nm per 1 ms tick, under base's 2 Nm per tick. It only matters with a shorter `tau_s` or a higher `max_nm`.

## Generator

`tools/gen_profile.py` needs plain Python 3, no packages:

```bash
python3 tools/gen_profile.py check
```

Validates every profile in `profiles/`, checks `base.yaml` matches the firmware's `#define`s and table, and checks `schema.json` is current. Other commands:

- `validate FILE... [--base profiles/base.yaml]`: check drafts, e.g. from the sim repo.
- `header NAME [-o FILE]`: the C header (`MAX_TORQUE`, `TORQUE_MAP_TABLE`, `PROFILE_NAME`, `PROFILE_HASH`...). Not used by the firmware yet.
- `assetto DIR [--source COMMIT]`: writes `profiles.lua` (every profile plus the car-fixed values, for `vcu.lua`) and `profiles_index.ini` (names and hashes), and nothing else. It warns if the VCU's gear ratio stops matching the sim's 3.909 (43/11).
- `report NAME [-o FILE]`: JSON with the resolved values, the diff against base, and pedal → torque and regen-vs-speed curves (map target only, no filter).
- `schema -o profiles/schema.json`: run this after changing a range in the script.

The hash is the first 8 hex digits of a SHA-256 over the resolved values, not the name, so two profiles with the same numbers share a hash.

## Reference outputs

`tests/reference/outputs/<profile>.csv` is the real `torque_map.c` and `power_limit.c` run over a fixed sweep: the map over pedal, brake and speed; speed from rpm; the power cap; and six tick-by-tick traces (tip-in, ramp, coast-down with braking, torque cuts, a power-cap run, and a 333 Hz pseudo-random drive at the sim's physics rate). Every other implementation is checked against them. The columns are listed at the top of [`tests/reference/reference.c`](../tests/reference/reference.c).

They cover `base` plus the fixtures in `tests/reference/fixtures/`. `test_spread` moves every tunable, including 150 Nm so the power cap binds, which tests that a port reads each value. Approved profiles don't get their own outputs: the pipeline is the same code for all of them.

```bash
make -C tests profiles        # everything above, plus the generator's tests; CI runs it
make -C tests reference-update   # after a deliberate pipeline change: rewrite the reference outputs
```

`profiles` fails if the reference outputs are out of date, so any change to the pipeline's output shows up as a diff of the CSVs in the same PR. Builds use `-ffp-contract=off`, so GCC and Clang, x86 and ARM, print the same numbers.

To check a port, write the same columns for the same rows and compare:

```bash
python3 tests/reference/compare.py tests/reference/outputs/base.csv my_port.csv --tol 1e-3
```

## The sim side (`mk12-assetto-sim`)

`vcu.lua` there is a line-by-line Lua port of `torque_map.c` and `power_limit.c`, and `powertrain.lua` drives the AC motor with it at the VCU's 1 kHz tick and 100 Hz `0x0C0` rate. `tools/vcu_sync.py` in that repo fetches from here at the tag in its `VCU_REF` file (sparse git checkout): `tools/gen_profile.py`, `profiles/`, the headers the generator reads (`torque_map.h`, `power_limit.h`, `motor_control.h`, `main.c`, `torque_map.c`), and `tests/reference/`. It builds `profiles.lua` from the approved profiles here, validates drafts with the same generator, and checks `vcu.lua` against the reference outputs within 0.001 Nm.

So a change here that the sim should follow is: merge, tag (`profiles-v2`...), bump `VCU_REF` there, run `python tools/vcu_sync.py build`, commit `profiles.lua`. A change to `torque_map.c` also needs the same change in `vcu.lua`; that repo's CI fails until it's ported.

## Not done yet

- **Firmware wiring:** the tunable headers include `profile_generated.h`, `make PROFILE=<name>`, the profile ID on CAN, and CI over every profile. Until then the firmware keeps its own copy of base's values, and `check` makes sure the two agree.
- **Sim side:** the editor move and the in-game profile picker. The sim port still needs a drive on the sim PC: positive `ac.overrideEngineTorque` and `car.gas` while the gas input is overridden are untested.
