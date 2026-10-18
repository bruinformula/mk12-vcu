# Tech inspection: VCU demos

What inspectors will ask the VCU to show, how to show it, and what proves it. Each demo gives the rule, what to do, what the car should do, and what to watch on `0x502` so a CAN log backs it up. Rule numbers refer to the FSAE Rules 2027 V1.0.

Rehearse the whole list on the bench first ([bench checklist](tuning-and-bench.md#bench-checklist)). Inspectors expect it to go smoothly.

## Before the event

Open items that can fail inspection. Close each one on the car and note the date here.

| Item | Rule | How to close it | Status |
|---|---|---|---|
| BSE open/short thresholds (250 / 2780) | T.4.3.4 | Unplug the BSE: `ADC_VAL[2]` < 250. Hardest brake press: well under 2780 | Open |
| "Brake pressed" threshold (ADC > 1000) means the brakes are actually engaged | EV.4.7.1, EV.10.6.2 | Press until the pads clamp the rotor and read `ADC_VAL[2]`. It must be above 1000 by then, and a resting foot must stay below | Open |
| APPS pull resistors: an open circuit must read 0% travel | T.4.2.3 | APPS read **lower** when pressed, so a pull-down sends an unplugged APPS to ~70 counts, past **100%**. The VCU sees that as a fault and cuts torque (T.4.2.10), but the rule asks for 0%. Ask the electrical team whether to change it to a pull-up, or how to justify it | Open |
| APPS1 and APPS2 differ enough | T.4.2.3 | Different offsets and gradients: yes (1400→320 vs 1965→470 counts). But at full pedal they're only 150 counts apart (about 4% of the ADC range). Check how "differ by minimum 10%" applies, with the electrical team. B5 will add a host test | Open |
| Inverter CAN timeout set short, CAN command mode on | T.4.2.12 | RMS GUI: "CAN Command Message Active" = 1, "CAN Timeout" short. The default (999 ms, or off) holds the last torque if the VCU stops sending | Open |
| `0x0C0` direction byte means forward | EV.3.1.1 | We send 0. Cascadia's doc says 0 = reverse. Confirm on stands that the wheels turn forward | Open |
| RTD sound level ≥ 80 dBA at 2 m | EV.10.7.2, IN.10.3 | Measure with a phone meter app at 2 m | Open |
| RTD sound length 1–3 s | EV.10.7.2 | 2.58 s measured from the WAV file | Done |
| Regen off | EV.3.3.3, IN.14.3.1 | `REGEN_ENABLED 0`. On the car, `0x502` bit 7 never set | Done in code |

## Setup

- Car on stands, wheels off the ground, Tractive System able to go active.
- CAN logger on FDCAN1 decoding `0x0C0` (torque command) and `0x502` (torque diagnostics). Frame layouts: [torque-control.md](torque-control.md#torque-diagnostics-frame-0x502).
- Three people: driver in the seat, one at the pedal box connectors (or breakout box), one on the logger calling out bits.
- **Recovering from a latched fault:** a sensor fault that lasts more than 100 ms keeps torque cut even after the connector goes back in. That's on purpose. To clear it, open the shutdown circuit (the VCU gets `0x6B6` and resets to IDLE), then precharge and RTD again. Tell the inspector this before unplugging anything.

`0x502` byte 5 bits used below: 0 APPS1 out of range, 1 APPS2 out of range, 2 BSE out of range, 3 APPS disagree, 4 brake + throttle, 5 fault latched, 6 torque cut, 7 regen. Byte 6 is `vcu_state` (0 IDLE, 1 PRECHARGED, 2 DRIVE).

## Demos

Run them in this order. Each starts from DRIVE with the pedal released unless it says otherwise.

### 1. Ready to Drive needs brake + button + Tractive System active

**Rule:** EV.10.6.2. **Tests:** `test_rtd_needs_brake_pressed`, `test_rtd_rejects_failed_bse`.

Starting from PRECHARGED (`vcu_state` 1):

| Do | Car does |
|---|---|
| Hold RTD 1 s, **no brake** | Nothing. No sound, stays PRECHARGED |
| Unplug the BSE, press the brake, hold RTD | Nothing. A failed BSE never counts as the brake being pressed |
| Plug the BSE back in, press the brake, hold RTD 1 s | RTD sound, then `vcu_state` goes to 2 |
| From IDLE (not precharged), brake + RTD | Nothing. RTD needs a successful precharge first |

### 2. Ready to Drive sound

**Rule:** EV.10.7, IN.10.3.

The sound plays once on entering drive: 2.58 s, inside the 1–3 s window. Inspectors measure the level at 2 m (≥ 80 dBA). No other VCU sound is similar.

### 3. Throttle held while entering drive

Not a rule by itself, but inspectors often try it. Enter drive with the throttle pressed: torque stays 0 Nm until the pedal is released below 5%. `0x502` bits 3 and 4 stay set until then, and bit 6 stays set too. **Test:** `test_reset_requires_pedal_release`.

### 4. APPS unplugged, one at a time

**Rule:** T.4.2.7 (checkable by unplugging), T.4.2.9, T.4.2.10 (open circuit = failure), T.4.2.5 (power stopped). **Tests:** `test_unplugged_apps_cuts_immediately`, `test_apps_range_fault_latches_after_100ms`, `test_request_cut_is_instant`.

| Do | Car does | `0x502` shows |
|---|---|---|
| Hold some throttle, unplug APPS1 | Torque 0 Nm in the next frame (≤ 10 ms) | Bit 0, bit 6; bit 5 after 100 ms |
| Plug APPS1 back in | Torque **stays 0** (latched) | Bit 5 and bit 6 stay set |
| Reset (see Setup), repeat with APPS2 | Same | Bit 1, bit 6, bit 5 |

APPS2 also sets the torque (`PEDAL_MODE` is `ONLY_APPS2`), but both sensors are range-checked and compared, so either one failing cuts torque.

### 5. BSE unplugged

**Rule:** T.4.3.2 (checkable by unplugging), T.4.3.3 (failure > 100 ms stops power), T.4.3.4. **Tests:** `test_bse_open_circuit_cuts_and_latches`, `test_bse_short_circuit_cuts_and_latches`, `test_hard_braking_is_not_a_fault`.

Same as demo 4, with bit 2. Torque is cut immediately (the rule allows up to 100 ms), and latches after 100 ms.

### 6. APPS disagreement (only with a breakout box)

**Rule:** T.4.2.4, T.4.2.5. **Tests:** `test_disagreement_cuts_after_100ms`, `test_agreement_restores_immediately`, `test_short_disagreement_restarts_timer`.

If the inspector has a breakout box, they can offset one APPS signal by more than 10% of travel. Torque continues for 100 ms, then cuts (bit 3, bit 6). It comes back as soon as the sensors agree again: a disagreement isn't latched, only an out-of-range reading is.

Without a breakout box, demo 4 covers T.4.2.7 (unplugging is one of the two allowed ways to check).

### 7. Brake + throttle

**Rule:** EV.4.7. The rule says the team must be able to demonstrate this. **Tests:** `test_brake_with_throttle_cuts_immediately`, `test_crosscheck_thresholds`, `test_crosscheck_holds_until_apps_below_5_percent`.

| Do | Car does | `0x502` shows |
|---|---|---|
| Hold throttle above 25%, press the brake | Torque 0 Nm in the next frame | Bit 4, bit 6 |
| Release the brake, stay on the throttle | Torque **stays 0** | Bit 4, bit 6 |
| Lift the throttle below 5% | Torque available again | Bits clear |
| Press the throttle again | Torque ramps up | — |

### 8. Brake test (dynamic)

**Rule:** IN.14.3.1: the brake test must use only the mechanical brakes. The Tractive System is switched off for the test anyway, and regen is off (`REGEN_ENABLED 0`), so the VCU adds nothing. `0x502` bit 7 never set in the log proves no regen was requested.

## Questions inspectors may ask

- **"What happens if the VCU stops sending?"** The inverter's CAN timeout stops torque once frames stop. Give the configured timeout here: ___ ms (see "Before the event").
- **"How do you know torque commands aren't lost?"** Every FDCAN1 frame goes through `canTx1()`, which can't be interrupted mid-send (T.4.2.12). `0x502` carries a counter that goes up by 1 per torque frame, so a gap in the log shows a skipped frame.
- **"Which APPS sets the torque?"** APPS2 (`PEDAL_MODE ONLY_APPS2`). APPS1 is range-checked and compared with APPS2 every 1 ms tick.
- **"How fast is power stopped?"** On the same 1 ms tick the fault is seen. The next torque frame (within 10 ms) says 0 Nm. No filtering on a cut.
- **"Can the car drive backward?"** No reverse command is ever sent (EV.3.1.1). Direction byte confirmation is in "Before the event".
- **"Does the VCU ever add torque?"** No. The output is clamped between 0 and the driver's request every tick (EV.3.1.3), checked by a 200,000-tick random test (`test_request_never_exceeds_driver_request`).
- **"Regen?"** Off. When it's turned on later, it's blocked below 5 km/h (EV.3.3.3).

The full rule → code → test table is in [torque-control.md](torque-control.md#rule--code--test). All tests run in CI on every push, or locally with `make -C tests`.
