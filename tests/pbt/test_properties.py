"""
Property-based tests (Hypothesis) of the real firmware files, through shim.c.

Hypothesis generates inputs, here mostly sequences of ticks, and checks
rules that must hold for all of them. When one fails it shrinks the input
to the smallest case that still fails (fewest ticks, simplest values) and
prints it, so a failure reads like a hand-written test.

The checks here are rule obligations only (what the
rulebook demands), not a full model of the car.

Run:  make -C tests pbt        (PBT_EXAMPLES=5000 for a longer search)
Rerun one failure: paste the @reproduce_failure line Hypothesis prints above
the test, or just run again: failures are saved in tests/build/hypothesis-db
and tried first.
"""

import ctypes
import math
import os
import sys
import traceback

from hypothesis import HealthCheck, given, note, settings
from hypothesis import strategies as st
from hypothesis.database import DirectoryBasedExampleDatabase

HERE = os.path.dirname(os.path.abspath(__file__))
BUILD = os.path.join(HERE, "..", "build")

lib = ctypes.CDLL(os.path.join(BUILD, sys.argv[1] if len(sys.argv) > 1 else "libvcu_pbt.so"))

for name, res, args in [
    ("pbt_pedal_reset", None, []),
    ("pbt_pedal_tick", ctypes.c_int, [ctypes.c_int] * 3 + [ctypes.c_uint32]),
    ("pbt_apps_diff", ctypes.c_float, [ctypes.c_int] * 2),
    ("pbt_apps_used", ctypes.c_float, [ctypes.c_int] * 2),
    ("pbt_brake_held_for_rtd", ctypes.c_int, [ctypes.c_int]),
    ("pbt_target", ctypes.c_float, [ctypes.c_float] * 3),
    ("pbt_filter_reset", None, []),
    ("pbt_request", ctypes.c_float, [ctypes.c_int] + [ctypes.c_float] * 5),
    ("pbt_filter_target", ctypes.c_float, []),
    ("pbt_carspeed", ctypes.c_float, [ctypes.c_int]),
    ("pbt_trusted_speed", ctypes.c_float, [ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32]),
    ("pbt_power_cap", ctypes.c_float, [ctypes.c_float]),
    ("pbt_power_cap_rpm", ctypes.c_float, [ctypes.c_float, ctypes.c_uint32, ctypes.c_uint32]),
    ("pbt_deci_nm", ctypes.c_int, [ctypes.c_float]),
]:
    fn = getattr(lib, name)
    fn.restype = res
    fn.argtypes = args


def const(name, ctype=ctypes.c_int):
    return ctype.in_dll(lib, "pbt_" + name).value


def f32(x):
    """Round to float32, as the firmware stores it."""
    return ctypes.c_float(x).value


K = {n: const(n) for n in [
    "regen_enabled", "pedal_mode", "apps1_valid_lo", "apps1_valid_hi", "apps2_valid_lo",
    "apps2_valid_hi", "apps1_released", "apps2_released", "apps1_full", "apps2_full",
    "bse_released", "bse_valid_lo", "bse_valid_hi", "bse_pressed", "stale_ms"]}
K.update({n: const(n, ctypes.c_float) for n in [
    "max_torque", "regen_max_torque", "regen_min_speed", "regen_brake_cancel", "slew",
    "regen_ramp", "power_limit_w", "efficiency", "motor_max_rpm"]})

# Rulebook numbers (FSAE 2027), not firmware settings.
APPS_MAX_DIFF = f32(0.10)  # T.4.2.4
TIMEOUT_MS = 100           # T.4.2.5, T.4.3.3
EV47_TRIGGER = f32(0.25)   # EV.4.7
EV47_RELEASE = f32(0.05)
REGEN_MIN_KPH = 5.0        # EV.3.3.3
POWER_RULE_W = 80000.0     # EV.3.3.1

DT = f32(0.001)
EPS = 1e-4
U32 = 2**32
EXAMPLES = int(os.environ.get("PBT_EXAMPLES", "500"))

SETTINGS = settings(
    max_examples=EXAMPLES,
    deadline=None,
    print_blob=True,
    database=DirectoryBasedExampleDatabase(
        os.environ.get("PBT_DB", os.path.join(BUILD, "hypothesis-db"))),
    suppress_health_check=[HealthCheck.too_slow, HealthCheck.data_too_large],
)

# ---------------------------------------------------------------------------
# Strategies
# ---------------------------------------------------------------------------


def near(*points, spread=3):
    """Counts at and right around the given boundaries."""
    return st.sampled_from(points).flatmap(
        lambda p: st.integers(max(0, p - spread), min(4095, p + spread)))


apps1_count = st.one_of(
    st.integers(0, 4095),
    st.integers(K["apps1_full"], K["apps1_released"]),
    near(K["apps1_valid_lo"], K["apps1_valid_hi"]))
apps2_count = st.one_of(
    st.integers(0, 4095),
    st.integers(K["apps2_full"], K["apps2_released"]),
    near(K["apps2_valid_lo"], K["apps2_valid_hi"]))
bse_count = st.one_of(
    st.just(K["bse_released"]),
    st.integers(0, 4095),
    near(K["bse_valid_lo"], K["bse_valid_hi"], K["bse_pressed"]))

# Run lengths around the 100 ms timers and the fault-clear time.
run_len = st.one_of(st.integers(1, 5), st.integers(95, 105), st.integers(1, 250))

def apps_counts(travel):
    a1 = K["apps1_released"] - travel * (K["apps1_released"] - K["apps1_full"])
    a2 = K["apps2_released"] - travel * (K["apps2_released"] - K["apps2_full"])
    return int(round(a1)), int(round(a2))


# Both APPS from pedal travel: in range, agreeing or not. Raw counts alone
# rarely give two valid readings that disagree, which is what T.4.2.5 is about.
travel_pair = st.tuples(st.floats(0.0, 1.0), st.floats(0.0, 1.0)).map(
    lambda t: (apps_counts(t[0])[0], apps_counts(t[1])[1]))
apps_pair = st.one_of(st.tuples(apps1_count, apps2_count), travel_pair)

released = (apps_counts(0.0)[0], apps_counts(0.0)[1], K["bse_released"], 1)

# A drive: a list of segments, each one reading held for some ticks, often
# after releasing the pedal (which clears the EV.4.7 cut armed at drive
# entry). Shrinks toward fewer, shorter segments and simpler readings.
pedal_segments = st.tuples(
    st.booleans(),
    st.lists(st.tuples(apps_pair, bse_count, run_len).map(lambda x: (*x[0], x[1], x[2])),
             min_size=1, max_size=12),
).map(lambda x: [released] + x[1] if x[0] else x[1])

# Ages in ms around the CARSPEED_STALE_MS boundary, and anywhere.
age_ms = st.one_of(st.integers(0, 400), st.integers(K["stale_ms"] - 2, K["stale_ms"] + 2))


# ---------------------------------------------------------------------------
# Pedal chain
# ---------------------------------------------------------------------------


def valid(a1, a2, bse):
    return (K["apps1_valid_lo"] <= a1 <= K["apps1_valid_hi"],
            K["apps2_valid_lo"] <= a2 <= K["apps2_valid_hi"],
            K["bse_valid_lo"] <= bse <= K["bse_valid_hi"])


@SETTINGS
@given(segments=pedal_segments, start=st.integers(0, U32 - 1))
def test_pedal_chain_meets_rule_obligations(segments, start):
    """Whatever the sensors read, the firmware cuts torque whenever a rule
    says it must:
      - any reading out of range, that tick (T.4.2.10, T.4.3.4)
      - out of range for more than 100 ms straight: cut until reset (latch)
      - APPS disagreeing for more than 100 ms straight (T.4.2.5)
      - brake + APPS over 25%, until APPS is back to 5% (EV.4.7); this is
        armed at drive entry, so a held throttle gives nothing
    Starts at any HAL_GetTick() value, so the timers cross the uint32 wrap."""
    lib.pbt_pedal_reset()
    now = start
    out_run = disagree_run = 0
    latched = False
    ev47 = True
    t = 0
    for a1, a2, bse, n in segments:
        ok1, ok2, okb = valid(a1, a2, bse)
        out = not (ok1 and ok2 and okb)
        disagree = lib.pbt_apps_diff(a1, a2) > APPS_MAX_DIFF
        apps = lib.pbt_apps_used(a1, a2)
        for _ in range(n):
            now = (now + 1) % U32
            t += 1
            bits = lib.pbt_pedal_tick(a1, a2, bse, now)
            out_run = out_run + 1 if out else 0
            disagree_run = disagree_run + 1 if disagree else 0
            if out_run > TIMEOUT_MS + 1:
                latched = True
            if apps > EV47_TRIGGER and bse > K["bse_pressed"]:
                ev47 = True
            if apps <= EV47_RELEASE:
                ev47 = False
            must_cut = out or latched or disagree_run > TIMEOUT_MS + 1 or ev47
            if must_cut and not bits & 1:
                note(f"tick {t}: out={out} out_run={out_run} latched={latched} "
                     f"disagree_run={disagree_run} ev47={ev47} apps={apps:.4f} bits={bits:#x}")
            assert bits & 1 or not must_cut
            assert bool(bits & 4) == (not ok1) and bool(bits & 8) == (not ok2) \
                and bool(bits & 16) == (not okb)
            if latched:
                assert bits & 2, f"tick {t}: should be latched"


clean_step = st.tuples(
    st.one_of(st.floats(0.0, 1.0), st.floats(0.04, 0.06), st.floats(0.24, 0.26)),  # travel
    st.integers(-8, 8),           # APPS1 noise (counts)
    st.integers(-8, 8),           # APPS2 noise
    st.booleans(),                # brake pressed
    run_len)


@SETTINGS
@given(steps=st.lists(clean_step, min_size=1, max_size=15), start=st.integers(0, U32 - 1))
def test_healthy_pedals_only_cut_for_ev47(steps, start):
    """The other direction: two healthy APPS that agree (a few counts of
    noise) and a healthy BSE lose torque only for EV.4.7 (brake with APPS
    over 25%), and get it back as soon as APPS is down to 5%. Cutting more
    than the rules ask for strands the driver."""
    lib.pbt_pedal_reset()
    now = start
    a1, a2 = apps_counts(0.0)
    lib.pbt_pedal_tick(a1, a2, K["bse_released"], now)
    ev47 = False
    for travel, n1, n2, brake, n in steps:
        a1, a2 = apps_counts(travel)
        a1, a2 = a1 + n1, a2 + n2
        bse = K["bse_pressed"] + 200 if brake else K["bse_released"]
        apps = lib.pbt_apps_used(a1, a2)
        if apps > EV47_TRIGGER and brake:
            ev47 = True
        if apps <= EV47_RELEASE:
            ev47 = False
        # Right on 5% the rule says "less than", the firmware "at most": either.
        unsure = abs(apps - EV47_RELEASE) < 1e-6
        for _ in range(n):
            now = (now + 1) % U32
            bits = lib.pbt_pedal_tick(a1, a2, bse, now)
            if not unsure:
                assert bool(bits & 1) == ev47, \
                    f"cut={bits & 1} ev47={ev47} at APPS {apps:.4f} a1={a1} a2={a2} bse={bse}"


@SETTINGS
@given(bse=st.integers(0, 4095))
def test_rtd_brake_only_for_pressed_valid_bse(bse):
    """EV.10.6.2: RTD needs the brake pressed, and a failed BSE isn't pressed."""
    held = lib.pbt_brake_held_for_rtd(bse)
    assert held == (K["bse_pressed"] < bse <= K["bse_valid_hi"])


# ---------------------------------------------------------------------------
# Torque request
# ---------------------------------------------------------------------------

request_step = st.tuples(
    st.sampled_from([False] * 6 + [True]),                                   # cut
    st.one_of(st.floats(-0.2, 1.2), st.sampled_from([0.0, 0.03, 0.08, 1.0])),  # APPS travel
    st.one_of(st.just(0.0), st.floats(0.0, 1.0)),                            # BSE travel
    st.one_of(st.floats(-30.0, 120.0), st.sampled_from([0.0, 4.99, 5.0, 8.0, 20.0])),  # km/h
    st.one_of(st.just(K["max_torque"]), st.floats(-10.0, 120.0)),            # power cap
    run_len)


@SETTINGS
@given(steps=st.lists(request_step, min_size=1, max_size=12))
def test_request_obeys_rules_every_tick(steps):
    """Any pedal, brake, speed, cut and power-cap history, every tick:
      - a cut gives 0 Nm that tick (T.4.2.5, EV.4.7)
      - never more than the driver asked for, or the power cap (EV.3.1.3,
        EV.3.3.1); never regen deeper than REGEN_MAX_TORQUE
      - regen only at 5 km/h or more (EV.3.3.3), with the brake off, and
        only when regen is built in
      - torque rises no faster than the slew limit, regen builds no faster
        than REGEN_RAMP_NM_S"""
    lib.pbt_filter_reset()
    prev = 0.0
    t = 0
    for cut, apps, bse, speed, cap, n in steps:
        apps, bse, speed, cap = f32(apps), f32(bse), f32(speed), f32(cap)
        target = lib.pbt_target(apps, bse, speed)
        hi = max(min(target, max(cap, 0.0)), 0.0)
        for _ in range(n):
            t += 1
            out = lib.pbt_request(cut, apps, bse, speed, cap, DT)
            note(f"tick {t}: out={out} prev={prev} target={target} cap={cap}")
            assert lib.pbt_filter_target() == target
            if cut:
                assert out == 0.0
            assert out <= hi + EPS
            assert out >= K["regen_max_torque"] - EPS
            if out < 0.0:
                assert K["regen_enabled"] and speed >= REGEN_MIN_KPH and bse <= K["regen_brake_cancel"]
                assert out >= min(prev, 0.0) - K["regen_ramp"] * DT - EPS
            if out > 0.0:
                assert out <= max(prev, 0.0) + K["slew"] * DT + EPS
            prev = out


@SETTINGS
@given(apps=st.floats(0.1, 1.0), cap=st.floats(0.0, 120.0))
def test_held_pedal_gets_its_torque(apps, cap):
    """Liveness: holding the pedal with nothing wrong reaches the requested
    (or capped) torque within 0.5 s. The filter may slow torque, not lose it."""
    apps, cap = f32(apps), f32(cap)
    lib.pbt_filter_reset()
    out = 0.0
    for _ in range(500):
        out = lib.pbt_request(0, apps, 0.0, 30.0, cap, DT)
    want = min(lib.pbt_target(apps, 0.0, 30.0), cap)
    assert out >= 0.99 * want - EPS, f"out={out} want={want}"


# ---------------------------------------------------------------------------
# Car speed, power cap, 0x502
# ---------------------------------------------------------------------------


@SETTINGS
@given(rpm=st.integers(-32768, 32767), rx=st.integers(0, U32 - 1), age=age_ms)
def test_trusted_speed_only_lowers(rpm, rx, age):
    """The speed regen trusts is never above the 0x0A5 one, never flips a
    forward speed to backward, and is 0 once stale. Any HAL_GetTick() value."""
    v = lib.pbt_carspeed(rpm)
    tv = lib.pbt_trusted_speed(v, (rx + age) % U32, rx)
    if age > K["stale_ms"]:
        assert tv == 0.0
    elif v > 0.0:
        assert 0.0 <= tv <= v
    else:
        assert tv == v


@SETTINGS
@given(rpm=st.floats(-1e6, 1e6, allow_nan=False))
def test_power_cap_holds_any_speed(rpm):
    """EV.3.3.1: at any motor speed the cap keeps the modelled DC power at
    or under POWER_LIMIT_W, and even with a perfect drivetrain the shaft
    power stays under 80 kW."""
    rpm = f32(rpm)
    cap = lib.pbt_power_cap(rpm)
    omega = abs(rpm) * 2 * math.pi / 60
    assert 0.0 < cap <= K["max_torque"]
    assert cap * omega / K["efficiency"] <= K["power_limit_w"] * 1.0001
    assert cap * omega <= POWER_RULE_W


@SETTINGS
@given(rpm=st.floats(-10000, 10000), rx=st.integers(0, U32 - 1), age=age_ms)
def test_power_cap_assumes_max_speed_when_stale(rpm, rx, age):
    rpm = f32(rpm)
    got = lib.pbt_power_cap_rpm(rpm, (rx + age) % U32, rx)
    assert got == (K["motor_max_rpm"] if age > K["stale_ms"] else rpm)


@SETTINGS
@given(nm=st.floats(allow_nan=True, allow_infinity=True, width=32))
def test_deci_nm_saturates_and_truncates(nm):
    """0x0C0 / 0x502 torque counts: truncated toward zero, saturated to
    int16, 0 for NaN. A bad float must never wrap to the opposite sign."""
    d = lib.pbt_deci_nm(nm)
    assert -32768 <= d <= 32767
    if math.isnan(nm):
        assert d == 0
        return
    deci = f32(nm * 10.0)
    if deci >= 32767:
        assert d == 32767
    elif deci <= -32768:
        assert d == -32768
    else:
        assert d == math.trunc(deci)
    assert d == 0 or (d > 0) == (nm > 0)


# ---------------------------------------------------------------------------

def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    print(f"{sys.argv[1] if len(sys.argv) > 1 else 'libvcu_pbt.so'}: regen {K['regen_enabled']}, "
          f"PEDAL_MODE {K['pedal_mode']}, {EXAMPLES} examples per property")
    failed = 0
    for test in tests:
        try:
            test()
            print(f"ok   {test.__name__}")
        except Exception:  # Hypothesis has already printed the shrunk example
            failed += 1
            traceback.print_exc(limit=1)
            print(f"FAIL {test.__name__}")
    print("\nall properties held" if not failed else f"\n{failed} properties FAILED")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
