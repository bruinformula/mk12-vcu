#!/usr/bin/env python3
"""Unit tests for tools/gen_profile.py. Standard library only.

Run with `make -C tests profiles`, or directly: python3 tests/test_gen_profile.py
"""

import csv
import io
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
import gen_profile as gp  # noqa: E402

CAR = gp.read_car()
BASE_YAML = (HERE.parent / "profiles" / "base.yaml").read_text()


class Folder:
    """A temporary profiles/ folder holding base.yaml plus the given files."""

    def __init__(self, **files):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        (self.path / "base.yaml").write_text(BASE_YAML)
        for name, text in files.items():
            (self.path / f"{name}.yaml").write_text(text)

    def __enter__(self):
        return self.path

    def __exit__(self, *exc):
        self.tmp.cleanup()


def derived(name="drv", body=""):
    return f"name: {name}\nextends: base\n{body}"


class Parse(unittest.TestCase):
    def test_editor_format(self):
        # Exactly what tools/profile_editor exports, comments and all.
        p = gp.parse_yaml(
            "name: idan_autox\nextends: base\n\ntorque:\n"
            "  pedal_table: [0, 0.18, 0.33, 0.46, 0.57, 0.67, 0.76, 0.84, 0.91, 0.96, 1]"
            "   # torqueMap_table, fraction of max_nm\n\n"
            "filter:\n  tau_s: 0.03   # RC_TIME_CONSTANT\n  slew_nm_s: 3000   # SLEW_RATE_LIMIT\n")
        self.assertEqual(p["name"], "idan_autox")
        self.assertEqual(p["torque"]["pedal_table"][1], 0.18)
        self.assertEqual(p["filter"], {"tau_s": 0.03, "slew_nm_s": 3000})

    def test_round_trip(self):
        p = gp.parse_yaml(BASE_YAML)
        self.assertEqual(gp.parse_yaml(gp.to_yaml(p, CAR)), p)

    def test_rejects(self):
        for text, msg in [
            ("name: x\nbogus:\n  a: 1\n", "unknown section"),
            ("name: x\ncolor: red\n", "unknown key"),
            ("name: x\ntorque:\n  max_nm: fast\n", "isn't a number"),
            ("name: x\ntorque:\n  max_nm: nan\n", "isn't finite"),
            ("name: x\ntorque:\n  pedal_table: [0, 1\n", "list not closed"),
            ("name: x\n  max_nm: 80\n", "isn't inside a section"),
            ("name: x\ntorque:\n  max_nm: 80\n  max_nm: 90\n", "appears twice"),
            ("name: x\n- 1\n", "can't read"),
        ]:
            with self.subTest(msg=msg), self.assertRaisesRegex(gp.ProfileError, msg):
                gp.parse_yaml(text)


class Load(unittest.TestCase):
    def load(self, text, name="drv"):
        with Folder(**{name: text}) as d:
            return gp.load(d / f"{name}.yaml", CAR)

    def test_base_is_complete_and_matches_editor_defaults(self):
        with Folder() as d:
            e = gp.load(d / "base.yaml", CAR)
        self.assertEqual(e["torque"]["max_nm"], 80)
        self.assertEqual(e["regen"]["liftoff_nm"], -30)

    def test_merge_overrides_only_what_it_sets(self):
        e = self.load(derived(body="regen:\n  liftoff_nm: -12\n"))
        self.assertEqual(e["regen"]["liftoff_nm"], -12)
        self.assertEqual(e["regen"]["fade_hi_kph"], 6)  # from base, same section
        self.assertEqual(e["torque"]["max_nm"], 80)  # from base, other section
        self.assertEqual(e["overrides"], {"regen": {"liftoff_nm": -12}})

    def test_hash_ignores_name_and_tracks_values(self):
        same = self.load(derived("other"), "other")
        with Folder() as d:
            base = gp.load(d / "base.yaml", CAR)
        self.assertEqual(gp.profile_hash(same), gp.profile_hash(base))
        changed = self.load(derived(body="torque:\n  max_nm: 81\n"))
        self.assertNotEqual(gp.profile_hash(changed), gp.profile_hash(base))

    def test_regen_limit_comes_from_firmware(self):
        lo = CAR["REGEN_MAX_TORQUE"]
        self.load(derived(body=f"regen:\n  liftoff_nm: {lo}\n"))
        with self.assertRaisesRegex(gp.ProfileError, "liftoff_nm"):
            self.load(derived(body=f"regen:\n  liftoff_nm: {lo - 1}\n"))

    def test_rejects(self):
        cases = [
            (derived(body="torque:\n  max_nm: 221\n"), "outside"),
            (derived(body="torque:\n  max_nm: 80.5\n"), "whole number"),
            (derived(body="filter:\n  tau_s: 0\n"), "outside"),
            (derived(body="regen:\n  liftoff_nm: 1\n"), "outside"),
            (derived(body="torque:\n  pedal_table: [0, 0.5, 1]\n"), "needs 11 points"),
            (derived(body="torque:\n  pedal_table: [0, .2, .1, .3, .4, .5, .6, .7, .8, .9, 1]\n"), "never decrease"),
            (derived(body="torque:\n  pedal_table: [0, .1, .2, .3, .4, .5, .6, .7, .8, .9, .95]\n"), "end at 1"),
            (derived(body="torque:\n  pedal_table: [.1, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1]\n"), "start at 0"),
            (derived(body="torque:\n  max_nm: [80]\n"), "not a list"),
            (derived(body="torque:\n  pedal_table: 1\n"), "must be a list"),
            (derived(body="pedal:\n  coast_end: 0.08\n  drive_start: 0.08\n"), "coast_end must be below"),
            (derived(body="regen:\n  fade_lo_kph: 6\n"), "fade_lo_kph must be below"),
            (derived(body="regen:\n  boost: 1\n"), "unknown key"),
            ("name: drv\n", "extends: base"),
            ("name: drv\nextends: endurance\n", "extends: base"),
            ("extends: base\n", "missing name"),
            (derived("Drv"), "lowercase"),
            (derived("other"), "file name must be other.yaml"),
        ]
        for text, msg in cases:
            with self.subTest(msg=msg), self.assertRaisesRegex(gp.ProfileError, msg):
                self.load(text)

    def test_base_rules(self):
        with Folder() as d:
            (d / "base.yaml").write_text(BASE_YAML.replace("name: base", "name: base\nextends: base"))
            with self.assertRaisesRegex(gp.ProfileError, "can't extend"):
                gp.load(d / "base.yaml", CAR)
            (d / "base.yaml").write_text(BASE_YAML.replace("  ramp_nm_s: 200", ""))
            with self.assertRaisesRegex(gp.ProfileError, "ramp_nm_s .* is missing"):
                gp.load(d / "base.yaml", CAR)

    def test_load_all_order(self):
        with Folder(zed=derived("zed"), alpha=derived("alpha")) as d:
            self.assertEqual(list(gp.load_all(d, CAR)), ["base", "alpha", "zed"])


class Outputs(unittest.TestCase):
    def setUp(self):
        self.folder = Folder(drv=derived(body="torque:\n  max_nm: 120\n"))
        self.dir = self.folder.__enter__()
        self.profiles = gp.load_all(self.dir, CAR)

    def tearDown(self):
        self.folder.__exit__()

    def test_header(self):
        e = self.profiles["drv"]
        h = gp.gen_header(e, CAR)
        self.assertIn('#define PROFILE_NAME "drv"', h)
        self.assertIn(f"#define PROFILE_HASH 0x{gp.profile_hash(e)}u", h)
        self.assertRegex(h, r"#define MAX_TORQUE\s+120\n")  # integer, like the firmware
        self.assertRegex(h, r"#define REGEN_LIFTOFF_TORQUE\s+-30\.0f\n")
        # Every tunable the firmware defines, nothing else.
        names = re.findall(r"#define (\w+)", h)
        self.assertEqual(set(names) - {"PROFILE_GENERATED_H_", "PROFILE_NAME", "PROFILE_HASH", "TORQUE_MAP_TABLE"},
                         {c for _, _, c, *_ in gp.fields(CAR)})

    def test_lua(self):
        lua = gp.gen_lua(self.profiles, CAR, "abc1234")
        self.assertIn("at abc1234", lua)
        self.assertIn('order = { "base", "drv" }', lua)
        self.assertIn(f'hash = "{gp.profile_hash(self.profiles["drv"])}"', lua)
        self.assertIn("torque = { max_nm = 120, pedal_table = { 0, 0.1,", lua)
        self.assertIn(f"regen_max_nm = {CAR['REGEN_MAX_TORQUE']}", lua)
        self.assertEqual(lua.count("{"), lua.count("}"))

    def test_assetto_writes_only_two_files(self):
        with tempfile.TemporaryDirectory() as out, redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()):
            self.assertEqual(gp.main(["--profiles", str(self.dir), "assetto", out]), 0)
            self.assertEqual(sorted(p.name for p in Path(out).iterdir()),
                             ["profiles.lua", "profiles_index.ini"])
            ini = (Path(out) / "profiles_index.ini").read_text()
        self.assertIn("COUNT=2", ini)
        self.assertIn("NAME=drv", ini)

    def test_report_diff(self):
        r = gp.gen_report(self.profiles["drv"], self.profiles["base"], CAR)
        self.assertEqual(r["diff_vs_base"], {"torque.max_nm": {"base": 80, "profile": 120}})
        self.assertEqual(r["pedal_to_torque_at_60kph"][-1], [1.0, 120.0])

    def test_schema_in_repo_is_current(self):
        self.assertEqual(gp.SCHEMA_PATH.read_text(), gp.schema_text(CAR),
                         "run tools/gen_profile.py schema -o profiles/schema.json")

    def test_cli_error_exit(self):
        bad = self.dir / "bad.yaml"
        bad.write_text(derived("bad", "torque:\n  max_nm: 999\n"))
        with redirect_stderr(io.StringIO()) as err:
            self.assertEqual(gp.main(["--profiles", str(self.dir), "validate", str(bad)]), 1)
        self.assertIn("outside", err.getvalue())


class MatchesReferenceOutputs(unittest.TestCase):
    """The report's Python target_nm against the real C (reference outputs)."""

    def check(self, name, folder):
        e = gp.load(folder / f"{name}.yaml", CAR, gp.load(HERE.parent / "profiles" / "base.yaml", CAR))
        path = HERE / "reference" / "outputs" / f"{name}.csv"
        with open(path, newline="") as f:
            rows = [r for r in csv.DictReader(x for x in f if not x.startswith("#")) if r["kind"] == "map"]
        self.assertGreater(len(rows), 1000)
        for r in rows:
            got = gp.target_nm(e, CAR, float(r["apps"]), float(r["bse"]), float(r["speed_kph"]))
            self.assertAlmostEqual(got, float(r["target_nm"]), delta=1e-3, msg=str(r))

    def test_base(self):
        self.check("base", HERE.parent / "profiles")

    def test_spread(self):
        self.check("test_spread", HERE / "reference" / "fixtures")


if __name__ == "__main__":
    unittest.main(verbosity=1)
