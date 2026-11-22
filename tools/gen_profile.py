#!/usr/bin/env python3
"""Driver/event profile generator (docs/profiles.md).

One profile YAML in profiles/ becomes the VCU's tunables and the Assetto
sim's, so a driver practises in the sim what the car does.

  gen_profile.py check                     validate profiles/, base matches firmware,
                                           schema up to date
  gen_profile.py validate FILE...          validate drafts (--base for their base)
  gen_profile.py header NAME [-o FILE]     C header for one profile
  gen_profile.py assetto DIR               profiles.lua + profiles_index.ini
  gen_profile.py report NAME [-o FILE]     JSON summary: curves, diff vs base
  gen_profile.py schema [-o FILE]          JSON Schema (profiles/schema.json)

Standard library only, so it runs on the sim PC with a plain Python 3.
Profiles use a small YAML subset, the one tools/profile_editor writes:
top-level `name:` and `extends:`, and sections of `key: number` or
`key: [numbers]`. Anything else is an error, not a guess.

Car-fixed values (regen hard limit, power cap, gear ratio...) are read from
the firmware headers, never copied, so a profile can't outrun the car.
"""

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
PROFILES_DIR = REPO / "profiles"
SCHEMA_PATH = PROFILES_DIR / "schema.json"
SECTIONS = ("torque", "pedal", "regen", "filter")
NAME_RE = re.compile(r"^[a-z][a-z0-9_]{0,30}$")

# Highest MAX_TORQUE a profile may ask for. Not a firmware limit (the
# inverter EEPROM allows 231 Nm and power_limit.c caps power); a team
# decision, same as the editor's. Raise it here when the car is ready.
MAX_NM_CEILING = 220

# Car-fixed values: (C name, file under Core/). Read at run time from the firmware.
CAR_DEFINES = [
    ("MIN_TORQUE", "Inc/torque_map.h"),
    ("REGEN_MAX_TORQUE", "Inc/torque_map.h"),
    ("REGEN_ENABLED", "Inc/torque_map.h"),
    ("REGEN_MIN_CARSPEED", "Inc/torque_map.h"),
    ("REGEN_BRAKE_CANCEL_PERCENT", "Inc/torque_map.h"),
    ("TORQUE_MAP_POINTS", "Inc/torque_map.h"),
    ("GEAR_RATIO", "Inc/torque_map.h"),
    ("TIRE_DIAMETER_IN", "Inc/torque_map.h"),
    ("MOTOR_FORWARD_RPM_SIGN", "Inc/torque_map.h"),
    ("POWER_LIMIT_W", "Inc/power_limit.h"),
    ("DRIVETRAIN_EFFICIENCY", "Inc/power_limit.h"),
    ("MOTOR_MAX_RPM", "Inc/power_limit.h"),
    ("CONTROL_LOOP_HZ", "Inc/motor_control.h"),
    ("TORQUE_TX_DIVIDER", "Src/main.c"),
]


class ProfileError(Exception):
    pass


def read_define(name, header):
    """A numeric #define from a firmware file (path under Core/)."""
    text = (REPO / "Core" / header).read_text()
    m = re.search(rf"^\s*#define\s+{name}\s+(.+?)\s*(//.*)?$", text, re.M)
    if not m:
        raise ProfileError(f"{header}: no #define {name}")
    expr = re.sub(r"(\d)f\b", r"\1", m.group(1))  # 59.0f -> 59.0
    if not re.fullmatch(r"[\d.\s()+\-*/eE]+", expr):
        raise ProfileError(f"{header}: can't read {name} = {m.group(1)}")
    return eval(expr, {"__builtins__": {}})  # digits and operators only


def read_car():
    """Car-fixed values from the firmware headers, as numbers."""
    return {name: read_define(name, header) for name, header in CAR_DEFINES}


def check_base_matches_firmware(base, car):
    """Until the firmware reads profile_generated.h, the
    firmware keeps its own copy of base's values. They must agree."""
    errors = []
    for sec, key, cname, *_ in fields(car):
        fw = read_define(cname, "Inc/torque_map.h")
        if fw != base[sec][key]:
            errors.append(f"{sec}.{key} = {base[sec][key]}, firmware {cname} = {fw}")
    src = (REPO / "Core" / "Src" / "torque_map.c").read_text()
    m = re.search(r"torqueMap_table\[TORQUE_MAP_POINTS\]\s*=\s*\{([^}]*)\}", src)
    if not m:
        raise ProfileError("torque_map.c: can't find torqueMap_table")
    fw_table = [float(x.strip().rstrip("f")) for x in m.group(1).split(",") if x.strip()]
    if fw_table != [float(x) for x in base["torque"]["pedal_table"]]:
        errors.append(f"torque.pedal_table = {base['torque']['pedal_table']}, firmware {fw_table}")
    if errors:
        raise ProfileError("base.yaml doesn't match the firmware:\n  " + "\n  ".join(errors))


def fields(car):
    """One entry per tunable: (section, key, C name, min, max, integer)."""
    return [
        ("torque", "max_nm", "MAX_TORQUE", 20, MAX_NM_CEILING, True),
        ("pedal", "coast_end", "REGEN_END_PERCENT", 0.005, 0.10, False),
        ("pedal", "drive_start", "APPS_INFLECTION_PERCENT", 0.02, 0.20, False),
        ("regen", "liftoff_nm", "REGEN_LIFTOFF_TORQUE", car["REGEN_MAX_TORQUE"], 0, False),
        ("regen", "fade_lo_kph", "REGEN_FADE_LO_KPH", 0, 40, False),
        ("regen", "fade_hi_kph", "REGEN_FADE_HI_KPH", 5, 60, False),
        ("regen", "ramp_nm_s", "REGEN_RAMP_NM_S", 5, 200, False),
        ("filter", "tau_s", "RC_TIME_CONSTANT", 0.005, 0.2, False),
        ("filter", "slew_nm_s", "SLEW_RATE_LIMIT", 200, 10000, False),
    ]


# ---------- YAML subset ----------

def _number(text, where):
    try:
        v = float(text)
    except ValueError:
        raise ProfileError(f'{where}: "{text}" isn\'t a number') from None
    if not math.isfinite(v):
        raise ProfileError(f"{where}: {text} isn't finite")
    return int(v) if v.is_integer() else v


def parse_yaml(text, where="profile"):
    p = {}
    sec = None
    for n, raw in enumerate(text.splitlines(), 1):
        line = re.sub(r"(^|\s)#.*$", "", raw).rstrip()
        if not line.strip():
            continue
        m = re.match(r"^(\s*)([a-z_][a-z0-9_]*):\s*(.*)$", line, re.I)
        if not m:
            raise ProfileError(f'{where}:{n}: can\'t read "{raw.strip()}"')
        indent, key, val = m.groups()
        at = f"{where}:{n}"
        if not indent:
            if key in p:
                raise ProfileError(f'{at}: "{key}" appears twice')
            if val == "":
                if key not in SECTIONS:
                    raise ProfileError(f'{at}: unknown section "{key}"')
                sec = p[key] = {}
            else:
                if key not in ("name", "extends"):
                    raise ProfileError(f'{at}: unknown key "{key}"')
                p[key] = val
                sec = None
        else:
            if sec is None:
                raise ProfileError(f'{at}: "{key}" isn\'t inside a section')
            if key in sec:
                raise ProfileError(f'{at}: "{key}" appears twice')
            if val.startswith("["):
                if not val.endswith("]"):
                    raise ProfileError(f"{at}: list not closed")
                inner = val[1:-1].strip()
                sec[key] = [_number(x.strip(), at) for x in inner.split(",")] if inner else []
            else:
                sec[key] = _number(val, at)
    return p


def to_yaml(p, car):
    """Inverse of parse_yaml, same layout as the editor's export."""
    cnames = {(s, k): c for s, k, c, *_ in fields(car)}
    lines = [f"name: {p['name']}"]
    if p.get("extends"):
        lines.append(f"extends: {p['extends']}")
    for sec in SECTIONS:
        if not p.get(sec):
            continue
        lines += ["", f"{sec}:"]
        for k, v in p[sec].items():
            if isinstance(v, list):
                val = "[" + ", ".join(_fmt(x) for x in v) + "]"
                comment = "torqueMap_table, fraction of max_nm"
            else:
                val, comment = _fmt(v), cnames.get((sec, k), "")
            lines.append(f"  {k}: {val}" + (f"   # {comment}" if comment else ""))
    return "\n".join(lines) + "\n"


def _fmt(v):
    return str(int(v)) if float(v).is_integer() else repr(float(v))


# ---------- validation ----------

def check_raw(p, car, where):
    """Shape checks on one file before merging: names, keys, types."""
    name = p.get("name")
    if not name:
        raise ProfileError(f"{where}: missing name:")
    if not NAME_RE.match(name):
        raise ProfileError(f"{where}: name must be lowercase letters, digits and "
                           "underscores, starting with a letter (31 characters max)")
    if name == "base":
        if "extends" in p:
            raise ProfileError(f"{where}: base can't extend another profile")
    elif p.get("extends") != "base":
        raise ProfileError(f"{where}: needs `extends: base` (profiles extend base only)")
    known = {(s, k): integer for s, k, _, _, _, integer in fields(car)}
    for sec in SECTIONS:
        for key, v in p.get(sec, {}).items():
            if (sec, key) == ("torque", "pedal_table"):
                if not isinstance(v, list):
                    raise ProfileError(f"{where}: torque.pedal_table must be a list")
            elif (sec, key) not in known:
                raise ProfileError(f'{where}: unknown key "{sec}.{key}"')
            elif isinstance(v, list):
                raise ProfileError(f"{where}: {sec}.{key} must be a number, not a list")
            elif known[(sec, key)] and not float(v).is_integer():
                raise ProfileError(f"{where}: {sec}.{key} must be a whole number")


def merge(base, p):
    out = {"name": p["name"]}
    for sec in SECTIONS:
        out[sec] = dict(base.get(sec, {}))
        out[sec].update(p.get(sec, {}))
    return out


def check_resolved(e, car, where):
    """Every tunable present and in range, plus the firmware's own checks."""
    errors = []
    for sec, key, cname, lo, hi, _ in fields(car):
        if key not in e[sec]:
            errors.append(f"{sec}.{key} ({cname}) is missing")
            continue
        v = e[sec][key]
        if not lo <= v <= hi:
            errors.append(f"{sec}.{key} = {v} is outside [{lo}, {hi}] ({cname})")
    table = e["torque"].get("pedal_table")
    points = car["TORQUE_MAP_POINTS"]
    if table is None:
        errors.append("torque.pedal_table is missing")
    elif len(table) != points:
        errors.append(f"torque.pedal_table needs {points} points, has {len(table)}")
    elif not table_is_valid(table):
        errors.append("torque.pedal_table must start at 0, end at 1, stay within "
                      "[0, 1] and never decrease (torqueMap_tableIsValid)")
    pd, rg = e["pedal"], e["regen"]
    if "coast_end" in pd and "drive_start" in pd and not pd["coast_end"] < pd["drive_start"]:
        errors.append("pedal.coast_end must be below pedal.drive_start")
    if "fade_lo_kph" in rg and "fade_hi_kph" in rg and not rg["fade_lo_kph"] < rg["fade_hi_kph"]:
        errors.append("regen.fade_lo_kph must be below regen.fade_hi_kph")
    if errors:
        raise ProfileError(f"{where}:\n  " + "\n  ".join(errors))


def table_is_valid(t):
    # Same rule as torqueMap_tableIsValid() in torque_map.c.
    if len(t) < 2 or t[0] != 0 or t[-1] != 1:
        return False
    return all(t[i - 1] <= t[i] <= 1 for i in range(1, len(t)))


def load(path, car, base=None):
    """Read, check and resolve one profile. base: the resolved base profile
    (loaded from the same folder when not given)."""
    path = Path(path)
    p = parse_yaml(path.read_text(), path.name)
    check_raw(p, car, path.name)
    if path.stem != p["name"]:
        raise ProfileError(f"{path.name}: file name must be {p['name']}.yaml")
    if p["name"] == "base":
        e = merge({}, p)
    else:
        if base is None:
            base = load(path.parent / "base.yaml", car)
        e = merge(base, p)
    check_resolved(e, car, path.name)
    e["extends"] = p.get("extends")
    e["overrides"] = {s: dict(p[s]) for s in SECTIONS if p.get(s)}
    return e


def load_all(folder, car, base_path=None):
    """Every profile in a folder, base first, then by name."""
    folder = Path(folder)
    base = load(Path(base_path) if base_path else folder / "base.yaml", car)
    out = {"base": base}
    for f in sorted(folder.glob("*.yaml")):
        if f.stem != "base":
            e = load(f, car, base)
            out[e["name"]] = e
    return out


def tunables(e):
    return {s: e[s] for s in SECTIONS}


def profile_hash(e):
    """Short hash of the resolved tunables (not the name): two profiles with
    the same numbers have the same hash. Carried to the car and the sim."""
    canon = json.dumps(tunables(e), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canon.encode()).hexdigest()[:8]


# ---------- outputs ----------

def _cfloat(v):
    s = repr(float(v))
    return s + "f"


def gen_header(e, car):
    h = profile_hash(e)
    origin = f" (extends {e['extends']})" if e.get("extends") else ""
    lines = [
        "// Generated by tools/gen_profile.py from profiles/{}.yaml. Do not edit.".format(e["name"]),
        f"// Profile: {e['name']}{origin}, hash {h}",
        "",
        "#ifndef PROFILE_GENERATED_H_",
        "#define PROFILE_GENERATED_H_",
        "",
        f'#define PROFILE_NAME "{e["name"]}"',
        f"#define PROFILE_HASH 0x{h}u",
        "",
    ]
    for sec, key, cname, _, _, integer in fields(car):
        v = e[sec][key]
        lines.append(f"#define {cname:<24} {int(v) if integer else _cfloat(v)}")
    lines += [
        "",
        "#define TORQUE_MAP_TABLE { \\",
        "\t" + ", ".join(_cfloat(v) for v in e["torque"]["pedal_table"]) + ", \\",
        "}",
        "",
        "#endif /* PROFILE_GENERATED_H_ */",
        "",
    ]
    return "\n".join(lines)


def _lua(v):
    if isinstance(v, dict):
        return "{ " + ", ".join(f"{k} = {_lua(v[k])}" for k in v) + " }"
    if isinstance(v, list):
        return "{ " + ", ".join(_lua(x) for x in v) + " }"
    if isinstance(v, str):
        return json.dumps(v)
    return _fmt(v)


def gen_lua(profiles, car, source=""):
    """Every profile as a Lua table, for vcu.lua in the sim repo."""
    car_lua = {
        "regen_max_nm": car["REGEN_MAX_TORQUE"],
        "regen_enabled": car["REGEN_ENABLED"],
        "regen_min_kph": car["REGEN_MIN_CARSPEED"],
        "regen_brake_cancel": car["REGEN_BRAKE_CANCEL_PERCENT"],
        "min_nm": car["MIN_TORQUE"],
        "gear_ratio": car["GEAR_RATIO"],
        "tire_diameter_in": car["TIRE_DIAMETER_IN"],
        "motor_forward_rpm_sign": car["MOTOR_FORWARD_RPM_SIGN"],
        "power_limit_w": car["POWER_LIMIT_W"],
        "drivetrain_efficiency": car["DRIVETRAIN_EFFICIENCY"],
        "motor_max_rpm": car["MOTOR_MAX_RPM"],
        "control_loop_hz": car["CONTROL_LOOP_HZ"],
        "torque_tx_divider": car["TORQUE_TX_DIVIDER"],
    }
    lines = [
        "-- Generated by mk12-vcu tools/gen_profile.py" + (f" at {source}" if source else "") + ". Do not edit.",
        "-- car: fixed values from the firmware headers. profiles: every approved",
        "-- profile, resolved against base. vcu.lua reads both and has no numbers of its own.",
        "return {",
        f"  car = {_lua(car_lua)},",
        "  order = " + _lua(list(profiles)) + ",",
        "  profiles = {",
    ]
    for name, e in profiles.items():
        lines.append(f"    {name} = {{")
        lines.append(f'      hash = "{profile_hash(e)}",')
        for sec in SECTIONS:
            lines.append(f"      {sec} = {_lua(e[sec])},")
        lines.append("    },")
    lines += ["  },", "}", ""]
    return "\n".join(lines)


def gen_index(profiles):
    lines = ["; Generated by mk12-vcu tools/gen_profile.py. Do not edit.", "[PROFILES]",
             f"COUNT={len(profiles)}"]
    for i, (name, e) in enumerate(profiles.items()):
        lines += ["", f"[PROFILE_{i}]", f"NAME={name}", f"HASH={profile_hash(e)}"]
    return "\n".join(lines) + "\n"


def target_nm(e, car, apps, bse, kph):
    """torqueMap_target in Python, for the report's plots only. The reference
    outputs (tests/reference) are the real C, not this."""
    t, pd, rg = e["torque"]["pedal_table"], e["pedal"], e["regen"]
    apps, bse = min(max(apps, 0.0), 1.0), min(max(bse, 0.0), 1.0)
    if apps >= pd["drive_start"]:
        pos = max((apps - pd["drive_start"]) / (1 - pd["drive_start"]) * (len(t) - 1), 0.0)
        i = int(pos)
        frac = t[-1] if i >= len(t) - 1 else t[i] + (t[i + 1] - t[i]) * (pos - i)
        return min(max(e["torque"]["max_nm"] * frac, 0.0), e["torque"]["max_nm"])
    if not car["REGEN_ENABLED"] or apps >= pd["coast_end"]:
        return 0.0
    if kph < car["REGEN_MIN_CARSPEED"] or bse > car["REGEN_BRAKE_CANCEL_PERCENT"]:
        return 0.0
    fade = min(max((kph - rg["fade_lo_kph"]) / (rg["fade_hi_kph"] - rg["fade_lo_kph"]), 0.0), 1.0)
    depth = (pd["coast_end"] - apps) / pd["coast_end"]
    return max(rg["liftoff_nm"] * depth * fade, car["REGEN_MAX_TORQUE"])


def gen_report(e, base, car):
    def curve(p):
        return [[round(a / 100, 4), round(target_nm(p, car, a / 100, 0, 60), 4)]
                for a in range(0, 101)]

    def regen(p):
        return [[k, round(target_nm(p, car, 0, 0, k), 4)] for k in range(0, 61, 2)]

    diff = {}
    for sec in SECTIONS:
        for k, v in e[sec].items():
            if base[sec].get(k) != v:
                diff[f"{sec}.{k}"] = {"base": base[sec].get(k), "profile": v}
    return {
        "name": e["name"],
        "hash": profile_hash(e),
        "extends": e.get("extends"),
        "tunables": tunables(e),
        "diff_vs_base": diff,
        "pedal_to_torque_at_60kph": curve(e),
        "liftoff_regen_vs_kph": regen(e),
        "note": "Curves are the map's target only: no filter, slew or power cap.",
    }


def gen_schema(car):
    """JSON Schema for profile files: autocomplete and red squiggles in
    editors. Cross-field rules (coast_end < drive_start, table shape) are
    only in this script."""
    secs = {}
    for sec, key, cname, lo, hi, integer in fields(car):
        s = secs.setdefault(sec, {"type": "object", "additionalProperties": False, "properties": {}})
        s["properties"][key] = {"type": "integer" if integer else "number",
                                "minimum": lo, "maximum": hi, "description": cname}
    secs["torque"]["properties"]["pedal_table"] = {
        "type": "array", "minItems": car["TORQUE_MAP_POINTS"], "maxItems": car["TORQUE_MAP_POINTS"],
        "items": {"type": "number", "minimum": 0, "maximum": 1},
        "description": "torqueMap_table: fraction of max_nm from drive_start to full pedal. "
                       "Starts at 0, ends at 1, never decreases."}
    return {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "$id": "https://github.com/bruinformula/mk12-vcu/profiles/schema.json",
        "title": "MK12 VCU driver profile",
        "description": "Generated by tools/gen_profile.py schema. Do not edit. base.yaml sets "
                       "every key; other profiles set `extends: base` and override a subset.",
        "type": "object",
        "additionalProperties": False,
        "required": ["name"],
        "properties": {
            "name": {"type": "string", "pattern": NAME_RE.pattern},
            "extends": {"const": "base"},
            **secs,
        },
    }


def schema_text(car):
    return json.dumps(gen_schema(car), indent=2) + "\n"


# ---------- CLI ----------

def _write(text, out):
    if out:
        Path(out).parent.mkdir(parents=True, exist_ok=True)
        Path(out).write_text(text)
    else:
        sys.stdout.write(text)


def _base(args, car):
    return load(args.base or Path(args.profiles) / "base.yaml", car)


def _find(name, car, args):
    path = Path(args.profiles) / f"{name}.yaml"
    if not path.exists():
        raise ProfileError(f"no profile {path}")
    return load(path, car, _base(args, car))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--profiles", default=PROFILES_DIR, help="profile folder (default profiles/)")
    ap.add_argument("--base", help="base.yaml other profiles extend (default: the one in --profiles)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="validate every profile and check schema.json is up to date")
    v = sub.add_parser("validate", help="validate profile files, e.g. sim drafts")
    v.add_argument("files", nargs="+")
    h = sub.add_parser("header", help="C header for one profile")
    h.add_argument("name")
    h.add_argument("-o", "--out")
    a = sub.add_parser("assetto", help="profiles.lua and profiles_index.ini for the sim")
    a.add_argument("dir")
    a.add_argument("--source", default="", help="source commit to note in profiles.lua")
    r = sub.add_parser("report", help="JSON summary of one profile")
    r.add_argument("name")
    r.add_argument("-o", "--out")
    s = sub.add_parser("schema", help="JSON Schema for profile files")
    s.add_argument("-o", "--out")
    args = ap.parse_args(argv)

    try:
        car = read_car()
        if args.cmd == "check":
            profiles = load_all(args.profiles, car, args.base)
            for name, e in profiles.items():
                print(f"ok  {name:<24} {profile_hash(e)}")
            check_base_matches_firmware(profiles["base"], car)
            print("ok  base.yaml matches the firmware's #defines and table")
            if SCHEMA_PATH.read_text() != schema_text(car):
                raise ProfileError("profiles/schema.json is out of date: "
                                   "run tools/gen_profile.py schema -o profiles/schema.json")
            print("ok  schema.json up to date")
        elif args.cmd == "validate":
            base = _base(args, car)
            for f in args.files:
                e = load(f, car, base)
                print(f"ok  {f} ({e['name']}, {profile_hash(e)})")
        elif args.cmd == "header":
            _write(gen_header(_find(args.name, car, args), car), args.out)
        elif args.cmd == "assetto":
            profiles = load_all(args.profiles, car, args.base)
            out = Path(args.dir)
            out.mkdir(parents=True, exist_ok=True)
            (out / "profiles.lua").write_text(gen_lua(profiles, car, args.source))
            (out / "profiles_index.ini").write_text(gen_index(profiles))
            print(f"wrote {out / 'profiles.lua'}, {out / 'profiles_index.ini'} ({len(profiles)} profiles)")
            sim_ratio = 43 / 11  # mk12-assetto-sim bfr_mk12/data/drivetrain.ini GEAR_1
            if not math.isclose(car["GEAR_RATIO"], sim_ratio, rel_tol=1e-3):
                print(f"warning: VCU GEAR_RATIO {car['GEAR_RATIO']:.3f} != sim drivetrain.ini "
                      f"{sim_ratio:.3f}. Settled at 3.91 on 2026-10-25: fix whichever moved.",
                      file=sys.stderr)
        elif args.cmd == "report":
            e = _find(args.name, car, args)
            base = _base(args, car)
            _write(json.dumps(gen_report(e, base, car), indent=2) + "\n", args.out)
        elif args.cmd == "schema":
            _write(schema_text(car), args.out)
    except ProfileError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
