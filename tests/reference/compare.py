#!/usr/bin/env python3
"""Compare a CSV against reference outputs, number by number.

  compare.py REFERENCE ACTUAL [--tol NM] [--columns a,b,...]

Rows must be in the same order. Every column both files have is compared
(or just --columns): numbers within --tol (absolute) plus a 1e-6 relative
part, or exactly with --tol 0; text exactly. A port such as vcu.lua writes
the same columns as reference.c for the rows it runs, so the sim repo's CI can
use this script as is. Standard library only.
"""

import argparse
import csv
import sys


def rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(line for line in f if not line.startswith("#")))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("reference")
    ap.add_argument("actual")
    ap.add_argument("--tol", type=float, default=1e-4, help="absolute tolerance (default 1e-4)")
    ap.add_argument("--columns", help="comma-separated columns to compare (default: all shared)")
    ap.add_argument("--show", type=int, default=10, help="mismatches to print (default 10)")
    args = ap.parse_args(argv)

    ref, actual = rows(args.reference), rows(args.actual)
    if len(ref) != len(actual):
        print(f"FAIL: {len(ref)} reference rows, {len(actual)} actual rows")
        return 1
    if not ref:
        print("FAIL: no rows")
        return 1
    shared = [c for c in ref[0] if c in actual[0]]
    cols = args.columns.split(",") if args.columns else shared
    missing = [c for c in cols if c not in shared]
    if missing:
        print(f"FAIL: columns missing from one file: {', '.join(missing)}")
        return 1

    bad, worst = 0, 0.0
    for i, (g, a) in enumerate(zip(ref, actual)):
        for c in cols:
            gv, av = g[c], a[c]
            if gv == av:
                continue
            try:
                gf, af = float(gv), float(av)
                diff = abs(gf - af)
                worst = max(worst, diff)
                if diff <= args.tol + (1e-6 * abs(gf) if args.tol else 0.0):
                    continue
            except ValueError:
                pass
            bad += 1
            if bad <= args.show:
                where = " ".join(f"{k}={g[k]}" for k in ("kind", "scenario", "tick", "apps", "bse",
                                                          "speed_kph", "motor_rpm") if g.get(k))
                print(f"  row {i + 1} [{where}] {c}: reference {gv}, actual {av}")
    print(f"{'FAIL' if bad else 'ok  '} {args.actual}: {len(ref)} rows, {len(cols)} columns, "
          f"{bad} mismatches, largest difference {worst:.3g}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
