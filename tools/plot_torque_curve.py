"""
plot_torque_curve.py

Reads the CSV produced by torque_curve_sim.c and plots the firmware's
pedal-to-torque map, stopped and moving (regen region) with matplotlib.

Usage:
    ./torque_curve_sim > sweep.csv
    python3 tools/plot_torque_curve.py sweep.csv
"""

import csv
import sys

import matplotlib.pyplot as plt


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 plot_torque_curve.py <sweep.csv>")
        sys.exit(1)

    apps_percent, stopped, moving = [], [], []
    with open(sys.argv[1], newline="") as f:
        for row in csv.DictReader(f):
            apps_percent.append(float(row["apps_percent"]) * 100)
            stopped.append(float(row["stopped_torque"]))
            moving.append(float(row["moving_torque"]))

    plt.plot(apps_percent, moving, label="Moving (regen below inflection)")
    plt.plot(apps_percent, stopped, "--", label="Stopped (no regen)")
    plt.axhline(0, color="gray", linewidth=0.8)
    plt.xlabel("Pedal position (%)")
    plt.ylabel("Requested torque (Nm)")
    plt.title("Pedal-to-torque mapping")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
