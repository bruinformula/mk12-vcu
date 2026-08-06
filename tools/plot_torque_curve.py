"""
plot_torque_curve.py

Reads the CSV produced by torque_curve_sim.c and plots linear vs.
progressive torque curves with matplotlib.

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

    apps_percent, linear, progressive = [], [], []
    with open(sys.argv[1], newline="") as f:
        for row in csv.DictReader(f):
            apps_percent.append(float(row["apps_percent"]) * 100)
            linear.append(float(row["linear_torque"]))
            progressive.append(float(row["progressive_torque"]))

    plt.plot(apps_percent, linear, label="Linear (current)")
    plt.plot(apps_percent, progressive, label="Progressive (proposed)")
    plt.xlabel("Pedal position (%)")
    plt.ylabel("Requested torque (Nm)")
    plt.title("Pedal-to-torque mapping")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.show()


if __name__ == "__main__":
    main()
