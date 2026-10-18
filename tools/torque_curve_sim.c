/*
 * torque_curve_sim.c
 *
 * Standalone, hardware-independent sweep of the pedal-to-torque map.
 * Compiles and runs natively on a laptop (no STM32/HAL dependency) so a
 * retuned torqueMap_table can be inspected before it touches real hardware.
 *
 * Both columns come from the real firmware map in Core/Src/torque_map.c,
 * so they always match what the car runs.
 *
 * Build:  clang -O2 -ICore/Inc tools/torque_curve_sim.c Core/Src/torque_map.c -lm -o torque_curve_sim
 * Run:    ./torque_curve_sim > sweep.csv
 * Plot:   python3 tools/plot_torque_curve.py sweep.csv
 */

#include <stdio.h>
#include "torque_map.h"

#define SIM_CARSPEED 10.0f // above REGEN_MIN_CARSPEED, so the regen region shows (if REGEN_ENABLED)

int main(void) {
    if (!torqueMap_tableIsValid(torqueMap_table, TORQUE_MAP_POINTS)) {
        fprintf(stderr, "torqueMap_table is invalid: the VCU would halt at boot\n");
        return 1;
    }

    printf("apps_percent,stopped_torque,moving_torque\n");
    for (int i = 0; i <= 100; i++) {
        float apps = i / 100.0f;
        printf("%.2f,%.2f,%.2f\n",
               apps,
               torqueMap_target(apps, 0.0f, 0.0f),           // no brake, stationary: no regen
               torqueMap_target(apps, 0.0f, SIM_CARSPEED));  // no brake, moving: regen below inflection
    }
    return 0;
}
