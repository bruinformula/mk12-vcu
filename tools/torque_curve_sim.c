/*
 * torque_curve_sim.c
 *
 * Standalone, hardware-independent sweep of the pedal-to-torque curve.
 * Compiles and runs natively on a laptop (no STM32/HAL dependency) so the
 * curve shape can be tuned and inspected before it ever touches real
 * hardware.
 *
 * The progressive column comes from the real firmware map in
 * Core/Src/torque_map.c, so it always matches what the car runs.
 *
 * Build:  clang -O2 -ICore/Inc tools/torque_curve_sim.c Core/Src/torque_map.c -lm -o torque_curve_sim
 * Run:    ./torque_curve_sim > sweep.csv
 */

#include <stdio.h>
#include "torque_map.h"

static float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Old linear map, kept here only as a comparison column
float linearTorque(float apps_percent) {
    float normalized = clamp01((apps_percent - APPS_INFLECTION_PERCENT) / (1.0f - APPS_INFLECTION_PERCENT));
    return MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) * normalized;
}

int main(void) {
    printf("apps_percent,linear_torque,progressive_torque\n");
    for (int i = 0; i <= 100; i++) {
        float apps = i / 100.0f;
        printf("%.2f,%.2f,%.2f\n",
               apps,
               linearTorque(apps),
               torqueMap_target(apps, 0.0f, 0.0f)); // no brake, stationary: no regen
    }
    return 0;
}
