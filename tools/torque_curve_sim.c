/*
 * torque_curve_sim.c
 *
 * Standalone, hardware-independent sweep of the pedal-to-torque curve.
 * Compiles and runs natively on a laptop (no STM32/HAL dependency) so the
 * curve shape can be tuned and inspected before it ever touches real
 * hardware.
 *
 * Build:  clang -O2 -lm tools/torque_curve_sim.c -o torque_curve_sim
 * Run:    ./torque_curve_sim > sweep.csv
 */

#include <stdio.h>
#include <math.h>

// Mirrors motor_control.h constants. Keep in sync manually for now.
#define APPS_INFLECTION_PERCENT 0.05f
#define MAX_TORQUE 80.0f
#define MIN_TORQUE 0.0f

// Tunable: >1 = progressive (soft near tip-in, aggressive near full pedal)
//          <1 = aggressive near tip-in, soft near full pedal
//          =1 = identical to today's linear curve
#define TORQUE_CURVE_EXPONENT 1.8f

static float clamp01(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

// Today's existing linear map (motor_control.c calculateTorqueRequest, positive-torque branch)
float linearTorque(float apps_percent) {
    float normalized = clamp01((apps_percent - APPS_INFLECTION_PERCENT) / (1.0f - APPS_INFLECTION_PERCENT));
    return MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) * normalized;
}

// Proposed progressive (power/gamma) curve
float progressiveTorque(float apps_percent, float exponent) {
    float normalized = clamp01((apps_percent - APPS_INFLECTION_PERCENT) / (1.0f - APPS_INFLECTION_PERCENT));
    float shaped = powf(normalized, exponent);
    return MIN_TORQUE + (MAX_TORQUE - MIN_TORQUE) * shaped;
}

int main(void) {
    printf("apps_percent,linear_torque,progressive_torque\n");
    for (int i = 0; i <= 100; i++) {
        float apps = i / 100.0f;
        printf("%.2f,%.2f,%.2f\n",
               apps,
               linearTorque(apps),
               progressiveTorque(apps, TORQUE_CURVE_EXPONENT));
    }
    return 0;
}
