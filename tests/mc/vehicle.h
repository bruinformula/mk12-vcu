/*
 * vehicle.h
 *
 * Point-mass longitudinal car model for the Monte Carlo tests: the true car
 * speed, driven by the torque the VCU commands, the brake, drag and rolling
 * resistance. Closed loop: regen slows the car, which changes the speed the
 * next 0x0A5 reports.
 *
 * Mass, tire size, drag and brake strength are random per run: they aren't
 * known exactly, so every run uses different ones. No wheel slip (the motor
 * speed is the wheel speed), no hills.
 */

#ifndef TESTS_MC_VEHICLE_H_
#define TESTS_MC_VEHICLE_H_

#include <stdint.h>
#include "rng.h"

typedef struct Vehicle {
	float mass_kg;          // car + driver
	float tire_diameter_in; // the real tire, not TIRE_DIAMETER_IN
	float cda;              // drag area, m^2
	float brake_decel;      // m/s^2 at full brake
	float speed_kph;        // truth, negative rolling back
} Vehicle;

// Random parameters and a random starting speed, around the regen thresholds
// a good share of the time.
void vehicle_init(Vehicle *v, uint64_t seed);

// Advance by dt with motor torque (Nm, negative = regen) and brake travel.
void vehicle_step(Vehicle *v, float torque_nm, float bse, float dt);

// What the inverter's resolver measures: motor rpm, forward = MOTOR_FORWARD_RPM_SIGN.
float vehicle_motor_rpm(const Vehicle *v);

#endif /* TESTS_MC_VEHICLE_H_ */
