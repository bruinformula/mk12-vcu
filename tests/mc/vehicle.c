/*
 * vehicle.c
 *
 * Car model for the Monte Carlo tests. See vehicle.h.
 */

#include "torque_map.h"
#include "vehicle.h"

#define GRAVITY 9.81f
#define AIR_DENSITY 1.2f       // kg/m^3
#define ROLLING_RESISTANCE 0.015f
#define DRIVETRAIN_EFFICIENCY 0.95f
#define IN_TO_M 0.0254f
#define PI 3.14159265f

#define COUNT(a) ((int)(sizeof(a) / sizeof((a)[0])))

// Start on and around the regen thresholds, so short runs reach them.
static const float SPEED_EDGES[] = {
	0.0f, REGEN_MIN_CARSPEED, REGEN_FADE_LO_KPH, REGEN_FADE_HI_KPH,
};

void vehicle_init(Vehicle *v, uint64_t seed) {
	Rng r = { seed };
	// Guesses with wide margins until the car is weighed and measured.
	v->mass_kg = rng_uniform(&r, 250.0f, 340.0f);
	// TIRE_DIAMETER_IN is the smallest plausible tire, so the real one is
	// the same or bigger (torque_map.h).
	v->tire_diameter_in = rng_uniform(&r, TIRE_DIAMETER_IN, TIRE_DIAMETER_IN + 3.0f);
	v->cda = rng_uniform(&r, 0.8f, 1.6f);
	v->brake_decel = rng_uniform(&r, 10.0f, 18.0f); // 1 to 1.8 g

	if (rng_chance(&r, 0.1f)) {
		v->speed_kph = rng_uniform(&r, -12.0f, 0.0f); // rolling back (pushed, or off a ramp)
	} else if (rng_chance(&r, 0.5f)) {
		v->speed_kph = rng_pick(&r, SPEED_EDGES, COUNT(SPEED_EDGES)) + rng_uniform(&r, -1.0f, 3.0f);
	} else {
		v->speed_kph = rng_uniform(&r, 0.0f, 110.0f);
	}
}

static float tire_radius_m(const Vehicle *v) {
	return v->tire_diameter_in * IN_TO_M / 2.0f;
}

void vehicle_step(Vehicle *v, float torque_nm, float bse, float dt) {
	float speed = v->speed_kph / 3.6f; // m/s
	float drive_force = torque_nm * GEAR_RATIO * DRIVETRAIN_EFFICIENCY / tire_radius_m(v);
	// Resisting forces only ever slow the car to a stop, never reverse it.
	float brake = fminf(fmaxf(bse, 0.0f), 1.0f) * v->brake_decel * v->mass_kg;
	float resist = brake + ROLLING_RESISTANCE * v->mass_kg * GRAVITY +
			0.5f * AIR_DENSITY * v->cda * speed * speed;

	// Regen pushes against the direction of travel. The firmware's speed
	// checks never allow it rolling back, but if they did, it'd still only
	// slow the car.
	float free = speed + drive_force / v->mass_kg * dt;
	float slow = resist / v->mass_kg * dt;
	if (fabsf(free) <= slow) {
		speed = 0.0f;
	} else {
		speed = free - copysignf(slow, free);
	}
	v->speed_kph = speed * 3.6f;
}

float vehicle_motor_rpm(const Vehicle *v) {
	float wheel_rpm = (v->speed_kph / 3.6f) / (PI * 2.0f * tire_radius_m(v)) * 60.0f;
	return wheel_rpm * GEAR_RATIO * MOTOR_FORWARD_RPM_SIGN;
}
