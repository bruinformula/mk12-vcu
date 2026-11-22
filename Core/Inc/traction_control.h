/*
 * traction_control.h
 *
 * Wrapper around the Simulink traction control model (Block_Diagram_1 v2.0,
 * 1 kHz Embedded Coder output in Core/Src/traction_model/, copied unmodified
 * so a new export can be dropped over it). No HAL, CAN, or global state of ours.
 *
 * Uses the launch-blended controller (Torque_CommandNm): slip-speed control
 * below 1 m/s front wheel speed, linear transition from 1 to 2 m/s, and
 * slip-ratio control above 2 m/s (target 0.136).
 *
 * TC_ENABLED defaults to 0 on STM32 firmware until wheel speeds arrive on
 * CAN2; host tests and the Assetto Corsa SIL build compile with -DTC_ENABLED=1.
 */

#ifndef INC_TRACTION_CONTROL_H_
#define INC_TRACTION_CONTROL_H_

#ifndef TC_ENABLED
#define TC_ENABLED 0
#endif

// Call once at boot, before the control loop starts.
void tractionControl_init(void);

// One control loop tick with real wheel angular speeds (rad/s: FL, FR, RL, RR).
// With TC_ENABLED 1, returns the TC-limited torque in [0, driver_nm] for
// positive drive demand (regen / coast passes through unchanged).
// With TC_ENABLED 0 (shadow mode), steps the model and returns driver_nm.
float tractionControl_applyWheels(float driver_nm,
		float wheel_fl_rads, float wheel_fr_rads,
		float wheel_rl_rads, float wheel_rr_rads);

// Compatibility entry point for motor_control.c before CAN2 wheel speeds exist
// (feeds 0 rad/s wheels).
float tractionControl_apply(float driver_nm);

// Last model outputs, for Live Expressions, telemetry, and tests.
float tractionControl_modelTorqueNm(void);
float tractionControl_slipRatio(void);
float tractionControl_slipSpeedMps(void);

#endif /* INC_TRACTION_CONTROL_H_ */
