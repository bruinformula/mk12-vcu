/*
 * traction_control.h
 *
 * Wrapper around the Simulink traction control model (Block_Diagram_1,
 * Embedded Coder output in Core/Src/traction_model/, copied unmodified so a
 * new export can be dropped over it). No HAL, CAN, or global state of ours.
 *
 * Today the model only runs in shadow: it is stepped every drive tick so it
 * builds, links and runs on the car, but its output never reaches the
 * inverter. TC_ENABLED 1 is blocked at compile time until:
 *   - wheel speeds + ground speed reach the VCU (DAQ/MDU frames on CAN2),
 *     so the slip inputs are real instead of 0
 *   - the model's '0-1 Clamp' lower limit is fixed (both limits are 1 in
 *     model v1.17, so Torque_Command is always 0 Nm)
 *   - the slip estimation uses inports instead of Constant blocks, driver
 *     demand comes from an inport instead of a 220 Nm constant, and the
 *     model is regenerated at 0.001 s (it was built at 0.2 s)
 */

#ifndef INC_TRACTION_CONTROL_H_
#define INC_TRACTION_CONTROL_H_

#ifndef TC_ENABLED
#define TC_ENABLED 0
#endif

// Call once at boot, before the control loop starts.
void tractionControl_init(void);

// One control loop tick. Steps the model and returns the torque to command.
// With TC_ENABLED 0 this is always driver_nm, unchanged.
float tractionControl_apply(float driver_nm);

// Last model outputs, for Live Expressions / tests (shadow mode).
float tractionControl_modelTorqueNm(void);

#endif /* INC_TRACTION_CONTROL_H_ */
