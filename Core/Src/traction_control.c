/*
 * traction_control.c
 *
 * See traction_control.h. Wires Block_Diagram_1 v2.0 (1 kHz, 5 named inports:
 * wheel_fl_rads, wheel_fr_rads, wheel_rl_rads, wheel_rr_rads, driver_nm) and
 * reads the launch-blended output Torque_CommandNm.
 */

#include "traction_control.h"
#include "traction_model/Block_Diagram_1.h"

void tractionControl_init(void) {
	Block_Diagram_1_initialize();
}

float tractionControl_applyWheels(float driver_nm,
		float wheel_fl_rads, float wheel_fr_rads,
		float wheel_rl_rads, float wheel_rr_rads) {
	Block_Diagram_1_U.wheel_fl_rads = (double)wheel_fl_rads;
	Block_Diagram_1_U.wheel_fr_rads = (double)wheel_fr_rads;
	Block_Diagram_1_U.wheel_rl_rads = (double)wheel_rl_rads;
	Block_Diagram_1_U.wheel_rr_rads = (double)wheel_rr_rads;
	Block_Diagram_1_U.driver_nm = (driver_nm > 0.0f) ? (double)driver_nm : 0.0;
	Block_Diagram_1_step();

#if TC_ENABLED
	// Only limit positive drive torque; never alter coast or regen.
	if (driver_nm <= 0.0f) {
		return driver_nm;
	}
	float tc_nm = (float)Block_Diagram_1_Y.Torque_CommandNm;
	// EV.3.1.3 guard: TC may only reduce driver demand, never add torque or output NaN.
	if (!(tc_nm >= 0.0f)) {
		return 0.0f;
	}
	if (tc_nm > driver_nm) {
		return driver_nm;
	}
	return tc_nm;
#else
	// Shadow mode: the model runs, its output is not used.
	return driver_nm;
#endif
}

float tractionControl_apply(float driver_nm) {
	return tractionControl_applyWheels(driver_nm, 0.0f, 0.0f, 0.0f, 0.0f);
}

float tractionControl_modelTorqueNm(void) {
	return (float)Block_Diagram_1_Y.Torque_CommandNm;
}

float tractionControl_slipRatio(void) {
	return (float)Block_Diagram_1_Y.CurrentSlipRatio;
}

float tractionControl_slipSpeedMps(void) {
	return (float)Block_Diagram_1_Y.CurrentSlipSpeed;
}
