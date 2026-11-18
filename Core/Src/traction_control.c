/*
 * traction_control.c
 *
 * See traction_control.h. The model's inports are unnamed in v1.17:
 * In1 and In3 are compared against the 0.136 slip ratio target, In4 against
 * the slip speed target. They are fed 0 until wheel speeds exist.
 */

#include "traction_control.h"
#include "traction_model/Block_Diagram_1.h"

#if TC_ENABLED
#error "Traction control can't be enabled yet: no wheel speed inputs and model v1.17 always outputs 0 Nm. See traction_control.h."
#endif

void tractionControl_init(void) {
	Block_Diagram_1_initialize();
}

float tractionControl_apply(float driver_nm) {
	Block_Diagram_1_U.In1 = 0.0;
	Block_Diagram_1_U.In3 = 0.0;
	Block_Diagram_1_U.In4 = 0.0;
	Block_Diagram_1_step();

	// Shadow mode: the model runs, its output is not used.
	return driver_nm;
}

float tractionControl_modelTorqueNm(void) {
	return (float)Block_Diagram_1_Y.Torque_Command;
}
