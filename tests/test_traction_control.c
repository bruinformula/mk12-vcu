// Host tests for the Simulink TC model wrapper (shadow mode).
#include "check.h"
#include "traction_control.h"

static void test_shadow_mode(void) {
	tractionControl_init();

	// Shadow mode never changes the command, whatever the driver asks for.
	const float demands[] = { 0.0f, 1.0f, 40.0f, 80.0f, 231.0f, -20.0f };
	for (unsigned i = 0; i < sizeof demands / sizeof demands[0]; i++) {
		CHECK(tractionControl_apply(demands[i]) == demands[i]);
	}

	// Pins model v1.17's '0-1 Clamp' bug (both limits 1): output is always
	// 0 Nm. When a fixed export lands, this check should fail; update it.
	CHECK(tractionControl_modelTorqueNm() == 0.0f);
}

int main(void) {
	RUN(test_shadow_mode);
	return CHECK_SUMMARY();
}
