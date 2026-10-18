# Host unit tests. Run from the repo root:  make -C tests
# Builds with the laptop's compiler; nothing here goes into the firmware
# (CubeIDE only compiles Core/ and Drivers/).

CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../Core/Inc
LDLIBS = -lm

BUILD = build
TESTS = $(BUILD)/test_torque_map $(BUILD)/test_pedal_checks $(BUILD)/test_torque_diag

.PHONY: test clean
test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

$(BUILD)/test_torque_map: test_torque_map.c check.h ../Core/Src/torque_map.c ../Core/Inc/torque_map.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_torque_map.c ../Core/Src/torque_map.c $(LDLIBS) -o $@

$(BUILD)/test_pedal_checks: test_pedal_checks.c check.h ../Core/Src/pedal_checks.c ../Core/Inc/pedal_checks.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_pedal_checks.c ../Core/Src/pedal_checks.c $(LDLIBS) -o $@

$(BUILD)/test_torque_diag: test_torque_diag.c check.h ../Core/Src/torque_diag.c ../Core/Inc/torque_diag.h ../Core/Src/pedal_checks.c ../Core/Inc/pedal_checks.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_torque_diag.c ../Core/Src/torque_diag.c ../Core/Src/pedal_checks.c $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD)
