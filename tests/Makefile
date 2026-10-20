# Host unit tests. Run from the repo root:  make -C tests
# Builds with the laptop's compiler; nothing here goes into the firmware
# (CubeIDE only compiles Core/ and Drivers/).

CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../Core/Inc
LDLIBS = -lm

BUILD = build
TESTS = $(BUILD)/test_torque_map $(BUILD)/test_torque_map_regen $(BUILD)/test_pedal_checks $(BUILD)/test_pedal_checks_min_apps $(BUILD)/test_torque_diag \
	$(BUILD)/test_mc $(BUILD)/test_mc_regen $(BUILD)/test_mc_min_apps

.PHONY: test clean
test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

$(BUILD)/test_torque_map: test_torque_map.c check.h ../Core/Src/torque_map.c ../Core/Inc/torque_map.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=0 test_torque_map.c ../Core/Src/torque_map.c $(LDLIBS) -o $@

# Same tests with regen compiled in. Both builds set REGEN_ENABLED explicitly,
# so they test the same thing on feature/torque-mapping (ships 0) and
# feature/regen (ships 1).
$(BUILD)/test_torque_map_regen: test_torque_map.c check.h ../Core/Src/torque_map.c ../Core/Inc/torque_map.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=1 test_torque_map.c ../Core/Src/torque_map.c $(LDLIBS) -o $@

$(BUILD)/test_pedal_checks: test_pedal_checks.c check.h ../Core/Src/pedal_checks.c ../Core/Inc/pedal_checks.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_pedal_checks.c ../Core/Src/pedal_checks.c $(LDLIBS) -o $@

# Same tests with the MIN_APPS pedal mode, ready but not switched on.
$(BUILD)/test_pedal_checks_min_apps: test_pedal_checks.c check.h ../Core/Src/pedal_checks.c ../Core/Inc/pedal_checks.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DPEDAL_MODE=MIN_APPS test_pedal_checks.c ../Core/Src/pedal_checks.c $(LDLIBS) -o $@

$(BUILD)/test_torque_diag: test_torque_diag.c check.h ../Core/Src/torque_diag.c ../Core/Inc/torque_diag.h ../Core/Src/pedal_checks.c ../Core/Inc/pedal_checks.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_torque_diag.c ../Core/Src/torque_diag.c ../Core/Src/pedal_checks.c $(LDLIBS) -o $@

# Monte Carlo (mc/test_mc.c): random drives through the real pedal checks
# and torque map, every tick checked against fixed rules. Built with regen
# off, on, and on with the MIN_APPS pedal mode.
MC_SRC = mc/test_mc.c mc/scenario.c mc/sensors.c mc/vehicle.c mc/inverter_can.c mc/oracle.c \
	../Core/Src/torque_map.c ../Core/Src/pedal_checks.c
MC_DEPS = $(MC_SRC) mc/rng.h mc/scenario.h mc/sensors.h mc/vehicle.h mc/inverter_can.h mc/oracle.h \
	../Core/Inc/torque_map.h ../Core/Inc/pedal_checks.h

$(BUILD)/test_mc: $(MC_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=0 $(MC_SRC) $(LDLIBS) -o $@

$(BUILD)/test_mc_regen: $(MC_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=1 $(MC_SRC) $(LDLIBS) -o $@

$(BUILD)/test_mc_min_apps: $(MC_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=1 -DPEDAL_MODE=MIN_APPS $(MC_SRC) $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD)
