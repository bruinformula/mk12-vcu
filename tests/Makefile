# Host unit tests. Run from the repo root:  make -C tests
# Builds with the laptop's compiler; nothing here goes into the firmware
# (CubeIDE only compiles Core/ and Drivers/).

CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../Core/Inc
LDLIBS = -lm

BUILD = build
TESTS = $(BUILD)/test_torque_map $(BUILD)/test_torque_map_regen $(BUILD)/test_pedal_checks $(BUILD)/test_pedal_checks_min_apps $(BUILD)/test_torque_diag \
	$(BUILD)/test_power_limit $(BUILD)/test_power_limit_230 \
	$(BUILD)/test_exhaustive $(BUILD)/test_exhaustive_regen $(BUILD)/test_exhaustive_min_apps \
	$(BUILD)/test_mc $(BUILD)/test_mc_regen $(BUILD)/test_mc_min_apps

.PHONY: test pbt clean
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

PL_SRC = test_power_limit.c ../Core/Src/power_limit.c ../Core/Src/torque_map.c
PL_DEPS = $(PL_SRC) check.h ../Core/Inc/power_limit.h ../Core/Inc/torque_map.h

$(BUILD)/test_power_limit: $(PL_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(PL_SRC) $(LDLIBS) -o $@

# Same tests at the inverter's 231 Nm torque limit, where the cap binds.
$(BUILD)/test_power_limit_230: $(PL_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DMAX_TORQUE=230 $(PL_SRC) $(LDLIBS) -o $@

# Exhaustive and grid sweeps (test_exhaustive.c): every ADC count and APPS
# pair, every 0x0A5 rpm, float grids, bounded-exhaustive fault timing.
EX_SRC = test_exhaustive.c ../Core/Src/pedal_checks.c ../Core/Src/torque_map.c ../Core/Src/power_limit.c
EX_DEPS = $(EX_SRC) check.h ../Core/Inc/pedal_checks.h ../Core/Inc/torque_map.h ../Core/Inc/power_limit.h

$(BUILD)/test_exhaustive: $(EX_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=0 $(EX_SRC) $(LDLIBS) -o $@

$(BUILD)/test_exhaustive_regen: $(EX_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=1 $(EX_SRC) $(LDLIBS) -o $@

$(BUILD)/test_exhaustive_min_apps: $(EX_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -DREGEN_ENABLED=1 -DPEDAL_MODE=MIN_APPS $(EX_SRC) $(LDLIBS) -o $@

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

# Property-based tests (pbt/test_properties.py, Hypothesis): the real files
# built into a shared library, driven from Python. Not part of `make test`
# because it needs Python and installs Hypothesis into $(BUILD)/venv.
#   make -C tests pbt                      500 examples per property
#   make -C tests pbt PBT_EXAMPLES=5000    longer search
PYTHON ?= python3
VENV = $(BUILD)/venv
PBT_SRC = pbt/shim.c ../Core/Src/pedal_checks.c ../Core/Src/torque_map.c \
	../Core/Src/power_limit.c ../Core/Src/torque_diag.c
PBT_DEPS = $(PBT_SRC) ../Core/Inc/pedal_checks.h ../Core/Inc/torque_map.h \
	../Core/Inc/power_limit.h ../Core/Inc/torque_diag.h
PBT_LIBS = $(BUILD)/libvcu_pbt.so $(BUILD)/libvcu_pbt_regen.so $(BUILD)/libvcu_pbt_min_apps.so

pbt: $(PBT_LIBS) $(VENV)/.installed
	@for l in $(PBT_LIBS); do echo "== $$l"; $(VENV)/bin/python pbt/test_properties.py $$(basename $$l) || exit 1; done

$(VENV)/.installed: pbt/requirements.txt
	$(PYTHON) -m venv $(VENV)
	$(VENV)/bin/pip install -q -r pbt/requirements.txt
	@touch $@

$(BUILD)/libvcu_pbt.so: $(PBT_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared -DREGEN_ENABLED=0 $(PBT_SRC) $(LDLIBS) -o $@

$(BUILD)/libvcu_pbt_regen.so: $(PBT_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared -DREGEN_ENABLED=1 $(PBT_SRC) $(LDLIBS) -o $@

$(BUILD)/libvcu_pbt_min_apps.so: $(PBT_DEPS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -fPIC -shared -DREGEN_ENABLED=1 -DPEDAL_MODE=MIN_APPS $(PBT_SRC) $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD)
