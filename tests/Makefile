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
	$(BUILD)/test_traction_control

.PHONY: test pbt profiles reference-update clean
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

# Driver profiles and reference outputs (docs/profiles.md).
# Needs python3 only (standard library). Not part of `make test`.
#   make -C tests profiles        validate profiles/, generator tests, base.yaml
#                                 matches the firmware, reference outputs up to date
#   make -C tests reference-update   rewrite reference/outputs/ after a deliberate
#                                 change to the pipeline; review the diff
# Outputs cover base plus the test fixtures in reference/fixtures/, which move
# every tunable. reference.c builds the real torque_map.c and power_limit.c with
# each profile's values; reference_firmware builds them untouched.
GEN = ../tools/gen_profile.py
REF_BUILD = $(BUILD)/reference
REF_PROFILES = base $(basename $(notdir $(wildcard reference/fixtures/*.yaml)))
REF_CSVS = $(REF_PROFILES:%=$(REF_BUILD)/%.csv)
# No fused multiply-add, so x86 and ARM laptops print the same outputs.
REF_CFLAGS = $(CFLAGS) -ffp-contract=off
REF_DEPS = reference/reference.c ../Core/Src/torque_map.c ../Core/Src/power_limit.c \
	../Core/Inc/torque_map.h ../Core/Inc/power_limit.h

profiles: $(REF_CSVS) $(REF_BUILD)/firmware.csv
	$(PYTHON) $(GEN) check
	$(PYTHON) $(GEN) --base ../profiles/base.yaml --profiles reference/fixtures validate reference/fixtures/*.yaml
	$(PYTHON) test_gen_profile.py
	@echo "== base.yaml matches the firmware"
	$(PYTHON) reference/compare.py --tol 0 $(REF_BUILD)/firmware.csv $(REF_BUILD)/base.csv
	@echo "== reference outputs up to date (else: make -C tests reference-update)"
	@for p in $(REF_PROFILES); do $(PYTHON) reference/compare.py reference/outputs/$$p.csv $(REF_BUILD)/$$p.csv || exit 1; done

reference-update: $(REF_CSVS)
	@mkdir -p reference/outputs
	@for p in $(REF_PROFILES); do cp $(REF_BUILD)/$$p.csv reference/outputs/$$p.csv; echo "wrote reference/outputs/$$p.csv"; done

$(REF_BUILD)/%.csv: $(REF_BUILD)/reference_%
	./$< > $@

$(REF_BUILD)/reference_firmware: $(REF_DEPS)
	@mkdir -p $(REF_BUILD)
	$(CC) $(REF_CFLAGS) reference/reference.c $(LDLIBS) -o $@

$(REF_BUILD)/reference_%: $(REF_BUILD)/%/profile_generated.h $(REF_DEPS)
	$(CC) $(REF_CFLAGS) -DPROFILE_HEADER='"profile_generated.h"' -I$(REF_BUILD)/$* reference/reference.c $(LDLIBS) -o $@

$(REF_BUILD)/%/profile_generated.h: $(GEN) ../profiles/base.yaml $(wildcard reference/fixtures/*.yaml) ../Core/Inc/torque_map.h ../Core/Inc/power_limit.h
	@mkdir -p $(dir $@)
	$(PYTHON) $(GEN) --base ../profiles/base.yaml --profiles $(if $(filter base,$*),../profiles,reference/fixtures) header $* -o $@

.PRECIOUS: $(REF_BUILD)/reference_% $(REF_BUILD)/%/profile_generated.h

clean:
	rm -rf $(BUILD)

# Simulink TC model (Embedded Coder output, unmodified) in shadow mode.
TC_SRC = test_traction_control.c ../Core/Src/traction_control.c \
	../Core/Src/traction_model/Block_Diagram_1.c ../Core/Src/traction_model/Block_Diagram_1_data.c
$(BUILD)/test_traction_control: $(TC_SRC) check.h ../Core/Inc/traction_control.h $(wildcard ../Core/Src/traction_model/*.h)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(TC_SRC) $(LDLIBS) -o $@
