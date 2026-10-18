# Host unit tests. Run from the repo root:  make -C tests
# Builds with the laptop's compiler; nothing here goes into the firmware
# (CubeIDE only compiles Core/ and Drivers/).

CC ?= cc
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../Core/Inc
LDLIBS = -lm

BUILD = build
TESTS = $(BUILD)/test_torque_map

.PHONY: test clean
test: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done

$(BUILD)/test_torque_map: test_torque_map.c check.h ../Core/Src/torque_map.c ../Core/Inc/torque_map.h
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) test_torque_map.c ../Core/Src/torque_map.c $(LDLIBS) -o $@

clean:
	rm -rf $(BUILD)
