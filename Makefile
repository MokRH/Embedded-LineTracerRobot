# Host-side tests: no hardware and no AVR toolchain needed (g++ and make only).
#   make test      core unit tests + hardware-layer tests for every robot + compile matrix
#   make core      only the hardware-independent unit tests (sanitizers on)
CXX      ?= g++
CXXFLAGS ?= -std=gnu++11 -Wall -Wextra -Wshadow -Werror -g
SAN      ?= -fsanitize=address,undefined -fno-sanitize-recover=undefined
BUILD    := build

ROBOTS := HammerHead_5 HammerHead_11 Comel2 Comel3

.PHONY: test core hal matrix diff clean
test: core hal matrix diff

core: $(BUILD)/test_core
	./$(BUILD)/test_core

$(BUILD)/test_core: test/test_core.cpp src/KilatCore.cpp src/KilatCore.h test/minitest.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SAN) -o $@ test/test_core.cpp src/KilatCore.cpp

hal: $(ROBOTS:%=$(BUILD)/test_hal_%)
	@set -e; for r in $(ROBOTS); do echo "== hardware layer: $$r"; ./$(BUILD)/test_hal_$$r; done

$(BUILD)/test_hal_%: test/test_hal.cpp src/KilatTracer.cpp src/KilatCore.cpp test/stubs/arduino_stub.cpp test/minitest.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SAN) -D$* -Itest/stubs -Isrc -o $@ test/test_hal.cpp src/KilatTracer.cpp src/KilatCore.cpp test/stubs/arduino_stub.cpp

matrix:
	./test/compile_matrix.sh

clean:
	rm -rf $(BUILD)

# Differential test: the ORIGINAL library (test/golden/legacy) and the new one are
# run through the same random scenarios; their traces must be identical.
DIFF_N ?= 1500
diff:
	@mkdir -p $(BUILD)
	@set -e; for r in $(ROBOTS); do \
	  $(CXX) -std=gnu++11 -w -g -D$$r -Itest/golden/legacy -Itest/stubs -o $(BUILD)/trace_legacy_$$r \
	    test/diff/trace_runner.cpp test/golden/legacy/KilatTracer.cpp test/stubs/arduino_stub.cpp; \
	  $(CXX) $(CXXFLAGS) $(SAN) -D$$r -Isrc -Itest/stubs -o $(BUILD)/trace_new_$$r \
	    test/diff/trace_runner.cpp src/KilatTracer.cpp src/KilatCore.cpp test/stubs/arduino_stub.cpp; \
	  ./$(BUILD)/trace_legacy_$$r $(DIFF_N) > $(BUILD)/trace_legacy_$$r.txt; \
	  ./$(BUILD)/trace_new_$$r    $(DIFF_N) > $(BUILD)/trace_new_$$r.txt; \
	  if cmp -s $(BUILD)/trace_legacy_$$r.txt $(BUILD)/trace_new_$$r.txt; then \
	    echo "diff $$r: identical ($$(wc -l < $(BUILD)/trace_new_$$r.txt) traces)"; \
	  else echo "diff $$r: DIFFERENT"; diff $(BUILD)/trace_legacy_$$r.txt $(BUILD)/trace_new_$$r.txt | head -10; exit 1; fi; \
	done
