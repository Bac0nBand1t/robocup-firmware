.PHONY : all control kicker fpga

CURRENT_DIR=$(shell pwd)

SHARED_DIR="$(CURRENT_DIR)/shared"
CONVERT_SCRIPT="$(CURRENT_DIR)/convert.py"
FLASH_COPY_SCRIPT="$(CURRENT_DIR)/util/flash-mtrain.py"

all: control

control-upload: control

control: fpga kicker
	cd control && \
mkdir -p build && cd build && \
cmake -DSHARED_DIR=${SHARED_DIR} -DFLASH_COPY_SCRIPT=${FLASH_COPY_SCRIPT} .. && make

fpga:
	cd fpga && \
mkdir -p build && cd build && \
cmake -DSHARED_DIR=${SHARED_DIR} .. && make

kicker:
	cd kicker && \
mkdir -p build && cd build && \
cmake -DSHARED_DIR=${SHARED_DIR} -DCONVERT_SCRIPT=${CONVERT_SCRIPT} .. && make

docs:
	cd doc && doxygen Doxyfile
	@echo "\n=> Open up 'generated-docs/index.html' in a browser to view a local copy of the documentation"

clean:
	cd control && rm -r build
	cd fpga && rm -r build
	cd kicker && rm -r build


# .PHONY : all kicker configure robot control-upload docs $(ROBOT_TESTS:%=test-%-upload)

# all: kicker robot

# kicker:
# 	cd kicker && \
# mkdir -p build && cd build && \
# cmake -DCMAKE_TOOLCHAIN_FILE=../attiny_toolchain.cmake .. && make

# # Tell CMake to create compile_commands.json for debug builds for clang-tidy
# DEBUG_FLAGS=-DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# # build a specified target with CMake and Ninja
# # usage: $(call cmake_build_target, target, extraCmakeFlags)
# define cmake_build_target
# 	mkdir -p build
# 	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Debug $(DEBUG_FLAGS) --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
# endef

# define cmake_build_target_release
# 	mkdir -p build
# 	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=Release --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
# endef

# define cmake_build_target_perf
# 	mkdir -p build
# 	cd build && cmake -GNinja -Wno-dev -DCMAKE_BUILD_TYPE=RelWithDebInfo --target $1 $2 .. && ninja $(NINJA_FLAGS) $1
# endef

# all:
# 	$(call cmake_build_target, all)

# all_including_tests:
# 	$(call cmake_build_target, all)
# 	$(call cmake_build_target, test-soccer)

# all-release:
# 	$(call cmake_build_target_release, all)

# all-perf:
# 	$(call cmake_build_target_perf, all)
# perf: all-perf

# ROBOT_TESTS = test

# # Temp fix
# control-upload: configure
# 	./util/flash-mtrain
# # cd robot/build; make control-upload

# $(ROBOT_TESTS:%=test-%-upload): configure
# 	cd robot/build; make $(@F)

# # Define BUILDTYPE as Debug for this target and all subtargets
# debug : BUILDTYPE = "Debug"
# debug : kicker robot

# clean:
# 	rm -rf kicker/build
# 	rm -rf robot/build
