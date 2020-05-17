.PHONY : all kicker configure robot control-upload docs $(ROBOT_TESTS:%=test-%-upload)

all: kicker robot

kicker:
	cd kicker && \
mkdir -p build && cd build && \
cmake -DCMAKE_TOOLCHAIN_FILE=../attiny_toolchain.cmake .. && make

# Define BUILDTYPE as Release if not already set for this target and subtargets
robot/build/conaninfo.txt : BUILDTYPE ?= "Release"
robot/build/conaninfo.txt : robot/conanfile.py
	cd robot && conan install . -if build -pr armv7hf -r robojackets -s build_type=$(BUILDTYPE) --build missing

configure : robot/build/conaninfo.txt
	cd robot && conan build . -bf build -c


ROBOT_TESTS = test

robot : robot/build/conaninfo.txt
	cd robot && conan build . -bf build

# Temp fix
control-upload: configure
	./util/flash-mtrain
# cd robot/build; make control-upload

$(ROBOT_TESTS:%=test-%-upload): configure
	cd robot/build; make $(@F)

# Define BUILDTYPE as Debug for this target and all subtargets
debug : BUILDTYPE = "Debug"
debug : kicker robot

clean:
	rm -rf kicker/build
	rm -rf robot/build
	conan remove RoboCupFirmware/* --builds
	conan remove mTrain/* --builds

docs:
	cd doc && doxygen Doxyfile
	@echo "\n=> Open up 'docs/index.html' in a browser to view a local copy of the documentation"
