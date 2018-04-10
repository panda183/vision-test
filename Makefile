# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/shortcut

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/shortcut

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ubuntu/shortcut/CMakeFiles /home/ubuntu/shortcut/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/ubuntu/shortcut/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named vision-test

# Build rule for target.
vision-test: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 vision-test
.PHONY : vision-test

# fast build rule for target.
vision-test/fast:
	$(MAKE) -f CMakeFiles/vision-test.dir/build.make CMakeFiles/vision-test.dir/build
.PHONY : vision-test/fast

#=============================================================================
# Target rules for targets named i2c-pwm

# Build rule for target.
i2c-pwm: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 i2c-pwm
.PHONY : i2c-pwm

# fast build rule for target.
i2c-pwm/fast:
	$(MAKE) -f peripheral_driver/i2c/CMakeFiles/i2c-pwm.dir/build.make peripheral_driver/i2c/CMakeFiles/i2c-pwm.dir/build
.PHONY : i2c-pwm/fast

#=============================================================================
# Target rules for targets named test-uart

# Build rule for target.
test-uart: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-uart
.PHONY : test-uart

# fast build rule for target.
test-uart/fast:
	$(MAKE) -f peripheral_driver/uart/CMakeFiles/test-uart.dir/build.make peripheral_driver/uart/CMakeFiles/test-uart.dir/build
.PHONY : test-uart/fast

#=============================================================================
# Target rules for targets named uart

# Build rule for target.
uart: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 uart
.PHONY : uart

# fast build rule for target.
uart/fast:
	$(MAKE) -f peripheral_driver/uart/CMakeFiles/uart.dir/build.make peripheral_driver/uart/CMakeFiles/uart.dir/build
.PHONY : uart/fast

#=============================================================================
# Target rules for targets named HAL

# Build rule for target.
HAL: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 HAL
.PHONY : HAL

# fast build rule for target.
HAL/fast:
	$(MAKE) -f HAL/CMakeFiles/HAL.dir/build.make HAL/CMakeFiles/HAL.dir/build
.PHONY : HAL/fast

#=============================================================================
# Target rules for targets named openni2

# Build rule for target.
openni2: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 openni2
.PHONY : openni2

# fast build rule for target.
openni2/fast:
	$(MAKE) -f openni2/CMakeFiles/openni2.dir/build.make openni2/CMakeFiles/openni2.dir/build
.PHONY : openni2/fast

main_pid/vision_test.o: main_pid/vision_test.cpp.o
.PHONY : main_pid/vision_test.o

# target to build an object file
main_pid/vision_test.cpp.o:
	$(MAKE) -f CMakeFiles/vision-test.dir/build.make CMakeFiles/vision-test.dir/main_pid/vision_test.cpp.o
.PHONY : main_pid/vision_test.cpp.o

main_pid/vision_test.i: main_pid/vision_test.cpp.i
.PHONY : main_pid/vision_test.i

# target to preprocess a source file
main_pid/vision_test.cpp.i:
	$(MAKE) -f CMakeFiles/vision-test.dir/build.make CMakeFiles/vision-test.dir/main_pid/vision_test.cpp.i
.PHONY : main_pid/vision_test.cpp.i

main_pid/vision_test.s: main_pid/vision_test.cpp.s
.PHONY : main_pid/vision_test.s

# target to generate assembly for a file
main_pid/vision_test.cpp.s:
	$(MAKE) -f CMakeFiles/vision-test.dir/build.make CMakeFiles/vision-test.dir/main_pid/vision_test.cpp.s
.PHONY : main_pid/vision_test.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... vision-test"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... i2c-pwm"
	@echo "... uart"
	@echo "... test-uart"
	@echo "... HAL"
	@echo "... openni2"
	@echo "... main_pid/vision_test.o"
	@echo "... main_pid/vision_test.i"
	@echo "... main_pid/vision_test.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

