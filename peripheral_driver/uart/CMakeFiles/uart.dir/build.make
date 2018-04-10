# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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

# Include any dependencies generated for this target.
include peripheral_driver/uart/CMakeFiles/uart.dir/depend.make

# Include the progress variables for this target.
include peripheral_driver/uart/CMakeFiles/uart.dir/progress.make

# Include the compile flags for this target's objects.
include peripheral_driver/uart/CMakeFiles/uart.dir/flags.make

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o: peripheral_driver/uart/CMakeFiles/uart.dir/flags.make
peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o: peripheral_driver/uart/rs232.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/shortcut/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/uart.dir/rs232.c.o   -c /home/ubuntu/shortcut/peripheral_driver/uart/rs232.c

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/uart.dir/rs232.c.i"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/ubuntu/shortcut/peripheral_driver/uart/rs232.c > CMakeFiles/uart.dir/rs232.c.i

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/uart.dir/rs232.c.s"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/ubuntu/shortcut/peripheral_driver/uart/rs232.c -o CMakeFiles/uart.dir/rs232.c.s

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.requires:
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.requires

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.provides: peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.requires
	$(MAKE) -f peripheral_driver/uart/CMakeFiles/uart.dir/build.make peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.provides.build
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.provides

peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.provides.build: peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o: peripheral_driver/uart/CMakeFiles/uart.dir/flags.make
peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o: peripheral_driver/uart/api_uart.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/shortcut/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/uart.dir/api_uart.cpp.o -c /home/ubuntu/shortcut/peripheral_driver/uart/api_uart.cpp

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/uart.dir/api_uart.cpp.i"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/shortcut/peripheral_driver/uart/api_uart.cpp > CMakeFiles/uart.dir/api_uart.cpp.i

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/uart.dir/api_uart.cpp.s"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/shortcut/peripheral_driver/uart/api_uart.cpp -o CMakeFiles/uart.dir/api_uart.cpp.s

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.requires:
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.requires

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.provides: peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.requires
	$(MAKE) -f peripheral_driver/uart/CMakeFiles/uart.dir/build.make peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.provides.build
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.provides

peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.provides.build: peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o

# Object files for target uart
uart_OBJECTS = \
"CMakeFiles/uart.dir/rs232.c.o" \
"CMakeFiles/uart.dir/api_uart.cpp.o"

# External object files for target uart
uart_EXTERNAL_OBJECTS =

bin/Release/libuart.a: peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o
bin/Release/libuart.a: peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o
bin/Release/libuart.a: peripheral_driver/uart/CMakeFiles/uart.dir/build.make
bin/Release/libuart.a: peripheral_driver/uart/CMakeFiles/uart.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../../bin/Release/libuart.a"
	cd /home/ubuntu/shortcut/peripheral_driver/uart && $(CMAKE_COMMAND) -P CMakeFiles/uart.dir/cmake_clean_target.cmake
	cd /home/ubuntu/shortcut/peripheral_driver/uart && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/uart.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
peripheral_driver/uart/CMakeFiles/uart.dir/build: bin/Release/libuart.a
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/build

peripheral_driver/uart/CMakeFiles/uart.dir/requires: peripheral_driver/uart/CMakeFiles/uart.dir/rs232.c.o.requires
peripheral_driver/uart/CMakeFiles/uart.dir/requires: peripheral_driver/uart/CMakeFiles/uart.dir/api_uart.cpp.o.requires
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/requires

peripheral_driver/uart/CMakeFiles/uart.dir/clean:
	cd /home/ubuntu/shortcut/peripheral_driver/uart && $(CMAKE_COMMAND) -P CMakeFiles/uart.dir/cmake_clean.cmake
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/clean

peripheral_driver/uart/CMakeFiles/uart.dir/depend:
	cd /home/ubuntu/shortcut && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/shortcut /home/ubuntu/shortcut/peripheral_driver/uart /home/ubuntu/shortcut /home/ubuntu/shortcut/peripheral_driver/uart /home/ubuntu/shortcut/peripheral_driver/uart/CMakeFiles/uart.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : peripheral_driver/uart/CMakeFiles/uart.dir/depend

