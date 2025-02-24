# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robogram/motor_ws/Controllers

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robogram/motor_ws/Controllers/build

# Include any dependencies generated for this target.
include CMakeFiles/01_CSP_ZERO_VEL.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/01_CSP_ZERO_VEL.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/01_CSP_ZERO_VEL.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/01_CSP_ZERO_VEL.dir/flags.make

CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o: CMakeFiles/01_CSP_ZERO_VEL.dir/flags.make
CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o: ../src/MotorController.cpp
CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o: CMakeFiles/01_CSP_ZERO_VEL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robogram/motor_ws/Controllers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o -MF CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o.d -o CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o -c /home/robogram/motor_ws/Controllers/src/MotorController.cpp

CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robogram/motor_ws/Controllers/src/MotorController.cpp > CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.i

CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robogram/motor_ws/Controllers/src/MotorController.cpp -o CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.s

CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o: CMakeFiles/01_CSP_ZERO_VEL.dir/flags.make
CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o: ../src/csp_zero_vel.cpp
CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o: CMakeFiles/01_CSP_ZERO_VEL.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robogram/motor_ws/Controllers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o -MF CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o.d -o CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o -c /home/robogram/motor_ws/Controllers/src/csp_zero_vel.cpp

CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robogram/motor_ws/Controllers/src/csp_zero_vel.cpp > CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.i

CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robogram/motor_ws/Controllers/src/csp_zero_vel.cpp -o CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.s

# Object files for target 01_CSP_ZERO_VEL
01_CSP_ZERO_VEL_OBJECTS = \
"CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o" \
"CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o"

# External object files for target 01_CSP_ZERO_VEL
01_CSP_ZERO_VEL_EXTERNAL_OBJECTS =

01_CSP_ZERO_VEL: CMakeFiles/01_CSP_ZERO_VEL.dir/src/MotorController.cpp.o
01_CSP_ZERO_VEL: CMakeFiles/01_CSP_ZERO_VEL.dir/src/csp_zero_vel.cpp.o
01_CSP_ZERO_VEL: CMakeFiles/01_CSP_ZERO_VEL.dir/build.make
01_CSP_ZERO_VEL: CMakeFiles/01_CSP_ZERO_VEL.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robogram/motor_ws/Controllers/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable 01_CSP_ZERO_VEL"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/01_CSP_ZERO_VEL.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/01_CSP_ZERO_VEL.dir/build: 01_CSP_ZERO_VEL
.PHONY : CMakeFiles/01_CSP_ZERO_VEL.dir/build

CMakeFiles/01_CSP_ZERO_VEL.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/01_CSP_ZERO_VEL.dir/cmake_clean.cmake
.PHONY : CMakeFiles/01_CSP_ZERO_VEL.dir/clean

CMakeFiles/01_CSP_ZERO_VEL.dir/depend:
	cd /home/robogram/motor_ws/Controllers/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robogram/motor_ws/Controllers /home/robogram/motor_ws/Controllers /home/robogram/motor_ws/Controllers/build /home/robogram/motor_ws/Controllers/build /home/robogram/motor_ws/Controllers/build/CMakeFiles/01_CSP_ZERO_VEL.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/01_CSP_ZERO_VEL.dir/depend

