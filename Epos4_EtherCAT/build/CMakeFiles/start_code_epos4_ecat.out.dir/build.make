# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/ghan/motor_ws/Epos4_EtherCAT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ghan/motor_ws/Epos4_EtherCAT/build

# Include any dependencies generated for this target.
include CMakeFiles/start_code_epos4_ecat.out.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/start_code_epos4_ecat.out.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/start_code_epos4_ecat.out.dir/flags.make

CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o: CMakeFiles/start_code_epos4_ecat.out.dir/flags.make
CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o: ../src/01_start_code_epos4_ecat.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ghan/motor_ws/Epos4_EtherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o   -c /home/ghan/motor_ws/Epos4_EtherCAT/src/01_start_code_epos4_ecat.c

CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ghan/motor_ws/Epos4_EtherCAT/src/01_start_code_epos4_ecat.c > CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.i

CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ghan/motor_ws/Epos4_EtherCAT/src/01_start_code_epos4_ecat.c -o CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.s

# Object files for target start_code_epos4_ecat.out
start_code_epos4_ecat_out_OBJECTS = \
"CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o"

# External object files for target start_code_epos4_ecat.out
start_code_epos4_ecat_out_EXTERNAL_OBJECTS =

start_code_epos4_ecat.out: CMakeFiles/start_code_epos4_ecat.out.dir/src/01_start_code_epos4_ecat.c.o
start_code_epos4_ecat.out: CMakeFiles/start_code_epos4_ecat.out.dir/build.make
start_code_epos4_ecat.out: CMakeFiles/start_code_epos4_ecat.out.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ghan/motor_ws/Epos4_EtherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable start_code_epos4_ecat.out"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/start_code_epos4_ecat.out.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/start_code_epos4_ecat.out.dir/build: start_code_epos4_ecat.out

.PHONY : CMakeFiles/start_code_epos4_ecat.out.dir/build

CMakeFiles/start_code_epos4_ecat.out.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/start_code_epos4_ecat.out.dir/cmake_clean.cmake
.PHONY : CMakeFiles/start_code_epos4_ecat.out.dir/clean

CMakeFiles/start_code_epos4_ecat.out.dir/depend:
	cd /home/ghan/motor_ws/Epos4_EtherCAT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ghan/motor_ws/Epos4_EtherCAT /home/ghan/motor_ws/Epos4_EtherCAT /home/ghan/motor_ws/Epos4_EtherCAT/build /home/ghan/motor_ws/Epos4_EtherCAT/build /home/ghan/motor_ws/Epos4_EtherCAT/build/CMakeFiles/start_code_epos4_ecat.out.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/start_code_epos4_ecat.out.dir/depend

