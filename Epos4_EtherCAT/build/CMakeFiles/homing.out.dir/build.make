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
CMAKE_SOURCE_DIR = /home/ghan/study_ws/epos4_etherCAT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ghan/study_ws/epos4_etherCAT/build

# Include any dependencies generated for this target.
include CMakeFiles/homing.out.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/homing.out.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/homing.out.dir/flags.make

CMakeFiles/homing.out.dir/src/08_homing.c.o: CMakeFiles/homing.out.dir/flags.make
CMakeFiles/homing.out.dir/src/08_homing.c.o: ../src/08_homing.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/homing.out.dir/src/08_homing.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/homing.out.dir/src/08_homing.c.o   -c /home/ghan/study_ws/epos4_etherCAT/src/08_homing.c

CMakeFiles/homing.out.dir/src/08_homing.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/homing.out.dir/src/08_homing.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ghan/study_ws/epos4_etherCAT/src/08_homing.c > CMakeFiles/homing.out.dir/src/08_homing.c.i

CMakeFiles/homing.out.dir/src/08_homing.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/homing.out.dir/src/08_homing.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ghan/study_ws/epos4_etherCAT/src/08_homing.c -o CMakeFiles/homing.out.dir/src/08_homing.c.s

# Object files for target homing.out
homing_out_OBJECTS = \
"CMakeFiles/homing.out.dir/src/08_homing.c.o"

# External object files for target homing.out
homing_out_EXTERNAL_OBJECTS =

homing.out: CMakeFiles/homing.out.dir/src/08_homing.c.o
homing.out: CMakeFiles/homing.out.dir/build.make
homing.out: CMakeFiles/homing.out.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable homing.out"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/homing.out.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/homing.out.dir/build: homing.out

.PHONY : CMakeFiles/homing.out.dir/build

CMakeFiles/homing.out.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/homing.out.dir/cmake_clean.cmake
.PHONY : CMakeFiles/homing.out.dir/clean

CMakeFiles/homing.out.dir/depend:
	cd /home/ghan/study_ws/epos4_etherCAT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ghan/study_ws/epos4_etherCAT /home/ghan/study_ws/epos4_etherCAT /home/ghan/study_ws/epos4_etherCAT/build /home/ghan/study_ws/epos4_etherCAT/build /home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles/homing.out.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/homing.out.dir/depend
