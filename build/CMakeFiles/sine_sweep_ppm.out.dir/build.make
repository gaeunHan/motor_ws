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
include CMakeFiles/sine_sweep_ppm.out.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sine_sweep_ppm.out.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sine_sweep_ppm.out.dir/flags.make

CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o: CMakeFiles/sine_sweep_ppm.out.dir/flags.make
CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o: ../src/06_sine_sweep_ppm.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o   -c /home/ghan/study_ws/epos4_etherCAT/src/06_sine_sweep_ppm.c

CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ghan/study_ws/epos4_etherCAT/src/06_sine_sweep_ppm.c > CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.i

CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ghan/study_ws/epos4_etherCAT/src/06_sine_sweep_ppm.c -o CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.s

# Object files for target sine_sweep_ppm.out
sine_sweep_ppm_out_OBJECTS = \
"CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o"

# External object files for target sine_sweep_ppm.out
sine_sweep_ppm_out_EXTERNAL_OBJECTS =

sine_sweep_ppm.out: CMakeFiles/sine_sweep_ppm.out.dir/src/06_sine_sweep_ppm.c.o
sine_sweep_ppm.out: CMakeFiles/sine_sweep_ppm.out.dir/build.make
sine_sweep_ppm.out: CMakeFiles/sine_sweep_ppm.out.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C executable sine_sweep_ppm.out"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sine_sweep_ppm.out.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sine_sweep_ppm.out.dir/build: sine_sweep_ppm.out

.PHONY : CMakeFiles/sine_sweep_ppm.out.dir/build

CMakeFiles/sine_sweep_ppm.out.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sine_sweep_ppm.out.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sine_sweep_ppm.out.dir/clean

CMakeFiles/sine_sweep_ppm.out.dir/depend:
	cd /home/ghan/study_ws/epos4_etherCAT/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ghan/study_ws/epos4_etherCAT /home/ghan/study_ws/epos4_etherCAT /home/ghan/study_ws/epos4_etherCAT/build /home/ghan/study_ws/epos4_etherCAT/build /home/ghan/study_ws/epos4_etherCAT/build/CMakeFiles/sine_sweep_ppm.out.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sine_sweep_ppm.out.dir/depend

