# 빌드 명령어
# cd ~/study_ws/epos4_etherCAT/build
# cmake ..
# make

# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name
project(epos4_etherCAT_project)

# Set the C standard to C11 (optional)
set(CMAKE_C_STANDARD 11)

# Source files directory
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

# Include directories (include your custom path and the default include path)
include_directories(${PROJECT_SOURCE_DIR}/include /home/ghan/study_ws/include)

# Library directories (add the necessary library path)
link_directories(/opt/etherlab/lib)

# Executables to be built with .out extension
add_executable(start_code_epos4_ecat.out ${SRC_DIR}/01_start_code_epos4_ecat.c)
target_link_libraries(start_code_epos4_ecat.out ethercat m)

add_executable(two_motors_and_logging.out ${SRC_DIR}/02_two_motors_and_logging.c)
target_link_libraries(two_motors_and_logging.out ethercat m)

add_executable(csv_with_pi_controller.out ${SRC_DIR}/03_csv_with_pi_controller.c)
target_link_libraries(csv_with_pi_controller.out ethercat m)

add_executable(csp_poly5.out ${SRC_DIR}/04_csp_poly5.c)
target_link_libraries(csp_poly5.out ethercat m)

add_executable(sine_sweep_csp.out ${SRC_DIR}/05_sine_sweep_csp.c)
target_link_libraries(sine_sweep_csp.out ethercat m)

add_executable(sine_sweep_ppm.out ${SRC_DIR}/06_sine_sweep_ppm.c)
target_link_libraries(sine_sweep_ppm.out ethercat m)

add_executable(sine_sweep_csv.out ${SRC_DIR}/07_sine_sweep_csv.c)
target_link_libraries(sine_sweep_csv.out ethercat m)

add_executable(homing.out ${SRC_DIR}/08_homing.c)
target_link_libraries(homing.out ethercat m)

# 새로운 프로그램을 추가
# add_executable(new_program.out ${SRC_DIR}/new_program.c)
# target_link_libraries(new_program.out ethercat m)


