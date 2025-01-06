cmake_minimum_required(VERSION 3.10)
project(Controllers)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/home/robogram/ros2_ws/src/test_pkg/include) # shared_memory.hpp

# Library directory
link_directories(/opt/etherlab/lib) # etherlab

# executable file 1: 01_CSP_ZERO_VEL
set(Controllers_01_SOURCES src/MotorController.cpp src/csp_zero_vel.cpp)
add_executable(01_CSP_ZERO_VEL ${Controllers_01_SOURCES})
target_link_libraries(01_CSP_ZERO_VEL ethercat)
