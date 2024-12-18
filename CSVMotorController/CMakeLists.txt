cmake_minimum_required(VERSION 3.10)
project(CSVMotorController)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/home/robogram/ros2_ws/src/test_pkg/include) # shared_memory.hpp

# Library directory
link_directories(/opt/etherlab/lib)

# executable file 1: 01_CSVMotorController
set(01_SOURCES src/CSVMotorController.cpp src/main.cpp)
add_executable(01_CSVMotorController ${01_SOURCES})
target_link_libraries(01_CSVMotorController ethercat)

# executable file 2: 02_TwoController
set(02_SOURCES src/CSVMotorController.cpp src/two_motors.cpp)
add_executable(02_TwoController ${02_SOURCES})
target_link_libraries(02_TwoController ethercat)

# executable file 3: 03_MultipleTarget
set(03_SOURCE src/CSVMotorController.cpp src/multiple_target.cpp)
add_executable(03_MultipleTarget ${03_SOURCE})
target_link_libraries(03_MultipleTarget ethercat)

# executable file 4: 04_MultiMotorTarget
set(04_SOURCE src/CSVMotorController.cpp src/multi_motor_target.cpp)
add_executable(04_MultiMotorTarget ${04_SOURCE})
target_link_libraries(04_MultiMotorTarget ethercat)

# executable file 5: 05_Robot1DOF
set(05_SOURCE src/CSVMotorController.cpp src/robot_1DOF.cpp)
add_executable(05_Robot1DOF ${05_SOURCE})
target_link_libraries(05_Robot1DOF ethercat)

# executable file 6: 06_rosQueueTest
set(06_SOURCE src/ros_queue_test.cpp)
add_executable(06_rosQueueTest ${06_SOURCE})
target_link_libraries(06_rosQueueTest ethercat)

# executable file 7: 07_rosMotorTest
set(07_SOURCE src/CSVMotorController.cpp src/ros_motor.cpp)
add_executable(07_rosMotorTest ${07_SOURCE})
target_link_libraries(07_rosMotorTest ethercat)