cmake_minimum_required(VERSION 3.10)
project(CSVMotorController)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library directory
link_directories(/opt/etherlab/lib)

# executable file 1: CSVMotorController
set(CSV_SOURCES src/CSVMotorController.cpp src/main.cpp)
add_executable(CSVMotorController ${CSV_SOURCES})
target_link_libraries(CSVMotorController ethercat)

# executable file 2: TwoController
set(TWO_SOURCES src/CSVMotorController.cpp src/two_motors.cpp)
add_executable(TwoController ${TWO_SOURCES})
target_link_libraries(TwoController ethercat)

# executable file 3: MultipleTarget
set(MULTIPLE_SOURCE src/CSVMotorController.cpp src/multiple_target.cpp)
add_executable(MultipleTarget ${MULTIPLE_SOURCE})
target_link_libraries(MultipleTarget ethercat)

