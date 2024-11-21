cmake_minimum_required(VERSION 3.10)
project(CSVMotorController)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library directory
link_directories(/opt/etherlab/lib)

# 첫 번째 실행 파일: CSVMotorController
set(CSV_SOURCES src/CSVMotorController.cpp src/main.cpp)
add_executable(CSVMotorController ${CSV_SOURCES})
target_link_libraries(CSVMotorController ethercat)

# 두 번째 실행 파일: TwoController
set(TWO_SOURCES src/CSVMotorController.cpp src/two_motors.cpp)
add_executable(TwoController ${TWO_SOURCES})
target_link_libraries(TwoController ethercat)

