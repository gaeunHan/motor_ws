cmake_minimum_required(VERSION 3.10)
project(CSVMotorController)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)

# Library directory
link_directories(/opt/etherlab/lib)

# 소스 파일 목록
set(SOURCES src/CSVMotorController.cpp src/main.cpp)

# 실행 파일 생성
add_executable(CSVMotorController ${SOURCES})

# EtherCAT 라이브러리 링크
target_link_libraries(CSVMotorController ethercat)

