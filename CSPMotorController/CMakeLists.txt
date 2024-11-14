cmake_minimum_required(VERSION 3.10)
project(CSPMotorController)

# 컴파일러 설정
set(CMAKE_CXX_STANDARD 11)

# include 디렉토리 추가
include_directories(${CMAKE_SOURCE_DIR}/include)

# 소스 파일 목록
set(SOURCES src/CSPMotorController.cpp src/main.cpp)

# 실행 파일 생성
add_executable(CSPMotorController ${SOURCES})

