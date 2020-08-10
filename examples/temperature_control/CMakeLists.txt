cmake_minimum_required(VERSION 2.8.4)
set(PROJECT_NAME temperature_three)

set(${PROJECT_NAME}_BOARD uno)
set(${PROJECT_NAME}_PORT COM5)

project(${PROJECT_NAME})

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(E:/SomefunAgba/Documents/Arduino/libraries)

set(${PROJECT_NAME}_LIBS ModernPIDControlSS)

set(${PROJECT_NAME}_SKETCH step_three_controlsys.ino)

set(${PROJECT_NAME}_AFLAGS -v)

generate_arduino_firmware(${PROJECT_NAME})