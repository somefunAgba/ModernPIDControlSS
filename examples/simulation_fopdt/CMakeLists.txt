cmake_minimum_required(VERSION 2.8.4)
set(PROJECT_NAME examples)

set(${PROJECT_NAME}_BOARD uno)
set(${PROJECT_NAME}_PORT COM5)

project(${PROJECT_NAME})

link_directories(..)
include_directories(E:/SomefunAgba/Documents/Arduino/libraries)

set(${PROJECT_NAME}_LIBS ModernPIDControlSS)

set(${PROJECT_NAME}_SKETCH simulation_fopdt.ino)

set(${PROJECT_NAME}_AFLAGS -v)

generate_arduino_firmware(${PROJECT_NAME})