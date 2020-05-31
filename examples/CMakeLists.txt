cmake_minimum_required(VERSION 2.8.4)
set(PROJECT_NAME examples)
set(${PROJECT_NAME}_BOARD mega)
 set(ARDUINO_CPU atmega2560)

project(${PROJECT_NAME})

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
#include_directories(E:/SomefunAgba/Documents/Arduino/libraries)

set(${PROJECT_NAME}_SKETCH simulation_fopdt/simulation_fopdt.ino)
#Set up target COM port, which is connect to the board
set(${PROJECT_NAME}_PORT COM3)

set(${PROJECT_NAME}_LIBS ModernPIDControlSS)

set(${PROJECT_NAME}_AFLAGS -v)

#add_executable(simulation_fopdt/simulation_fopdt.ino)

generate_arduino_firmware(${PROJECT_NAME})