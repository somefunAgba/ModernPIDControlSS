cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME ModernPIDControlSS)
set(${PROJECT_NAME}_BOARD mega)
set(ARDUINO_CPU atmega2560)

project(${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR})

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/ ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/${ARDUINO_SDK}/libraries)

set(${PROJECT_NAME}_SRCS nlsig.cpp sfunPID.cpp cplm_kernel.cpp testsys_ss.cpp tuneWn.cpp tuneKp.cpp tuneKi.cpp tuneKd.cpp)

set(${PROJECT_NAME}_HDRS ModernPIDControlSS.h)
# set(${PROJECT_NAME}_LIBS)

set(${PROJECT_NAME}_AFLAGS -v)

generate_arduino_library(${PROJECT_NAME})

add_subdirectory(examples)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)# set(${PROJECT_NAME}_SKETCH ModernPIDControlSS.ino)
set(${PROJECT_NAME}_PROGRAMMER avrispmkii)
set(${PROJECT_NAME}_PORT COM3)
# set(mega.upload.speed 9600)
