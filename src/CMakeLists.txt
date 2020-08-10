cmake_minimum_required(VERSION 2.8.4)
set(PROJECT_NAME ModernPIDControlSS)

set(${PROJECT_NAME}_BOARD uno)
set(${PROJECT_NAME}_PORT COM5)

project(${PROJECT_NAME})

include_directories(${PROJECT_SOURCE_DIR})

link_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${ARDUINO_SDK}/libraries/)

set(${PROJECT_NAME}_SRCS nlsig/nlsig.cpp pidkernel/PIDNet.cpp cplmfc/cplmfc.cpp
        cplmfc/filterFO_pass.cpp dynsys/testsys_ss.cpp )

set(${PROJECT_NAME}_HDRS ModernPIDControlSS.h)

set(${PROJECT_NAME}_AFLAGS -v)

generate_arduino_library(${PROJECT_NAME})