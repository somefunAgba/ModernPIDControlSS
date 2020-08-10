cmake_minimum_required(VERSION 2.8.4)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME ModernPIDControlSS_LIB)

set(${PROJECT_NAME}_BOARD uno)
set(${PROJECT_NAME}_PROGRAMMER avrispmkii)
set(${PROJECT_NAME}_PORT COM5)

project(${PROJECT_NAME})

link_directories(${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/${ARDUINO_SDK}/libraries)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)

set(${PROJECT_NAME}_AFLAGS -v)

add_subdirectory(src)
add_subdirectory(examples)