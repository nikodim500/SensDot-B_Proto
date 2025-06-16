# CMakeLists.txt for SensDot ESP32 project

cmake_minimum_required(VERSION 3.16)

# Set project name and supported languages
set(CMAKE_PROJECT_NAME "sensdot")
project(${CMAKE_PROJECT_NAME})

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(sensdot)