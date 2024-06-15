cmake_minimum_required(VERSION 3.16)

project(pi_i2c VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME}_lib STATIC
              src/pi_i2c.cpp
              src/MPU6050.cpp
              )

target_include_directories(${PROJECT_NAME}_lib PUBLIC include)