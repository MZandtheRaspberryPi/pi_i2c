cmake_minimum_required(VERSION 3.16)

project(pi_i2c VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME}_lib STATIC
              src/i2c_linux.cpp
              src/MPU6050.cpp
              Adafruit-PWM-Servo-Driver-Library/Adafruit_PWMServoDriver.cpp
              src/utils.cpp
              src/servo_controller.cpp
              )

target_include_directories(${PROJECT_NAME}_lib PUBLIC include Adafruit-PWM-Servo-Driver-Library)