cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    project(MotorControlSDK VERSION 1.0.0 LANGUAGES C)
endif()

add_library(motor_control_sdk STATIC
    "${CMAKE_CURRENT_LIST_DIR}/src/MotorSDK.c"
)
add_library(MotorSDK::MotorSDK ALIAS motor_control_sdk)

target_compile_features(motor_control_sdk PUBLIC c_std_11)
target_include_directories(motor_control_sdk
    PUBLIC
        "${CMAKE_CURRENT_LIST_DIR}/include"
)

set_target_properties(motor_control_sdk PROPERTIES
    OUTPUT_NAME MotorSDK
)
