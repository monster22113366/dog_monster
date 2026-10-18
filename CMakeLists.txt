cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dog_test)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
set(USER_SOURCES
    USER_FILE/Bsp/bsp_can.c
    USER_FILE/Bsp/bsp_imu.c
    USER_FILE/Bsp/bsp_tim.c
    USER_FILE/Bsp/bsp_sbus.c
    USER_FILE/App/app_motorSHS.c
    USER_FILE/App/app_motorQZB.c
    USER_FILE/App/app_legs.c
    USER_FILE/App/app_chassis.c
    USER_FILE/App/app_imu.c
    USER_FILE/App/app_rc.c
    USER_FILE/App/app_ctrl.c
    USER_FILE/App/app_log.c
    USER_FILE/App/app_scheduler.c
    USER_FILE/Componnet/pid/pid.c
    USER_FILE/Componnet/leg/leg_kin.c
    USER_FILE/Componnet/vmc/vmc.c
    USER_FILE/Componnet/gait/gait.c
    USER_FILE/Componnet/att/att_est.c
    USER_FILE/Componnet/sbus/sbus.c
    USER_FILE/Componnet/motor_Communication_Protocol/mw_canSHS.c
    USER_FILE/Componnet/motor_Communication_Protocol/mw_canQZB.c
    USER_FILE/Componnet/wheel/wheel_mix.c
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})

# 仅对用户代码启用 -Og，保持可调试；CubeMX 生成代码与 HAL 仍按 CMAKE_BUILD_TYPE。
set_source_files_properties(${USER_SOURCES} PROPERTIES COMPILE_OPTIONS "-Og")

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    USER_FILE/Bsp
    USER_FILE/App
    USER_FILE/Componnet/pid
    USER_FILE/Componnet/leg
    USER_FILE/Componnet/vmc
    USER_FILE/Componnet/gait
    USER_FILE/Componnet/att
    USER_FILE/Componnet/sbus
    USER_FILE/Componnet/motor_Communication_Protocol
    USER_FILE/Componnet/wheel
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    MWSHS_BUS_NUM=2
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
