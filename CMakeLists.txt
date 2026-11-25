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
    set(CMAKE_BUILD_TYPE "Release")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME DM)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
# 注意: Core/Src/main.c / gpio.c / dma.c / fdcan.c / usart.c / stm32h7xx_it.c
#       由 cmake/stm32cubemx/CMakeLists.txt 里的 MX_Application_Src 加入, 这里不要重复添加
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/dm_motor_ctrl.c
        Core/Src/motor_control.c
        # TIM3 没有出现在 DM.ioc 里(CubeMX 重新 Generate 会把 tim.c 从列表里丢掉),
        # 所以在这里手工加进来: 1kHz 控制节拍就靠它, 少了会链接报 undef MX_TIM3_Init/htim3
        Core/Src/tim.c
        application/vofa.c
        application/bsp_rc.c
        application/detect_task.c
        application/remote_control.c
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        application
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# dm_log() 里有 %f: newlib-nano 默认不带浮点 printf, 不链接 _printf_float 会打印出空
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
        -u _printf_float
        # 消掉链接时的 "DM.elf has a LOAD segment with RWX permissions" 警告:
        # ST 官方链接脚本把整个Flash段标成 rwx(读+写+执行), 裸机没有MMU去强制权限, 完全无害.
        # 是 GCC 14 自带的新版 binutils 默认对 rwx 段告警, 这里显式关掉
        -Wl,--no-warn-rwx-segments
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
)