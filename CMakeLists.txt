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

set(CMAKE_CXX_STANDARD 17)          # 新增C++标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 新增C++标准要求
set(CMAKE_C_EXTENSIONS ON)      # 新增C++扩展


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME stm32Car)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C CXX ASM)          # 添加C++支持

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)
# 添加 cJSON 库
add_subdirectory(Core/Cpp/ThirdParty/cJSON)
include_directories(Core/Cpp/ThirdParty/cJSON)



# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

#新的C++源文件可以添加到工程的 target_sources 部分（通常放在主CMakeLists的user sources区域）。
# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        
        #应用层代码
        Core/Cpp/App/Src/wheel_motor_app.cpp
        Core/Cpp/App/Src/uart_app.cpp

        #驱动层代码
        Core/Cpp/Drivers/Src/wheel_motor.cpp
        Core/Cpp/Drivers/Src/pid_controller.cpp
        Core/Cpp/Drivers/Src/chassis_controller.cpp
        Core/Cpp/Drivers/Src/uart_class.cpp
        
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Cpp/Drivers/Inc
    Core/Cpp/App/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    cjson
    # Add user defined libraries
)

add_link_options(
    -specs=nosys.specs
    -specs=nano.specs
    -u _printf_float
    -u _scanf_float
)

