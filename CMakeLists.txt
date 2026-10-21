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
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME foc)

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

file(GLOB_RECURSE USER_CPP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LIB/*.cpp
)

file(GLOB_RECURSE USER_HPP_SOURCES CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/BSP/*.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/LIB/*.hpp
)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Task/app_main.cpp
    Task/test.cpp
    ${USER_CPP_SOURCES}
    ${USER_HPP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Task
    BSP
    LIB
    LIB/controller
    LIB/comm
    LIB/encoder
    LIB/estimater
    LIB/motor
    LIB/tools
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Host-side default configuration image; never parsed on the MCU.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import yaml"
    RESULT_VARIABLE YAML_IMPORT_RESULT)
if(NOT YAML_IMPORT_RESULT EQUAL 0)
    message(FATAL_ERROR "PyYAML is required: ${Python3_EXECUTABLE} -m pip install PyYAML")
endif()
set(CONFIG_OUTPUTS
    "${CMAKE_CURRENT_BINARY_DIR}/default_config.bin"
    "${CMAKE_CURRENT_BINARY_DIR}/default_config.hex"
    "${CMAKE_CURRENT_BINARY_DIR}/default_config.json")
add_custom_command(OUTPUT ${CONFIG_OUTPUTS}
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_flash_config.py"
        --input "${CMAKE_CURRENT_SOURCE_DIR}/default_config.yaml"
        --output-dir "${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS default_config.yaml scripts/generate_flash_config.py
    VERBATIM)
add_custom_target(default_config ALL DEPENDS ${CONFIG_OUTPUTS})
add_dependencies(${CMAKE_PROJECT_NAME} default_config)

set(JLINK_EXECUTABLE "JLinkExe" CACHE STRING "J-Link Commander executable")
set(JLINK_SERIAL "" CACHE STRING "J-Link probe serial number (required if multiple probes are connected)")
set(JLINK_SELECT_ARGS)
if(JLINK_SERIAL)
    list(APPEND JLINK_SELECT_ARGS -USB "${JLINK_SERIAL}")
endif()
set(FLASH_ELF "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.elf")
configure_file(scripts/flash.jlink.in flash.jlink @ONLY)
add_custom_target(flash
    COMMAND "${JLINK_EXECUTABLE}" -NoGui 1 -ExitOnError 1 -AutoConnect 1
        -device STM32G431CB -if SWD -speed 4000 ${JLINK_SELECT_ARGS}
        -CommanderScript "${CMAKE_CURRENT_BINARY_DIR}/flash.jlink"
    DEPENDS ${CMAKE_PROJECT_NAME} default_config
    USES_TERMINAL VERBATIM)
