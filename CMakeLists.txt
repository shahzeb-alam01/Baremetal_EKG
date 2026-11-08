cmake_minimum_required(VERSION 3.21)

# Project definition
project(Baremetal_ECG
    VERSION 1.0.0
    DESCRIPTION "Bare Metal ECG Project with STM32G4xx"
    LANGUAGES C ASM
)

# MCU Configuration
set(MCU_NAME stm32g431rbtx)
set(MCU_FAMILY STM32G4xx)
set(ARM_ARCH armv7-m)
set(ARM_CPU Cortex-M4)

# Set compile flags
set(COMMON_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -Wextra
    -O2
)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${COMMON_FLAGS} -x assembler-with-cpp")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS
    "${COMMON_FLAGS} -nostartfiles -Wl,-Map=output.map,--cref"
)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/Baremetal_ECG/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Baremetal_ECG/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Baremetal_ECG/Drivers/STM32G4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Baremetal_ECG/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
)

# Source files
file(GLOB_RECURSE CORE_SOURCES "Baremetal_ECG/Core/Src/*.c")
file(GLOB_RECURSE HAL_SOURCES "Baremetal_ECG/Drivers/STM32G4xx_HAL_Driver/Src/*.c")
file(GLOB_RECURSE STARTUP_SOURCES "Baremetal_ECG/EWARM/startup*.s")

set(SOURCES
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SOURCES}
)

# Create the executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Compile definitions
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE
    USE_HAL_DRIVER
    STM32G431xx
    ARM_MATH_CM4
    CMSIS_DEVICE_HEADER="stm32g431xx.h"
)

# Additional build targets
add_custom_target(hex ALL
    COMMAND ${CMAKE_OBJCOPY} -Oihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Generating HEX file"
)

add_custom_target(bin ALL
    COMMAND ${CMAKE_OBJCOPY} -Obinary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    DEPENDS ${PROJECT_NAME}.elf
    COMMENT "Generating BIN file"
)

# Display build information
message(STATUS "=== Baremetal ECG Build Configuration ===")
message(STATUS "MCU: ${MCU_NAME}")
message(STATUS "ARM Architecture: ${ARM_ARCH}")
