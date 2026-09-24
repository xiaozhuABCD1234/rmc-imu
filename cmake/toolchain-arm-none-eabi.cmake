# Cross-compilation toolchain file for ARM Cortex-M bare metal targets
# using the arm-none-eabi GNU toolchain.
#
# Usage:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake
#
# If the toolchain is not on PATH, point ARM_TOOLCHAIN at its bin directory:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake \
#         -DARM_TOOLCHAIN=E:/CubeCLT/STM32CubeCLT_1.20.0/GNU-tools-for-STM32/bin

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Prefix of the ARM toolchain binaries (may be overridden on the command line).
set(ARM_TOOLCHAIN "arm-none-eabi-" CACHE STRING "Prefix of ARM toolchain binaries")

find_program(CMAKE_C_COMPILER   ${ARM_TOOLCHAIN}gcc     REQUIRED)
find_program(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN}gcc     REQUIRED)
find_program(CMAKE_OBJCOPY      ${ARM_TOOLCHAIN}objcopy REQUIRED)

# Linking is done manually against the project linker script, so a full
# compiler link test is not needed.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# The host filesystem is not a valid search root for ARM libraries/headers.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
