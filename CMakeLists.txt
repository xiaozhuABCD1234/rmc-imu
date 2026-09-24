cmake_minimum_required(VERSION 3.20)

project(imu_justfloat
        VERSION 1.0.0
        DESCRIPTION "RoboMaster C board BMI088 IMU streaming over UART with the VOFA+ JustFloat protocol (STM32F407IGHx)"
        LANGUAGES C ASM)

# ---------------------------------------------------------------------------
# This is a bare-metal ARM project; a cross toolchain file is required.
# ---------------------------------------------------------------------------
if(NOT CMAKE_TOOLCHAIN_FILE)
  message(FATAL_ERROR
    "This is a bare-metal ARM project and must be configured with a cross toolchain.\n"
    "  cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake\n")
endif()

# ---------------------------------------------------------------------------
# Project settings
# ---------------------------------------------------------------------------
set(TARGET_NAME    "imu_justfloat")
set(DEVICE_DEFS    STM32F407xx USE_HAL_DRIVER)
set(LINKER_SCRIPT  "${CMAKE_SOURCE_DIR}/STM32F407IGHx_FLASH.ld")

# Default to Release (-O2) when the user does not pick a build type.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Architecture flags: identical for C, assembly and the link step.
set(ARCH_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

# ---------------------------------------------------------------------------
# Sources
# ---------------------------------------------------------------------------
set(CORE_SOURCES
    Core/Src/main.c
    Core/Src/bmi088.c
    Core/Src/imu_heater.c
    Core/Src/imu_task.c
    Core/Src/imu_stream.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c)

# Only the HAL drivers this firmware calls. dma_ex, flash, flash_ex, pwr,
# pwr_ex and rcc_ex are deliberately left out: the linker map showed each of
# them contributing 0 bytes to the image. The vendor sources stay on disk, so
# re-adding one is a one-line change here.
set(HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_iwdg.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c)

# GCC startup file (the _keil.s variant is only for the ARM Compiler/Keil).
set(STARTUP_SOURCE
    startup/startup_stm32f407xx.s)

set(INCLUDE_DIRS
    Core/Inc
    Drivers/CMSIS/Core/Include
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/STM32F4xx_HAL_Driver/STM32F4xx_HAL_Driver/Inc)

# ---------------------------------------------------------------------------
# Target
# ---------------------------------------------------------------------------
add_executable(${TARGET_NAME}
    ${CORE_SOURCES}
    ${HAL_SOURCES}
    ${STARTUP_SOURCE})

set_target_properties(${TARGET_NAME} PROPERTIES
    OUTPUT_NAME "imu_justfloat"
    SUFFIX      ".elf")

target_compile_definitions(${TARGET_NAME} PRIVATE ${DEVICE_DEFS})
target_include_directories(${TARGET_NAME} PRIVATE ${INCLUDE_DIRS})

# C compilation flags.
target_compile_options(${TARGET_NAME} PRIVATE
    $<$<COMPILE_LANGUAGE:C>:${ARCH_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:-std=c11>
    $<$<COMPILE_LANGUAGE:C>:-Wall>
    $<$<COMPILE_LANGUAGE:C>:-Wextra>
    $<$<COMPILE_LANGUAGE:C>:-Wno-unused-parameter>
    $<$<COMPILE_LANGUAGE:C>:-ffunction-sections>
    $<$<COMPILE_LANGUAGE:C>:-fdata-sections>
    $<$<COMPILE_LANGUAGE:C>:-fno-common>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Release>:-O2>>
    $<$<COMPILE_LANGUAGE:C>:$<$<CONFIG:Debug>:-Og>>
    $<$<COMPILE_LANGUAGE:C>:-g3>
    # Assembly also needs the architecture flags.
    $<$<COMPILE_LANGUAGE:ASM>:${ARCH_FLAGS}>
    $<$<COMPILE_LANGUAGE:ASM>:-g>)

# Link flags.
target_link_options(${TARGET_NAME} PRIVATE
    ${ARCH_FLAGS}
    -nostartfiles
    "-T${LINKER_SCRIPT}"
    -specs=nano.specs
    -specs=nosys.specs
    -Wl,--gc-sections
    "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/imu_justfloat.map")

# ---------------------------------------------------------------------------
# Post-build: .hex / .bin generation and size report
# ---------------------------------------------------------------------------
if(NOT CMAKE_OBJCOPY)
  find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
endif()
find_program(ARM_SIZE arm-none-eabi-size REQUIRED)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${TARGET_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/imu_justfloat.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${TARGET_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/imu_justfloat.bin
    COMMAND ${ARM_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMENT "Generating imu_justfloat.hex / .bin and printing size")

# Convenience target: clean + rebuild.
add_custom_target(rebuild
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --clean-first
    COMMENT "Clean rebuild of ${TARGET_NAME}")

# ---------------------------------------------------------------------------
# Flashing via OpenOCD
#   cmake --build build --target flash
# Interface selection (board's onboard DAPLink by default):
#   -DOPENOCD_INTERFACE=stlink   (ST-Link)
#   -DOPENOCD_INTERFACE=jlink    (J-Link)
#   -DOPENOCD_INTERFACE=cmsis-dap
# ---------------------------------------------------------------------------
find_program(OPENOCD openocd)

if(OPENOCD)
  set(OPENOCD_INTERFACE "cmsis-dap" CACHE STRING "OpenOCD interface config (see scripts/interface/)")
  set(OPENOCD_TARGET    "stm32f4x"   CACHE STRING "OpenOCD target config (see scripts/target/)")

  add_custom_target(flash
      COMMAND ${OPENOCD}
              -f interface/${OPENOCD_INTERFACE}.cfg
              -f target/${OPENOCD_TARGET}.cfg
              -c "program ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.elf verify reset exit"
      DEPENDS ${TARGET_NAME}
      COMMENT "Flashing ${TARGET_NAME}.elf via OpenOCD (${OPENOCD_INTERFACE})")
else()
  message(WARNING "openocd not found on PATH; the 'flash' target is unavailable")
endif()
