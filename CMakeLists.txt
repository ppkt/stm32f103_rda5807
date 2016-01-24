PROJECT(stm32f103_rda5807)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
ENABLE_LANGUAGE(ASM)

FIND_PACKAGE(CMSIS REQUIRED)
FIND_PACKAGE(StdPeriphLib REQUIRED)

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMSIS_INCLUDE_DIR}
    ${StdPeriphLib_INCLUDE_DIR}
)

SET(PROJECT_SOURCES
    main.c
    misc_functions.c
    stm32f10x_it.c

    common_lib/utils.c
    common_lib/usart.c
    common_lib/i2c_dma.c
    common_lib/interrupts.c

    device_lib/rda5807.c
    device_lib/hd44780-i2c.c
    device_lib/ir.c
    device_lib/at24c64.c
    device_lib/esp8266.c

    stdio/printf.c
    stdio/syscalls.c
)

INCLUDE_DIRECTORIES(common_lib device_lib stdio)

SET(STM32_LINKER_SCRIPT ${CMSIS_LINKER_SCRIPT})

ADD_EXECUTABLE(${CMAKE_PROJECT_NAME} ${PROJECT_SOURCES} ${CMSIS_STARTUP_SOURCE} ${CMSIS_SOURCES} ${StdPeriphLib_SOURCES})

STM32_SET_TARGET_PROPERTIES(${CMAKE_PROJECT_NAME})
STM32_SET_HSE_VALUE(${CMAKE_PROJECT_NAME} 8000000)
STM32_ADD_HEX_BIN_TARGETS(${CMAKE_PROJECT_NAME})
