# Copyright (c) 2024 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# set(HPM_BUILD_TYPE "flash_xip")
set(HPM_BUILD_TYPE "flash_uf2")


if(HPM_BUILD_TYPE STREQUAL "flash_uf2")
set(CUSTOM_GCC_LINKER_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/linkfile/flash_uf2.ld
)
else()
set(CUSTOM_GCC_LINKER_FILE
        ${CMAKE_CURRENT_SOURCE_DIR}/linkfile/flash_xip.ld
)
endif()

# USB Device
set(CONFIG_USB_DEVICE 1)

# CherryUSB
set(CONFIG_CHERRYUSB 1)
set(CONFIG_CHERRYUSB_DEVICE 1)

# SEGGER RTT
set(CONFIG_NDEBUG_CONSOLE 1)
set(CONFIG_SEGGER_RTT 1)

find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(osbdm)

sdk_compile_options("-O3")

sdk_inc(application/include)

# Application
sdk_app_inc(application/include)
sdk_app_src(application/source/main.c)
sdk_app_src(application/source/usb_osbdm.c)

# Board
sdk_app_inc(board/include)
sdk_app_src(board/source/board_custom.c)
sdk_app_src(board/source/HSLink_Pro_expansion.cpp)

# EasyLogger
sdk_app_inc(middleware/easylogger/inc)
sdk_app_src(middleware/easylogger/port/elog_port.c)
sdk_app_src(middleware/easylogger/src/elog.c)
# sdk_app_src(middleware/easylogger/src/elog_async.c)
# sdk_app_src(middleware/easylogger/src/elog_buf.c)
sdk_app_src(middleware/easylogger/src/elog_utils.c)

# # SystemView
# sdk_compile_definitions(-DSEGGER_RTT_MAX_NUM_UP_BUFFERS=1)
# sdk_compile_definitions(-DSEGGER_RTT_MAX_NUM_DOWN_BUFFERS=1)
# sdk_app_inc(middleware/SystemView/Config)
# sdk_app_inc(middleware/SystemView/SEGGER)
# sdk_app_src(middleware/SystemView/SEGGER/SEGGER_RTT_printf.c)
# sdk_app_src(middleware/SystemView/SEGGER/SEGGER_RTT.c)
# # sdk_app_src(middleware/SystemView/SEGGER/SEGGER_SYSVIEW.c)

# SEGGER RTT
sdk_compile_definitions(-DCONFIG_NDEBUG_CONSOLE=1)
sdk_ses_opt_lib_io_type(RTT)
sdk_ses_opt_debug_connection(J-Link)

# OSBDM
sdk_app_inc(osbdm)
sdk_app_inc(osbdm/common)
sdk_app_src(osbdm/board_id.c)
sdk_app_src(osbdm/cmd_processing.c)
sdk_app_src(osbdm/jtag_eppc.c)
sdk_app_src(osbdm/util.c)

# generate_ide_projects()

if(HPM_BUILD_TYPE STREQUAL "flash_uf2")
add_custom_command(
        TARGET ${APP_ELF_NAME} POST_BUILD
        COMMAND ${python_exec} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/uf2conv.py -f 0x0A4D5048 -b 0x80020000 -c ${CMAKE_BINARY_DIR}/output/${APP_BIN_NAME} -o ${CMAKE_BINARY_DIR}/output/${APP_NAME}.uf2
        COMMENT "Convert to UF2"
)
endif()

