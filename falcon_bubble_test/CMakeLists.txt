
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

cmake_path(APPEND conf_file_path ${CMAKE_CURRENT_LIST_DIR} ${board} prj.conf)
set(CONF_FILE ${conf_file_path} ${CONF_FILE})
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(falcon_bubble_test LANGUAGES C CXX ASM PROJECT_BOARD_PORT_PATH ${board_root}/${board}/demo_apps/bubble)

include(${SdkRootDirPath}/CMakeLists.txt)


include(${board}/board_files.cmake)

mcux_add_source(
    SOURCES bubble/bubble.c
)
mcux_add_macro(
    CC "-DPRINTF_ADVANCED_ENABLE=1\
       -DSDK_I2C_BASED_COMPONENT_USED=1\
       -DFRDM_MCXN236\
       -DFREEDOM"
)

mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)
mcux_add_config_mex_path( PATH ${board} )
mcux_add_include(INCLUDES bubble)
