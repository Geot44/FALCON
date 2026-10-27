
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

mcux_add_configuration(
    CC "-DSDK_DEBUGCONSOLE=1"
    CX "-DSDK_DEBUGCONSOLE=1"
)


mcux_add_source(
    SOURCES frdmmcxn236/board.c
            frdmmcxn236/board.h
)

mcux_add_include(
    INCLUDES frdmmcxn236
)

mcux_add_source(
    SOURCES frdmmcxn236/clock_config.c
            frdmmcxn236/clock_config.h
)

mcux_add_include(
    INCLUDES frdmmcxn236
)

mcux_add_source(
    SOURCES freertos_hello/pin_mux.c
            freertos_hello/pin_mux.h
)

mcux_add_include(
    INCLUDES freertos_hello
)

mcux_add_source(
    SOURCES freertos_hello/app.h
            freertos_hello/hardware_init.c
)

mcux_add_include(
    INCLUDES freertos_hello
)
