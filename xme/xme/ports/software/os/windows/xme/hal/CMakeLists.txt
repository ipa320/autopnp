#
# Copyright (c) 2011-2013, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7718 2014-03-07 17:38:09Z ruiz $
#
# File:
#         CMake configuration script for Windows version of XME HAL components.
#

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/base64io/generic)

xme_add_component(
    "xme_hal_base64io"
    base64io_arch.h
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/byteOrder/generic)

xme_add_component(
    "xme_hal_byteOrder"
    LINK Ws2_32
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/circularBuffer/dynamicMemory)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/cmdLine/freegetopt)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/context/basedOnTls)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/dio/generic)

xme_add_component(
    "xme_hal_energy"
    energy_arch.c
)

xme_add_component(
    "xme_hal_env"
    env_arch.c
    xme_core_log
    xme_hal_safeString
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/fileio/generic)

xme_add_component(
    "xme_hal_fileio"
    fileio_arch.h
    xme_defines
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/graph/generic)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/graphviz/generic)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/linkedList/dynamicMemory)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/toolchain/stdlib/mem)

xme_add_component(
    "xme_hal_mmap"
    mmap_arch.h
    mmap_arch.c
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/net/winPosix)

xme_add_component(
    "xme_hal_net"
    LINK Ws2_32
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/toolchain/stdlib/random)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/ros/generic)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/qt/generic)

xme_add_component(
    "xme_hal_safeString"
    safeString_arch.h safeString_arch.c
    xme_core_fallback
)

xme_add_component(
    "xme_hal_sched"
    sched_arch.c
    xme_hal_context
    xme_hal_random
    xme_hal_tls
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/sharedPtr/dynamicMemory)

xme_add_component(
    "xme_hal_sleep"
    sleep_arch.c
)

xme_add_component(
    "xme_hal_sync"
    sync_arch.h
    sync_arch.c
    "xme_hal_table"
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/table/dynamicMemory)

xme_add_component(
    "xme_hal_task"
    task_arch.h
    task_arch.c
)

xme_add_component(
    "xme_hal_time"
    time_arch.h
    time_arch.c
    xme_defines
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/tls/multiThread)

xme_add_component(
    "xme_hal_tls"
    tls_arch.h
    tls_arch.c
    xme_defines
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/xml/tinyxml2)
#xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/xml/expat)
