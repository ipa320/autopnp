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
# $Id: CMakeLists.txt 6626 2014-02-05 14:36:43Z geisinger $
#
# File:
#         CMake configuration file for XME HAL components.
#

xme_add_subdirectory (doc)
xme_add_subdirectory (test)

xme_add_component(
    "xme_hal_base64io"
    include/base64io.h
)

xme_add_component(
    "xme_hal_byteOrder"
    include/byteOrder.h
)

xme_add_component(
    "xme_hal_can"
    include/can.h
)

xme_add_component(
    "xme_hal_circularBuffer"
    include/circularBuffer.h
	xme_core_log
)

xme_add_component(
    "xme_hal_cmdLine"
    include/cmdLine.h
)

xme_add_component(
    "xme_hal_console"
    include/console.h
)

xme_add_component(
    "xme_hal_dio"
    include/dio.h
    xme_hal_io
)

xme_add_component(
    "xme_hal_display"
    include/display.h
)

xme_add_component(
    "xme_hal_energy"
    include/energy.h
)

xme_add_component(
    "xme_hal_env"
    include/env.h
)

xme_add_component(
    "xme_hal_fileio"
    include/fileio.h
)

xme_add_component(
    "xme_hal_graph"
    include/graph.h
)

xme_add_component(
    "xme_hal_graphviz"
    include/graphviz.h
)

xme_add_component(
    "xme_hal_io"
    include/io.h
)

xme_add_component(
    "xme_hal_linkedList"
    include/linkedList.h
)

xme_add_component(
    "xme_hal_math"
    include/math.h
)

xme_add_component(
    "xme_hal_mmap"
    include/mmap.h
)

xme_add_component(
    "xme_hal_net"
    include/net.h
)

xme_add_component(
    "xme_hal_qt"
    include/qt.h
)

xme_add_component(
    "xme_hal_random"
    include/random.h
)

xme_add_component(
    "xme_hal_ros"
    include/ros.h
)

xme_add_component(
    "xme_hal_safeString"
    include/safeString.h
)

xme_add_component(
    "xme_hal_sched"
    include/sched.h
)

xme_add_component(
    "xme_hal_mem"
    include/mem.h
)

xme_add_component(
    "xme_hal_sharedPtr"
    include/sharedPtr.h
    xme_hal_mem
)

xme_add_component(
    "xme_hal_sleep"
    include/sleep.h
    xme_hal_time
)

xme_add_component(
    "xme_hal_sync"
    include/sync.h
    "xme_hal_table"
)

xme_add_component(
    "xme_hal_table"
    include/table.h
)

xme_add_component(
    "xme_hal_task"
    include/task.h
)

xme_add_component(
    "xme_hal_time"
    include/time.h
)

xme_add_component(
    "xme_hal_tls"
    include/tls.h
)

xme_add_component(
    "xme_hal_xml"
    include/xml.h
)
