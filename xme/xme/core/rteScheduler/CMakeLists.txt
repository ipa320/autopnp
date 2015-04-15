#
# Copyright (c) 2011-2012, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7459 2014-02-18 10:25:58Z geisinger $
#
# File:
#         CMake configuration file for XME Execution Manager.
#

# source files for core loop
set(LOOP_SOURCES
    src/chunk.c
    src/rteScheduler.c
)

# header files for core loop
set(LOOP_HEADERS
    include/coreLoopConfig.h
    include/coreLoopChunk.h
    include/coreLoopScheduler.h
)

xme_add_component(
    "xme_core_loop"
    ${LOOP_SOURCES}
    ${LOOP_HEADERS}
    xme_core_executionManager
)

