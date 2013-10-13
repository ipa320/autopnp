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
# $Id: CMakeLists.txt 3693 2013-06-07 17:45:09Z geisinger $
#
# File:
#         CMake configuration file for primitive XME components.
#

xme_add_component(
    "xme_prim_digitalOutput"
    digitalOutput.h digitalOutput.c
    xme_hal_dio
)

xme_add_component(
    "xme_prim_ipLoginClientProxy"
    ipLoginClientProxy.h ipLoginClientProxy.c
    xme_hal_net
)

xme_add_component(
    "xme_prim_ipLoginServerProxy"
    ipLoginServerProxy.h ipLoginServerProxy.c
    xme_hal_net
    xme_hal_time
)

xme_add_component (
    "xme_prim_randomArrayGenerator"
    randomArrayGenerator.h randomArrayGenerator.c
    xme_hal_random
)

xme_add_component (
    "xme_prim_randomNumberGenerator"
    randomNumberGenerator.h randomNumberGenerator.c
    xme_hal_random
)

xme_add_component (
    "xme_prim_randomStringGenerator"
    randomStringGenerator.h randomStringGenerator.c
    xme_hal_random
)

xme_add_component(
    "xme_prim_topicDump"
    topicDump.h topicDump.c
)

xme_add_component(
    "xme_prim_neighborhoodDiscovery"
    neighborhoodDiscovery.c neighborhoodDiscovery.h
    xme_hal_net
)

xme_add_component(
    "xme_prim_canTransceiver"
    canTransceiver.c canTransceiver.h
    xme_hal_can
)

xme_add_component(
    "xme_prim_tdmaScheduler"
    tdmaScheduler.c tdmaScheduler.h
    xme_core_log
    xme_hal_linkedList
    xme_hal_mem
    xme_hal_sched
    xme_hal_sleep
    xme_hal_sync
    xme_hal_time
)
