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
# $Id: CMakeLists.txt 5639 2013-10-25 14:42:41Z wiesmueller $
#
# File:
#         CMake configuration file for advanced XME components.
#

xme_add_component (
    "xme_adv_loginServer"
    include/loginServer.h 
    src/loginServer.c
)

xme_add_component (
    "xme_adv_heartbeat"
    include/heartbeat.h 
    include/healthmonitor.h
    src/heartbeat.c 
)

xme_add_component (
    "xme_adv_testHeartbeat"
    include/testHeartbeat.h 
    include/healthmonitor.h
    src/testHeartbeat.c 
    xme_hal_time
)

xme_add_component (
    "xme_adv_testConsistency"
    include/healthmonitor.h
    include/testConsistency.h 
    src/testConsistency.c 
)

xme_add_component (
    "xme_adv_testCpu"
    include/healthmonitor.h
    include/testCpu.h 
    src/testCpu.c
)

xme_add_component (
    "xme_adv_testMemory"
    include/healthmonitor.h
    include/testMemory.h
    src/testMemory.c
)

xme_add_component (
    "xme_adv_voter"
    include/healthmonitor.h
    include/voter.h
    src/voter.c
)

xme_add_component (
    "xme_adv_healthmonitor"
    include/healthmonitor.h
    src/healthmonitor.c
)
