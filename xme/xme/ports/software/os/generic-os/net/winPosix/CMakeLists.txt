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
# $Id: CMakeLists.txt 7686 2014-03-05 15:22:16Z gulati $
#

xme_add_component(
    "xme_hal_net"
    xme/hal/net_arch.h
    xme/hal/net_arch.c
    xme_com_interface
    xme_core_log
    xme_hal_safeString
    xme_hal_sync
)

if (WIN32)
    xme_add_component(
        "xme_hal_net"
        LINK IPHLPAPI.lib
    )
endif ()

xme_include_directory()
