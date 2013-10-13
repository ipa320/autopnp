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
# $Id: CMakeLists.txt 4334 2013-07-23 16:46:40Z geisinger $
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

xme_include_directory()
