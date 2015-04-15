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

xme_add_component(
    "xme_hal_time"
    xme/hal/time_arch.h
    xme/hal/time_arch.c
)

xme_include_directory()
