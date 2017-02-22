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
# $Id: CMakeLists.txt 4675 2013-08-13 13:57:57Z kukreja $
#

xme_add_component(
    "xme_hal_byteOrder"
    xme/hal/byteOrder_arch.h
    xme/hal/byteOrder_arch.c
)

xme_include_directory()
