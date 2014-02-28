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
# $Id: CMakeLists.txt 4366 2013-07-25 12:39:26Z geisinger $
#

xme_add_component(
    "xme_hal_linkedList"
    xme/hal/linkedList_arch.h
    xme/hal/linkedList_arch.c
    xme_defines
    xme_hal_mem
)

xme_include_directory()
