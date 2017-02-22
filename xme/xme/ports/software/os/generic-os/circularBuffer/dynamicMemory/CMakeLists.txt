#
# Copyright (c) 2011-2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
# 
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7142 2014-02-10 12:08:51Z geisinger $
#

xme_add_component(
    "xme_hal_circularBuffer"
    xme/hal/circularBuffer_arch.c
    xme_hal_mem
    xme_defines
)

xme_include_directory()
