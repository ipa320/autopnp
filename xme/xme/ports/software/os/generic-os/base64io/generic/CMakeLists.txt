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
# $Id: CMakeLists.txt 4597 2013-08-07 14:18:28Z ruiz $
#

xme_add_component(
    "xme_hal_base64io"
    xme_defines
    xme/hal/base64io_util.c
    xme/hal/base64io_util.h
)

xme_include_directory()
