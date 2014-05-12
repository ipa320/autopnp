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
# $Id: CMakeLists.txt 5641 2013-10-25 14:45:25Z geisinger $
#
# File:
#         CMake configuration file for XME HAL documentation.
#

#------------------------------------------------------------------------------#
#-     xme_hal_linkedList                                                     -#
#------------------------------------------------------------------------------#

xme_code_example(
    "xme_hal_linkedList"
    EXAMPLE "addItemOrdered"
    FILES linkedList_addItemOrdered.c
    BUILD
    xme_core_log
    xme_hal_linkedList
)
