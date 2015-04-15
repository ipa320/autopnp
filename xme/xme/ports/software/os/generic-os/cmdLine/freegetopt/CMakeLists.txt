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
# $Id: CMakeLists.txt 7678 2014-03-05 14:06:01Z geisinger $
#

xme_add_component(
    "xme_hal_cmdLine"
    xme/hal/cmdLine_arch.c
    xme_defines
    PACKAGES
        FreeGetOpt
    TARGETS
        freegetopt
    INCLUDE_PATH
        VALUE_OF FREEGETOPT_INCLUDE_DIR
    LINK
        VALUE_OF FREEGETOPT_LIB_FILE
)

xme_include_directory()
