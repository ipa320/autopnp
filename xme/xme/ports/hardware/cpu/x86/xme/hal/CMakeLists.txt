#
# Copyright (c) 2011-2012, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
# 
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 4595 2013-08-07 13:49:46Z ruiz $
#

xme_generic_port_path (_GEN_PATH_ "gen_c")

xme_add_component(
    "xme_hal_math"
    math_arch.c math_arch.h
)

#if you want to use posix, activate this component
#xme_add_component(
#    "xme_hal_math"
#    ${_GEN_PATH_}/math_arch.c ${_GEN_PATH_}/math_arch.h
#)
