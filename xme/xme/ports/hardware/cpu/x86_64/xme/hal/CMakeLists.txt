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
# $Id: CMakeLists.txt 3345 2013-05-17 12:07:58Z geisinger $
#

xme_generic_port_path (_GEN_PATH_ "gen_c")

xme_add_component(
	"xme_hal_math"
	${_GEN_PATH_}/math_arch.c ${_GEN_PATH_}/math_arch.h
)
