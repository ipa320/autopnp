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
# $Id: CMakeLists.txt 6196 2013-12-19 20:33:34Z geisinger $
#

cmake_minimum_required (VERSION 2.5)

project (freegetoptlib)

add_library(
    "freegetoptlib" STATIC
    getopt.h
    getopt.c
)
