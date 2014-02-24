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
# $Id: CMakeLists.txt 4345 2013-07-24 11:00:48Z geisinger $
#
# File:
#         CMake configuration file for XME manifest repository.
#

#------------------------------------------------------------------------------#
#-     xme_core_manifestRepository                                            -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_manifestRepository"
    include/manifestRepository.h
    src/manifestRepository.c
    xme_hal_table
)

xme_unit_test(
    "xme_core_manifestRepository"
    TYPE smoke
    test/smokeTestManifestRepository.cpp
)

xme_unit_test(
    "xme_core_manifestRepository"
    TYPE interface
    test/interfaceTestManifestRepository.cpp
)
