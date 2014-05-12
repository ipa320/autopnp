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
# $Id: CMakeLists.txt 6443 2014-01-22 17:47:53Z geisinger $
#
# File:
#         CMake configuration file for XME manifest repository.
#

#------------------------------------------------------------------------------#
#-     xme_core_manifestInterchange                                           -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_manifestInterchange"
    include/manifestInterchange.h
    src/manifestInterchange.c
    xme_core_directory_attribute
    xme_hal_mem
    xme_hal_xml
)

xme_unit_test(
    "xme_core_manifestInterchange"
    TYPE smoke
    test/smokeTestManifestInterchange.cpp
)

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
