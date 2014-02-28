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
# $Id: CMakeLists.txt 4595 2013-08-07 13:49:46Z ruiz $
#
# File:
#         XME CMake configuration script.
#         However, this is not the CMake source directory!
#         See below.
#

# Note: Do *not* use this directory as CMake source directory! Rather use one
#       of the directories from within the examples directory. CHROMOSOME is
#       designed such that the firmware for each node is to be built using a
#       separate build system. Consult the documentation for details.

cmake_minimum_required(VERSION 2.6)

# Make this directory appear as a subproject in the project explorer of eclipse
project (xme C)

if (NOT XME_FOUND)
    message (FATAL_ERROR "CHROMOSOME does not seem to be properly configured. Please ensure that you are using one of the projects from within the examples directory as CMake source directory and that the CMakeLists.txt file contained there directly or indirectly calls the find_package(XME REQUIRED) command!\nThe following settings are recommended for your configuration (NOTE: as soon as you paste one of these paths into CMake GUI, this text might vanish!):\nCMake source directory (\"Where is the source code\"): ${CMAKE_CURRENT_SOURCE_DIR}/../examples/your-project-name\nCMake build directory (\"Where to build the binaries\"): ${CMAKE_CURRENT_SOURCE_DIR}/../examples/build/your-project-name")
endif (NOT XME_FOUND)

xme_include_directory (..)

xme_add_component(
    "xme_defines"
    defines.h
    defines.c
    xme_core_core
)

xme_add_subdirectory (adv)
xme_add_subdirectory (core)
xme_add_subdirectory (hal)
xme_add_subdirectory (prim)
xme_add_subdirectory (wp)
xme_add_subdirectory (com)
