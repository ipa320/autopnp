# Copyright (c) 2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: autoDetect.cmake 6257 2014-01-07 17:41:22Z geisinger $
#

# This CMake script is executed in case no target identifier is specified
# using the XME_TARGET_IDENTIFIER CMake variable when the XME package is
# loaded for the first time. Its purpose is to "guess" the name of the
# current target platform and set that variable to the respective name,
# which should be the name of one of the subdirectories in the directory
# where this file is located. The target platform is typically detected
# depending on CMake variables such as (see also CMake documentation):
#  - CMAKE_SYSTEM, CMAKE_SYSTEM_NAME,
#    CMAKE_SYSTEM_PROCESSOR, CMAKE_SYSTEM_VERSION
#  - CMAKE_HOST_SYSTEM, CMAKE_HOST_SYSTEM_NAME,
#    CMAKE_HOST_SYSTEM_PROCESSOR, CMAKE_HOST_SYSTEM_VERSION,
#    CMAKE_HOST_APPLE, CMAKE_HOST_UNIX, CMAKE_HOST_WIN32
#  - CMAKE_GENERATOR
#  - CMAKE_CROSSCOMPILING
#  - CMAKE_SIZEOF_VOID_P
#  - APPLE, CYGWIN, UNIX, WIN32
#  - BORLAND, CMAKE_CL_64, CMAKE_COMPILER_2005, XCODE_VERSION
#  - MSVC, MSVC10, MSVC11, MSVC12, MSVC60, MSVC70, MSVC71, MSVC80, MSVC90,
#    MSVC_IDE, MSVC_VERSION
#  - ENV

if (${CMAKE_SYSTEM_NAME} STREQUAL "Windows")
    set(XME_TARGET_IDENTIFIER "windows")
elseif (${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(XME_TARGET_IDENTIFIER "posix")
else ()
    string (TOLOWER ${CMAKE_SYSTEM_NAME} XME_TARGET_IDENTIFIER)
endif ()
