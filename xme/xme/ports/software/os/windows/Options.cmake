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
# $Id: Options.cmake 3345 2013-05-17 12:07:58Z geisinger $
#
# File:
#         CMake options for Windows platform port.
#

xme_set (PROPERTY_GLOBAL XME_TARGET_OS "windows")

# Custom build options for this platform
xme_get(_XME_TARGET_CPU PROPERTY_GLOBAL XME_TARGET_CPU)

# _WIN32 defines always need to be defined (also for 64 bit builds).
# Citing MSDN: "_WIN32: Defined for applications for Win32 and Win64. Always defined."
xme_build_option_check (_WIN32 1 "xme/xme_opt.h" "Ensure that we are compiling for Windows")
xme_build_option (WIN32 1 "xme/xme_opt.h" "Compiling for Windows")
xme_build_option (__WIN32__ 1 "xme/xme_opt.h" "Compiling for Windows")

# CYGWIN must not be defined if the target platform is native Windows
xme_build_option_check (CYGWIN UNDEFINED "xme/xme_opt.h" "Ensure that we are not building for CYGWIN")

if (${_XME_TARGET_CPU} STREQUAL "x86_64")
	xme_build_option_check (_WIN64 1 "xme/xme_opt.h" "Ensure that we are compiling for Windows 64bit")
	xme_build_option (WIN64 1 "xme/xme_opt.h" "Compiling for Windows 64bit")
	xme_build_option (__WIN64__ 1 "xme/xme_opt.h" "Compiling for Windows 64bit")
endif (${_XME_TARGET_CPU} STREQUAL "x86_64")
