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
# $Id: CMakeLists.txt 4254 2013-07-17 13:13:22Z geisinger $
#

# TODO: Refactor such that the folder where this file is contained in
#       is called "pc" or similar and select the operating system to
#       use based on the supplied additional options? (Issue #1894)

# we need that in order to get into the correct subdirectory
if (${XME_TARGET_CPU} STREQUAL "x86")
	xme_add_subdirectory(../../hardware/cpu/x86)
elseif (${XME_TARGET_CPU} STREQUAL "x86_64")
	xme_add_subdirectory(../../hardware/cpu/x86_64)
else (${XME_TARGET_CPU} STREQUAL "x86")
	message(FATAL_ERROR "Do not know how to handle the given target cpu. Given target cpu = ${XME_CONFIG_TARGET_CPU}." )
endif (${XME_TARGET_CPU} STREQUAL "x86")

xme_add_subdirectory(../../hardware/mcu/generic-x86)
xme_add_subdirectory(../../hardware/board/pc)

xme_add_subdirectory(../../software/os/posix)

	
xme_add_subdirectory(../../software/toolchain/gcc)

if(CMAKE_EXTRA_GENERATOR STREQUAL "Eclipse CDT4")
	xme_add_subdirectory(../../software/toolchain/eclipse)
endif(CMAKE_EXTRA_GENERATOR STREQUAL "Eclipse CDT4")