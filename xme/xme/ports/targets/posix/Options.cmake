#
# Copyright (c) 2011-2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: Options.cmake 6348 2014-01-15 15:49:59Z geisinger $
#

# Check if XME_CONFIG_TARGET_CPU has been set. This is useful in order to select a
# specific processor type on the command line using cmake -DXME_CONFIG_TARGET_CPU=<...>. 
# N.B.: -DXME_CONFIG_TARGET_CPU defines a CMake variable, which is evaluated here in 
#       order to set a property of the same name.

# Global property XME_CONFIG_TARGET_CPU
xme_defined (XME_CONFIG_TARGET_CPU_DEFINED PROPERTY_GLOBAL XME_CONFIG_TARGET_CPU)
if(${XME_CONFIG_TARGET_CPU_DEFINED})
	xme_get (XME_CONFIG_TARGET_CPU PROPERTY_GLOBAL XME_CONFIG_TARGET_CPU)
else (${XME_CONFIG_TARGET_CPU_DEFINED})
	xme_defined(XME_TARGET_CPU_DEFINED PROPERTY_GLOBAL XME_TARGET_CPU)
	if(${XME_TARGET_CPU_DEFINED})
		xme_get (XME_CONFIG_TARGET_CPU PROPERTY_GLOBAL XME_TARGET_CPU)
	else (${XME_TARGET_CPU_DEFINED})
		if(CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(XME_CONFIG_TARGET_CPU "x86_64")
		else(CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(XME_CONFIG_TARGET_CPU "x86")
		endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
		message (STATUS "XME_CONFIG_TARGET_CPU not defined, setting to host CPU architecture '${XME_CONFIG_TARGET_CPU}'.")
	endif (${XME_TARGET_CPU_DEFINED})
endif (${XME_CONFIG_TARGET_CPU_DEFINED})

set(XME_TARGET_CPU ${XME_CONFIG_TARGET_CPU} PARENT_SCOPE)
set(_XME_TARGET_CPU ${XME_CONFIG_TARGET_CPU} PARENT_SCOPE)
xme_set (PROPERTY_GLOBAL XME_TARGET_CPU ${XME_CONFIG_TARGET_CPU})
message (STATUS "XME_CONFIG_TARGET_CPU: '${XME_CONFIG_TARGET_CPU}'.")

# we need that otherwise we will always change our xme_target_cpu later on to x86 even if we have overwritten it outside
if (${XME_CONFIG_TARGET_CPU} STREQUAL "x86")
	xme_add_subdirectory(../../hardware/cpu/x86 FALSE "Options.cmake")
elseif (${XME_CONFIG_TARGET_CPU} STREQUAL "x86_64")
	xme_add_subdirectory(../../hardware/cpu/x86_64 FALSE "Options.cmake")
endif (${XME_CONFIG_TARGET_CPU} STREQUAL "x86")

xme_add_subdirectory(../../hardware/mcu/generic-x86 FALSE "Options.cmake")
xme_add_subdirectory(../../hardware/board/pc FALSE "Options.cmake")

xme_add_subdirectory(../../software/os/posix FALSE "Options.cmake")
