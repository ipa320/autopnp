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
# $Id: CMakeLists.txt 7726 2014-03-10 10:45:38Z geisinger $
#
# File:
#         CMake configuration script for Cygwin version of XME HAL components.
#


# Retrieve relative path to generic OS-based implementation
xme_generic_port_path (_POSIX_PATH "posix")

# Require Pthreads package for scheduler implementation
find_package(Threads REQUIRED)

# Check whether linking against threading library is required
set (_LINK_THREADS)
if (CMAKE_THREAD_LIBS_INIT)
	set (_LINK_THREADS "LINK" ${CMAKE_THREAD_LIBS_INIT})
endif (CMAKE_THREAD_LIBS_INIT)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/circularBuffer/dynamicMemory)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/cmdLine/freegetopt)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/context/basedOnTls)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/dio/generic)

xme_add_component(
	"xme_hal_energy"
	${_POSIX_PATH}/energy_arch.c
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/linkedList/dynamicMemory)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/net/winPosix)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/toolchain/stdlib/random)

xme_add_component(
	"xme_hal_safeString"
	${_POSIX_PATH}/safeString_arch.h
)

xme_add_component(
	"xme_hal_sched"
	${_POSIX_PATH}/sched_arch.c
	"xme_hal_random"
	"xme_hal_tls"
	LINK rt
	${_LINK_THREADS}
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/toolchain/stdlib/mem)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/sharedPtr/dynamicMemory)

xme_add_component(
	"xme_hal_sleep"
	${_POSIX_PATH}/sleep_arch.c
)

xme_add_component(
	"xme_hal_sync"
	${_POSIX_PATH}/sync_arch.c
	"xme_hal_table"
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/table/dynamicMemory)

xme_add_component(
	"xme_hal_table"
	LINK m
)

xme_add_component(
	"xme_hal_time"
	${_POSIX_PATH}/time_arch.h ${_POSIX_PATH}/time_arch.c
	${_LINK_THREADS}
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/tls/multiThread)

xme_add_component(
	"xme_hal_tls"
	${_POSIX_PATH}/tls_arch.c
	${_LINK_THREADS}
)

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/xml/tinyxml2)
#xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/xml/expat)
