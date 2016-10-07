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
# $Id: FindFreeGetOpt.cmake 7252 2014-02-11 09:59:11Z geisinger $
#

include (ExternalProject)

if (FREEGETOPT_FOUND)
    return ()
endif ()

ExternalProject_Add(
    freegetopt
    PREFIX "${CMAKE_BINARY_DIR}/freegetopt"
    SOURCE_DIR "${XME_EXTERNAL_DIR}/freegetopt"
    CMAKE_ARGS "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS} -D_CRT_SECURE_NO_WARNINGS"
               "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS"
    INSTALL_COMMAND "" # Disable install step
    LOG_CONFIGURE 1
    LOG_BUILD 1
)

# Set output variables
ExternalProject_Get_Property (freegetopt source_dir)
ExternalProject_Get_Property (freegetopt binary_dir)
set (FREEGETOPT_DIR ${source_dir} CACHE PATH "FreeGetOpt source directory.")
set (FREEGETOPT_INCLUDE_DIR "${FREEGETOPT_DIR}" CACHE PATH "FreeGetOpt include directory.")
if (MSVC)
    set (FREEGETOPT_LIB "freegetoptlib.lib" CACHE STRING "FreeGetOpt linker library filename.")
    set (FREEGETOPT_LIB_DIR "${binary_dir}/$(ConfigurationName)" CACHE PATH "FreeGetOpt linker library directory.")
else ()
    set (FREEGETOPT_LIB "libfreegetoptlib.a" CACHE STRING "FreeGetOpt linker library filename.")
    set (FREEGETOPT_LIB_DIR "${FREEGETOPT_DIR}" CACHE PATH "FreeGetOpt linker library directory.")
endif ()
set (FREEGETOPT_LIB_FILE "${FREEGETOPT_LIB_DIR}/${FREEGETOPT_LIB}" CACHE FILEPATH "FreeGetOpt linker library filepath.")

set (FREEGETOPT_FOUND YES)

message (STATUS "FREEGETOPT_DIR: ${FREEGETOPT_DIR}")
message (STATUS "FREEGETOPT_INCLUDE_DIR: ${FREEGETOPT_INCLUDE_DIR}")
message (STATUS "FREEGETOPT_LIB: ${FREEGETOPT_LIB}")
message (STATUS "FREEGETOPT_LIB_DIR: ${FREEGETOPT_LIB_DIR}")
message (STATUS "FREEGETOPT_LIB_FILE: ${FREEGETOPT_LIB_FILE}")
