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
# $Id: FindExpat.cmake 7727 2014-03-10 10:46:10Z geisinger $
#

include(ExternalProject)

if (EXPAT_FOUND)
    return ()
endif ()

set(EXPAT_INSTALL_DIR "${CMAKE_BINARY_DIR}/expat.install")

if (MSVC)
    set (COMPILER_DEFINE_ARG "/D")
else ()
    set (COMPILER_DEFINE_ARG "-D")
endif ()

ExternalProject_Add(
    expat
    PREFIX "${CMAKE_BINARY_DIR}/expat"
    SOURCE_DIR "${XME_EXTERNAL_DIR}/expat"  
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
               "-DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG} ${COMPILER_DEFINE_ARG}XML_STATIC"
               "-DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG} ${COMPILER_DEFINE_ARG}XML_STATIC"
               "-DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE} ${COMPILER_DEFINE_ARG}XML_STATIC"
               "-DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE} ${COMPILER_DEFINE_ARG}XML_STATIC"
               "-DCMAKE_INSTALL_PREFIX=${EXPAT_INSTALL_DIR}"
               "-DBUILD_shared:BOOL=OFF"
               "-DBUILD_examples:BOOL=OFF" # Remove this to build the examples
               "-DBUILD_tests:BOOL=OFF" # Remove this to build the tests
               "-DBUILD_tools:BOOL=OFF" # Remove this to build the xmlwf tool, which is for testing whether an XML document is well-formed
    # Wrap configure and build steps in a script to log output
    LOG_CONFIGURE 1
    LOG_BUILD 1
    LOG_INSTALL 1
    LOG_TEST 1
)

# Set output variables
ExternalProject_Get_Property (expat source_dir)
ExternalProject_Get_Property (expat binary_dir)
set (EXPAT_DIR ${source_dir} CACHE PATH "Expat source directory.")
set (EXPAT_INCLUDE_DIR "${EXPAT_INSTALL_DIR}/include" CACHE PATH "Expat include directory.")
set (EXPAT_BIN_DIR "${EXPAT_INSTALL_DIR}/bin" CACHE PATH "Expat bin directory.")
set (EXPAT_LIB_DIR "${EXPAT_INSTALL_DIR}/lib" CACHE PATH "Expat linker library directory.")
if (MSVC)
    set (EXPAT_LIB_STATIC "expat.lib" CACHE STRING "Expat linker library filename (static).")
    #set (EXPAT_LIB_DYNAMIC "expat.dll" CACHE STRING "Expat linker library filename (dynamic).")
    #set (EXPAT_LIB_FILE_DYNAMIC "${EXPAT_BIN_DIR}/${EXPAT_LIB_DYNAMIC}" CACHE FILEPATH "Expat linker library filepath (dynamic).")
    #set (EXPAT_LIB_IMPORT "expat.lib" CACHE STRING "Expat linker library filename (implib).")
else ()
    set (EXPAT_LIB_STATIC "libexpat.a" CACHE STRING "Expat linker library filename (static).")
    #set (EXPAT_LIB_DYNAMIC "libexpat.so" CACHE STRING "Expat linker library filename (dynamic).")
    #set (EXPAT_LIB_FILE_DYNAMIC "${EXPAT_BIN_DIR}/${EXPAT_LIB_DYNAMIC}" CACHE FILEPATH "Expat linker library filepath (dynamic).")
    #set (EXPAT_LIB_IMPORT "" CACHE STRING "Expat linker library filename (implib).")
endif ()
set (EXPAT_LIB_FILE_STATIC "${EXPAT_LIB_DIR}/${EXPAT_LIB_STATIC}" CACHE FILEPATH "Expat linker library filepath (static).")

#if (EXPAT_LIB_IMPORT STREQUAL "")
#    set (EXPAT_LIB_FILE_IMPORT "" CACHE FILEPATH "Expat linker library filepath (implib).")
#else ()
#    set (EXPAT_LIB_FILE_IMPORT "${EXPAT_LIB_DIR}/${EXPAT_LIB_IMPORT}" CACHE FILEPATH "Expat linker library filepath (implib).")
#endif ()

set (EXPAT_FOUND YES)

#message (STATUS "EXPAT_DIR: ${EXPAT_DIR}")
#message (STATUS "EXPAT_INCLUDE_DIR: ${EXPAT_INCLUDE_DIR}")
#message (STATUS "EXPAT_BIN_DIR: ${EXPAT_BIN_DIR}")
#message (STATUS "EXPAT_LIB_DIR: ${EXPAT_LIB_DIR}")
#message (STATUS "EXPAT_LIB_STATIC: ${EXPAT_LIB_STATIC}")
##message (STATUS "EXPAT_LIB_DYNAMIC: ${EXPAT_LIB_DYNAMIC}")
##message (STATUS "EXPAT_LIB_IMPORT: ${EXPAT_LIB_IMPORT}")
#message (STATUS "EXPAT_LIB_FILE_STATIC: ${EXPAT_LIB_FILE_STATIC}")
##message (STATUS "EXPAT_LIB_FILE_DYNAMIC: ${EXPAT_LIB_FILE_DYNAMIC}")
##message (STATUS "EXPAT_LIB_FILE_IMPORT: ${EXPAT_LIB_FILE_IMPORT}")
