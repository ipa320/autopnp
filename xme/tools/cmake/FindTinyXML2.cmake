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
# $Id: FindTinyXML2.cmake 7725 2014-03-10 10:38:19Z geisinger $
#

include(ExternalProject)

if (TINYXML2_FOUND)
    return ()
endif ()

set(TINYXML2_INSTALL_DIR "${CMAKE_BINARY_DIR}/tinyxml2.install")

if (MSVC)
    set (COMPILER_DEFINE_ARG "/D")
else ()
    set (COMPILER_DEFINE_ARG "-D")
endif ()

ExternalProject_Add(
    tinyxml2
    PREFIX "${CMAKE_BINARY_DIR}/tinyxml2"
    SOURCE_DIR "${XME_EXTERNAL_DIR}/tinyxml2"
    CMAKE_ARGS "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
               "-DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG} ${COMPILER_DEFINE_ARG}_ITERATOR_DEBUG_LEVEL=0"
               "-DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG} ${COMPILER_DEFINE_ARG}_ITERATOR_DEBUG_LEVEL=0"
               "-DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE} ${COMPILER_DEFINE_ARG}_ITERATOR_DEBUG_LEVEL=0"
               "-DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE} ${COMPILER_DEFINE_ARG}_ITERATOR_DEBUG_LEVEL=0"
               "-DCMAKE_INSTALL_BINDIR:STRING=${TINYXML2_INSTALL_DIR}"
               "-DCMAKE_INSTALL_LIBDIR:STRING=${TINYXML2_INSTALL_DIR}"
               "-DCMAKE_INSTALL_INCLUDEDIR:STRING=${TINYXML2_INSTALL_DIR}"
               "-DBUILD_STATIC_LIBS:BOOL=ON"
    # Wrap configure and build steps in a script to log output
    LOG_CONFIGURE 1
    LOG_BUILD 1
    LOG_INSTALL 1
    LOG_TEST 1
)

# Set output variables
ExternalProject_Get_Property (tinyxml2 source_dir)
ExternalProject_Get_Property (tinyxml2 binary_dir)
set (TINYXML2_DIR ${source_dir} CACHE PATH "TinyXML2 source directory.")
set (TINYXML2_INCLUDE_DIR "${TINYXML2_INSTALL_DIR}" CACHE PATH "TinyXML2 include directory.")
if (MSVC)
    set (TINYXML2_LIB_STATIC "tinyxml2static.lib" CACHE STRING "TinyXML2 linker library filename (static).")
    set (TINYXML2_LIB_DYNAMIC "tinyxml2.dll" CACHE STRING "TinyXML2 linker library filename (dynamic).")
    set (TINYXML2_LIB_IMPORT "tinyxml2.lib" CACHE STRING "TinyXML2 linker library filename (implib).")
    set (TINYXML2_LIB_DIR "${TINYXML2_INSTALL_DIR}" CACHE PATH "TinyXML2 linker library directory.")
else ()
    set (TINYXML2_LIB_STATIC "libtinyxml2static.a" CACHE STRING "TinyXML2 linker library filename (static).")
    set (TINYXML2_LIB_DYNAMIC "libtinyxml2.so" CACHE STRING "TinyXML2 linker library filename (dynamic).")
    set (TINYXML2_LIB_IMPORT "" CACHE STRING "TinyXML2 linker library filename (implib).")
    set (TINYXML2_LIB_DIR "${TINYXML2_INSTALL_DIR}" CACHE PATH "TinyXML2 linker library directory.")
endif ()
set (TINYXML2_LIB_FILE_STATIC "${TINYXML2_LIB_DIR}/${TINYXML2_LIB_STATIC}" CACHE FILEPATH "TinyXML2 linker library filepath (static).")
set (TINYXML2_LIB_FILE_DYNAMIC "${TINYXML2_LIB_DIR}/${TINYXML2_LIB_DYNAMIC}" CACHE FILEPATH "TinyXML2 linker library filepath (dynamic).")
if (TINYXML2_LIB_IMPORT STREQUAL "")
    set (TINYXML2_LIB_FILE_IMPORT "" CACHE FILEPATH "TinyXML2 linker library filepath (implib).")
else ()
    set (TINYXML2_LIB_FILE_IMPORT "${TINYXML2_LIB_DIR}/${TINYXML2_LIB_IMPORT}" CACHE FILEPATH "TinyXML2 linker library filepath (implib).")
endif ()

set (TINYXML2_FOUND YES)

#message (STATUS "TINYXML2_DIR: ${TINYXML2_DIR}")
#message (STATUS "TINYXML2_INCLUDE_DIR: ${TINYXML2_INCLUDE_DIR}")
#message (STATUS "TINYXML2_LIB_STATIC: ${TINYXML2_LIB_STATIC}")
#message (STATUS "TINYXML2_LIB_DYNAMIC: ${TINYXML2_LIB_DYNAMIC}")
#message (STATUS "TINYXML2_LIB_IMPORT: ${TINYXML2_LIB_IMPORT}")
#message (STATUS "TINYXML2_LIB_DIR: ${TINYXML2_LIB_DIR}")
#message (STATUS "TINYXML2_LIB_FILE_STATIC: ${TINYXML2_LIB_FILE_STATIC}")
#message (STATUS "TINYXML2_LIB_FILE_DYNAMIC: ${TINYXML2_LIB_FILE_DYNAMIC}")
#message (STATUS "TINYXML2_LIB_FILE_IMPORT: ${TINYXML2_LIB_FILE_IMPORT}")
