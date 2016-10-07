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
# $Id: CMakeLists.txt 6538 2014-01-29 12:48:51Z geisinger $
#
# File:
#         Main CMake file for "testsuite node".
#

cmake_minimum_required(VERSION 2.6)

project(testsuite)

# Main project settings
set (XME_UNITTEST YES)
set (XME_ENABLE_CODE_EXAMPLE_BUILDS YES)
include ("../CMakeCommon.txt")

# Add definitions

# Add project specific include directories

# Add project specific component directories

# Needed to ensure PRIuXX macros are defined by <inttypes.h>. See Issue #3837.
add_definitions (-D__STDC_FORMAT_MACROS)

# Needed to unify _ITERATOR_DEBUG_LEVEL
if (MSVC)
    add_definitions (-D_ITERATOR_DEBUG_LEVEL=0)
endif ()

# Generate a dummy executable that servers as an
# artificial dependency against all unit tests
set (XME_DUMMY "${CMAKE_CURRENT_BINARY_DIR}/xme_dummy/xme_dummy.c")
file (WRITE ${XME_DUMMY} "// AUTOMATICALLY GENERATED FILE, DO NOT EDIT!\n")
file (APPEND ${XME_DUMMY} "#include <stdio.h>\n")
file (APPEND ${XME_DUMMY} "int main(void) {\n")
file (APPEND ${XME_DUMMY} "    printf(\"This is just an artificial dependency against all\\n\");\n")
file (APPEND ${XME_DUMMY} "    printf(\"unit tests and not supposed to do anything meaningful.\\n\");\n")
file (APPEND ${XME_DUMMY} "    return 0;\n")
file (APPEND ${XME_DUMMY} "}\n")

xme_add_executable(
    "xme_unittest_dummy"
    ${XME_DUMMY} GENERATED
)

# Link all components that have unit tests against 'xme_unittest_dummy'
xme_get (UNITTEST_COMPONENT_LIST PROPERTY_GLOBAL UNITTEST_COMPONENT_LIST)
xme_message(NOTE "All unit tests for the following components will be available:")
foreach (COMPONENT ${UNITTEST_COMPONENT_LIST})
    xme_message(NOTE " * ${COMPONENT}")
endforeach (COMPONENT)
xme_link_components("xme_unittest_dummy" ${UNITTEST_COMPONENT_LIST})

# Build documentation
xme_build_documentation(
    "CHROMOSOME Documentation"
    TARGET "testsuite_doc"
    OUTPUT_NAME "testsuite.chm"
    INSTALL_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    #AUTO
    CLEAN
)

# Execute xme_dummy such that code coverage analysis will be correct
xme_execution_test(
    "xme_unittest_dummy_run"
    EXECUTABLE "xme_unittest_dummy"
    TIMEOUT 5
)

xme_buildsystem_stats()
