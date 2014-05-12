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
# $Id: FindExpect.cmake 4254 2013-07-17 13:13:22Z geisinger $
#

find_path(
    EXPECT_BINARY_PATH
    NAMES "expect" "expect.exe"
    PATHS "/usr/local" "/usr" "C:/cygwin"
    PATH_SUFFIXES "bin" "sbin"
    DOC "Expect binary path"
)

if (EXPECT_BINARY_PATH)
    find_program(
        EXPECT_BINARY
        NAMES "expect" "expect.exe"
        HINTS ${EXPECT_BINARY_PATH}
        DOC "Expect binary"
        NO_DEFAULT_PATH
        NO_CMAKE_ENVIRONMENT_PATH
        NO_CMAKE_PATH
        NO_SYSTEM_ENVIRONMENT_PATH
        NO_CMAKE_SYSTEM_PATH
    )
endif ()

if (EXPECT_BINARY)
    set (EXPECT_FOUND YES)
    message (STATUS "Expect found in \"${EXPECT_BINARY_PATH}\"")
else ()
    set (EXPECT_FOUND NO)
    message (STATUS "Expect not found!")
endif ()
