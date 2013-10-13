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
# $Id: FindGraphviz.cmake 4254 2013-07-17 13:13:22Z geisinger $
#

if (CMAKE_HOST_WIN32)
    # Always prefer the version that is in the PATH
    find_path(
        GRAPHVIZ_BINARY_PATH
        NAMES "dot.exe"
        ENV PATH
        DOC "Graphviz binary path"
    )
    if (NOT GRAPHVIZ_BINARY_PATH)
        file (GLOB __GRAPHVIZ_DIRS__ "$ENV{PROGRAMFILES}/Graphviz*")
        foreach (__GRAPHVIZ_DIR__ ${__GRAPHVIZ_DIRS__})
            if (IS_DIRECTORY ${__GRAPHVIZ_DIR__})
                find_path(
                    GRAPHVIZ_BINARY_PATH
                    NAMES "dot.exe"
                    PATHS "${__GRAPHVIZ_DIR__}"
                    PATH_SUFFIXES "bin"
                    DOC "Graphviz binary path"
                )
            endif (IS_DIRECTORY ${__GRAPHVIZ_DIR__})
        endforeach (__GRAPHVIZ_DIR__)
    endif ()
else ()
    find_path(
        GRAPHVIZ_BINARY_PATH
        NAMES "dot"
        PATHS "/usr/local/bin" "/usr/bin"
        DOC "Graphviz binary path"
    )
endif ()

find_program(
    GRAPHVIZ_BINARY
    NAMES "dot" "dot.exe"
    HINTS ${GRAPHVIZ_BINARY_PATH}
    DOC "Graphviz binary"
    NO_DEFAULT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

if (GRAPHVIZ_BINARY)
    set (GRAPHVIZ_FOUND TRUE)
else ()
    set (GRAPHVIZ_FOUND FALSE)
endif ()
