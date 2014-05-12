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
# $Id: FindHTMLHelpWorkshop.cmake 4254 2013-07-17 13:13:22Z geisinger $
#

if (CMAKE_HOST_WIN32)
    # Always prefer the version that is in the PATH
    find_path(
        HTMLHELPWORKSHOP_BINARY_PATH
        NAMES "hhc.exe"
        ENV PATH
        DOC "HTML Help Workshop binary path"
    )
    if (NOT HTMLHELPWORKSHOP_BINARY_PATH)
        file (GLOB __HTMLHELPWORKSHOP_DIRS__ "$ENV{PROGRAMFILES}/HTML Help Workshop*")
        foreach (__HTMLHELPWORKSHOP_DIR__ ${__HTMLHELPWORKSHOP_DIRS__})
            if (IS_DIRECTORY ${__HTMLHELPWORKSHOP_DIR__})
                find_path(
                    HTMLHELPWORKSHOP_BINARY_PATH
                    NAMES "hhc.exe"
                    PATHS "${__HTMLHELPWORKSHOP_DIR__}"
                    DOC "HTML Help Workshop binary path"
                )
            endif (IS_DIRECTORY ${__HTMLHELPWORKSHOP_DIR__})
        endforeach (__HTMLHELPWORKSHOP_DIR__)
    endif ()
    
    find_program(
        HTMLHELPWORKSHOP_BINARY
        NAMES "hhc.exe"
        HINTS ${HTMLHELPWORKSHOP_BINARY_PATH}
        DOC "HTML Help Workshop binary"
        NO_DEFAULT_PATH
        NO_CMAKE_ENVIRONMENT_PATH
        NO_CMAKE_PATH
        NO_SYSTEM_ENVIRONMENT_PATH
        NO_CMAKE_SYSTEM_PATH
    )
else (CMAKE_HOST_WIN32)
    set (HTMLHELPWORKSHOP_BINARY_PATH "HTMLHELPWORKSHOP_BINARY_PATH-NOTFOUND" CACHE FILEPATH "HTML Help Workshop binary path")
    set (HTMLHELPWORKSHOP_BINARY "HTMLHELPWORKSHOP_BINARY_PATH-NOTFOUND" CACHE FILEPATH "HTML Help Workshop binary")
endif (CMAKE_HOST_WIN32)

if (HTMLHELPWORKSHOP_BINARY_PATH)
    set (HTMLHELPWORKSHOP_FOUND TRUE)
else ()
    set (HTMLHELPWORKSHOP_FOUND FALSE)
endif ()
