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
# $Id: CMakeLists.txt 5373 2013-10-02 07:58:48Z geisinger $
#
# File:
#         CMake configuration script for GCC compiler toolchain.
#

# Avoid incompatible combinations of build tools
if (CMAKE_GENERATOR MATCHES "^Visual Studio ")
    get_property (_XME_TARGET_IDENTIFIER GLOBAL PROPERTY XME_TARGET_IDENTIFIER)
    xme_message(FATAL_ERROR "'${CMAKE_GENERATOR}' CMake generator is currently not supported in the GCC toolchain! You probably need to select a different XME_TARGET_IDENTIFIER (currently set to '${_XME_TARGET_IDENTIFIER}') or a different CMake generator!")
endif ()

xme_include_directory()

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Pendantic: treat warnings as errors and report (almost) all warnings
# FIXME: how about -pedantic -strict -fstrict-aliasing -Wshadow -Wconversion -Wfloat-equal
# be aware of ansi + c == -std=c89, but ansi + c++ == -std=c++98
if((NOT CMAKE_C_FLAGS) OR (CMAKE_C_FLAGS STREQUAL " ") OR (CMAKE_C_FLAGS STREQUAL CMAKE_C_FLAGS_INIT))
    set (CMAKE_C_FLAGS "-D_GNU_SOURCE -std=c99 -W -Wextra -Wall -Wbad-function-cast -Wcomments -Wunused-macros -Wendif-labels -Wmissing-declarations -Wmissing-prototypes -Wnested-externs -Wold-style-definition -Wstrict-prototypes -Wdeclaration-after-statement -Werror -Wno-missing-field-initializers -fPIC -fstrict-overflow" CACHE STRING "" FORCE)
endif()

if((NOT CMAKE_CXX_FLAGS) OR (CMAKE_CXX_FLAGS STREQUAL " ") OR (CMAKE_CXX_FLAGS STREQUAL CMAKE_CXX_FLAGS_INIT))
    set (CMAKE_CXX_FLAGS "-D_GNU_SOURCE -ansi -W -Wextra -Wall -Wshadow -Wunused-variable -Wunused-parameter -Wunused-function -Wunused -Wno-system-headers -Wno-deprecated -Woverloaded-virtual -Wwrite-strings -Wno-missing-field-initializers -fPIC -fstrict-overflow" CACHE STRING "" FORCE)
endif()

xme_set (PROPERTY_GLOBAL XME_LINKER_FLAGS_MAP_FILE "-Wl,-Map,")

if(NOT CMAKE_C_FLAGS_DEBUG OR (CMAKE_C_FLAGS_DEBUG STREQUAL " ") OR (CMAKE_C_FLAGS_DEBUG STREQUAL CMAKE_C_FLAGS_DEBUG_INIT))
    set (CMAKE_C_FLAGS_DEBUG "-g -ggdb -O0 -pg --coverage -DDEBUG -fprofile-arcs -ftest-coverage" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_FLAGS_DEBUG OR (CMAKE_CXX_FLAGS_DEBUG STREQUAL " ") OR (CMAKE_CXX_FLAGS_DEBUG STREQUAL CMAKE_CXX_FLAGS_DEBUG_INIT))
    set (CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -g3 -gdwarf-2 -O0 -pg --coverage -DDEBUG -fprofile-arcs -ftest-coverage" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_C_FLAGS_RELEASE OR (CMAKE_C_FLAGS_RELEASE STREQUAL " ") OR (CMAKE_C_FLAGS_RELEASE STREQUAL CMAKE_C_FLAGS_RELEASE_INIT))
    set (CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_FLAGS_RELEASE OR (CMAKE_CXX_FLAGS_RELEASE STREQUAL " ") OR (CMAKE_CXX_FLAGS_RELEASE STREQUAL CMAKE_CXX_FLAGS_RELEASE_INIT))
    set (CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_C_FLAGS_MINSIZEREL OR (CMAKE_C_FLAGS_MINSIZEREL STREQUAL " ") OR (CMAKE_C_FLAGS_MINSIZEREL STREQUAL CMAKE_C_FLAGS_MINSIZEREL_INIT))
    set (CMAKE_C_FLAGS_MINSIZEREL "-O2 -Os -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_FLAGS_MINSIZEREL OR (CMAKE_CXX_FLAGS_MINSIZEREL STREQUAL " ") OR (CMAKE_CXX_FLAGS_MINSIZEREL STREQUAL CMAKE_CXX_FLAGS_MINSIZEREL_INIT))
    set (CMAKE_CXX_FLAGS_MINSIZEREL "-O2 -Os -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_C_FLAGS_RELWITHDEBINFO OR (CMAKE_C_FLAGS_RELWITHDEBINFO STREQUAL " ") OR (CMAKE_C_FLAGS_RELWITHDEBINFO STREQUAL CMAKE_C_FLAGS_RELWITHDEBINFO_INIT))
    set (CMAKE_C_FLAGS_RELWITHDEBINFO "-g -ggdb -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

if(NOT CMAKE_CXX_FLAGS_RELWITHDEBINFO OR (CMAKE_CXX_FLAGS_RELWITHDEBINFO STREQUAL " ") OR (CMAKE_CXX_FLAGS_RELWITHDEBINFO STREQUAL CMAKE_CXX_FLAGS_RELWITHDEBINFO_INIT))
    set (CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -g3 -gdwarf-2 -O2 -DNDEBUG -s" CACHE STRING "" FORCE)
endif()

xme_get(_XME_TARGET_CPU PROPERTY_GLOBAL XME_TARGET_CPU)
if (${_XME_TARGET_CPU} STREQUAL "x86")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m32" CACHE STRING "" FORCE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m32" CACHE STRING "" FORCE)
elseif (${_XME_TARGET_CPU} STREQUAL "x86_64")
    # FIXME: WE HAVE TO OMIT -Wconversion because there is a bug in gcc until glibc 2.16
    #        which forces hton to throw a warning of invalid conversion
    #        See: http://sourceware.org/bugzilla/show_bug.cgi?id=12194
    #        See: http://gcc.gnu.org/bugzilla/show_bug.cgi?id=44924

    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -m64" CACHE STRING "" FORCE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64" CACHE STRING "" FORCE)
endif (${_XME_TARGET_CPU} STREQUAL "x86")

if (DEFINED CMAKE_ASM_COMPILER)
    if(NOT CMAKE_ASM_FLAGS)
        set (CMAKE_ASM_FLAGS "-g3 -gdwarf-2" CACHE STRING "" FORCE)
    endif()
    
    if(NOT CMAKE_ASM_FLAGS_DEBUG)
        set (CMAKE_ASM_FLAGS_DEBUG "-g3 -gdwarf-2" CACHE STRING "" FORCE)
    endif()
        
    if(NOT CMAKE_ASM_FLAGS_MINSIZEREL)    
        set (CMAKE_ASM_FLAGS_MINSIZEREL "-g3 -gdwarf-2" CACHE STRING "" FORCE)
    endif()
    
    if(NOT CMAKE_ASM_FLAGS_RELWITHDEBINFO)
        set (CMAKE_ASM_FLAGS_RELWITHDEBINFO "-O2 -g3 -gdwarf-2"  CACHE STRING "" FORCE)
    endif()
endif(DEFINED CMAKE_ASM_COMPILER)
