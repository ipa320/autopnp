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
# $Id: GTest.cmake 6408 2014-01-21 12:32:32Z geisinger $
#
include(CTest)
enable_testing(true)

if (XME_UNITTEST)
    if (NOT IS_DIRECTORY "${XME_EXTERNAL_DIR}/gtest")
        xme_message(FATAL_ERROR "GoogleTest was not found. It is used as testing framework in CHROMOSOME.
In case you want to enable unit testing, please download GoogleTest, extract the content of the archive into the directory '${XME_EXTERNAL_DIR}/gtest' and re-run CMake configuration.
In case you do not want to enable unit testing, make sure the XME_UNITTEST CMake variable is set to a false value (e.g., 'NO', 'OFF', '0') before including 'CMakeCommon.txt'.
")
    endif ()
endif()

include(ExternalProject)

if (XME_UNITTEST)
	# Add gtest
	ExternalProject_Add(googletest
						PREFIX "${CMAKE_BINARY_DIR}/gtest"
						SOURCE_DIR "${XME_EXTERNAL_DIR}/gtest"
						# Force separate output paths for debug and release builds to allow easy
						# identification of correct lib in subsequent TARGET_LINK_LIBRARIES commands
						CMAKE_ARGS "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
								   "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
								   "-DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG}"
								   "-DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG}"
								   "-DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE}"
								   "-DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE}"
								   "-DCMAKE_C_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_C_FLAGS_RELWITHDEBINFO}"
								   "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
								   "-DCMAKE_C_FLAGS_MINSIZEREL:STRING=${CMAKE_C_FLAGS_MINSIZEREL}"
								   "-DCMAKE_CXX_FLAGS_MINSIZEREL:STRING=${CMAKE_CXX_FLAGS_MINSIZEREL}"
								   #-DGTEST_CREATE_SHARED_LIBRARY=1
								   -Dgtest_force_shared_crt=ON
						# Disable install step
						INSTALL_COMMAND ""
						# Wrap configure and build steps in a script to log output
						LOG_CONFIGURE 1
						LOG_BUILD 1
						LOG_TEST 1
						)

	# Specify include dir
	ExternalProject_Get_Property(googletest source_dir)
	ExternalProject_Get_Property(googletest binary_dir)
	set(GTEST_DIR ${source_dir})
	set(GTEST_INCLUDE_DIR "${GTEST_DIR}/include")
	set(GTEST_LIB_DIR ${binary_dir})
	if (MSVC)
		link_directories(${GTEST_LIB_DIR}/$(ConfigurationName)/)
	else (MSVC)
		link_directories(${GTEST_LIB_DIR})
	endif (MSVC)
	include_directories(${GTEST_INCLUDE_DIR})
endif()

# Macro:      xme_unit_test ( <component-name>
#                 TYPE <testsuite-type>
#                 <source1.c> [ <source2.c> ... ]
#                 [ <header1.h> [ <header2.h> ... ] ]
#                 [ <library1> [ <library2> ... ] ]
#                 [ MOCK <mock-component-name> <mock1.c> [ <mock2.c> ... ] [ <mock1.h> [ <mock2.h> ... ] ] ]
#                 [ TIMEOUT <seconds> ] )
#
# Purpose:    Creates a target representing a testsuite. Unit tests are added
#             to the build system if the XME_UNITTEST variable is set to a true
#             value when XME is configured.
#
# Parameters: <component-name>
#                     Name of the component for which the testsuite is defined.
#
#             <testsuite-type>
#                     Type of testsuite. This allows test to be run in a
#                     predefined order. Typical values are 'smoke' (executed
#                     first), 'interface' (executed if 'smoke' tests succeed)
#                     and 'integration' (executed when 'interface' tests
#                     succeed). You may choose an arbitrary testsuite type
#                     value and process it in your build. The testsuite type
#                     is used in the respective target name, so you can filter
#                     by it when running the tests.
#
#             <source1.c> [ <source2.c> ... ]
#                     Source files that contain the testsuite.
#                     At least one source file must be specified.
#
#             [ <header1.h> [ <header2.h> ... ] ]
#                     Optional header files used in the testsuite.
#
#             [ <library1> [ <library2> ... ] ]
#                     Libraries to link against (must be in link directories).
#
#             [ MOCK <mock-component-name> <mock1.c> [ <mock2.c> ... ] [ <mock1.h> [ <mock2.h> ... ] ] ] )
#                     Modify link dependencies such that the symbols from the
#                     given source files are used instead of linking against
#                     XME component <mock-component-name>.
#                     Notice that the current implementation of this concept
#                     relies on the linker to prefer symbols defined in the
#                     given mock source and header files.
#
#             [ TIMEOUT <seconds> ]
#                     Overrides the default timeout (60 seconds) for this
#                     testsuite.
# 
function(xme_unit_test)
    set (__USAGE__ "Usage: xme_unit_test ( <component-name> TYPE <testsuite-type> <source1.c> [ <source2.c> ... ] [ <header1.h> [ <header2.h> ... ] ] [ <library1> [ <library2> ... ] ] [ MOCK <mock-component-name> <mock1.c> [ <mock2.c> ... ] [ <mock1.h> [ <mock2.h> ... ] ] ] [ TIMEOUT <seconds> ] )")

    set (__COMP_NAME__)
    set (_test_source)
    set (_test_header)
    set (_test_lib)
    set (_test_internal_lib)
    set (_test_type)
    set (_test_mocks)
    set (_test_timeout 60) # Default timeout

    include_directories(${GTEST_INCLUDE_DIR})

    xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
    set (__MODE__ 0)
    foreach (ARG ${ARGV})
        if (${ARG} MATCHES "TYPE")
            set (__MODE__ 1)
        elseif (${ARG} MATCHES "MOCK")
            set (__MODE__ 2)
        elseif (${ARG} MATCHES "TIMEOUT")
            set (__MODE__ 3)
        elseif (__MODE__ EQUAL 0)
            set (__COMP_NAME__ ${ARG})
            set (__MODE__ 4)
        elseif (__MODE__ EQUAL 1)
            set (_test_type ${ARG})
            set (__MODE__ 4)
        elseif (__MODE__ EQUAL 2)
            list (APPEND _test_mocks ${ARG})
            set (__MODE__ 4)
        elseif (__MODE__ EQUAL 3)
            set (_test_timeout ${ARG})
            set (__MODE__ 4)
        elseif (__MODE__ EQUAL 4)
            xme_is_header_file ("${ARG}" IS_HEADER)
            xme_is_source_file ("${ARG}" IS_SOURCE)
            xme_is_asm_file ("${ARG}" IS_ASM)

            if (IS_HEADER)
                list(APPEND _test_header "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
            elseif (IS_SOURCE)
                list(APPEND _test_source "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
            elseif (IS_ASM)
                list(APPEND _test_source "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
            else ()
                list(APPEND _test_lib "${ARG}")
            endif ()
        endif()
    endforeach (ARG)

    if ("${__COMP_NAME__}" STREQUAL "")
        xme_message (FATAL_ERROR "Missing component name argument to xme_unit_test() function! ${__USAGE__}")
    endif ()
    set (__COMP_NAME_LOWER__ ${__COMP_NAME__})
    string (TOUPPER ${__COMP_NAME__} __COMP_NAME_UPPER__)

    if ("${_test_type}" STREQUAL "")
        xme_message (FATAL_ERROR "Missing TYPE argument to xme_unit_test() function! ${__USAGE__}")
    endif ()

    list (LENGTH _test_source _test_source_count)
    if (${_test_source_count} LESS 1)
        xme_message (WARNING "Unit test defined for '${__COMP_NAME_LOWER__}' (type '${_test_type}') with no source files!")
    endif (${_test_source_count} LESS 1)

    list(APPEND _test_internal_lib gtest)

    if (MINGW)
    elseif (CYGWIN)
    elseif (UNIX)
        list(APPEND _test_internal_lib pthread)
    endif (MINGW)

    if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
        if (MINGW)
            list (APPEND _test_internal_lib gcov)
        elseif (CYGWIN)
            list (APPEND _test_internal_lib gcov)
        elseif (UNIX)
            list (APPEND _test_internal_lib gcov)
        endif (MINGW)
    endif(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)

    xme_get (_UNITTEST_TYPE_LIST PROPERTY_GLOBAL "UNITTEST_TYPE_LIST")
    list (FIND _UNITTEST_TYPE_LIST "${_test_type}" _FOUND)
    if (_FOUND LESS 0)
        xme_append (PROPERTY_GLOBAL "UNITTEST_TYPE_LIST" "${_test_type}")
    endif ()

    xme_get (_UNITTEST_COMPONENT_LIST PROPERTY_GLOBAL "UNITTEST_COMPONENT_LIST")
    list (FIND _UNITTEST_COMPONENT_LIST "${__COMP_NAME__}" _FOUND)
    if (_FOUND LESS 0)
        xme_append (PROPERTY_GLOBAL "UNITTEST_COMPONENT_LIST" "${__COMP_NAME__}")
    endif ()

    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_HEADERS" "${_test_header}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_SOURCES" "${_test_source}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_LINKS" "${_test_lib}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_INTERNAL_LINKS" "${_test_internal_lib}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_MOCKS" "${_test_mocks}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_${_test_type}_UNITTEST_TIMEOUT" "${_test_timeout}")
endfunction(xme_unit_test)

# a phony target which causes all available *_tests targets to be executed
#add_custom_target(ALL_TESTS)

# FIXME: gmock is currently disabled, because it is not used
if (FALSE AND XME_UNITTEST)
	# Add gmock
	ExternalProject_Add(googlemock
						PREFIX "${CMAKE_BINARY_DIR}/gmock"
						SOURCE_DIR "${XME_EXTERNAL_DIR}/gmock"
						CMAKE_ARGS "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
								   "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
								   "-DCMAKE_C_FLAGS_DEBUG:STRING=${CMAKE_C_FLAGS_DEBUG}"
								   "-DCMAKE_CXX_FLAGS_DEBUG:STRING=${CMAKE_CXX_FLAGS_DEBUG}"
								   "-DCMAKE_C_FLAGS_RELEASE:STRING=${CMAKE_C_FLAGS_RELEASE}"
								   "-DCMAKE_CXX_FLAGS_RELEASE:STRING=${CMAKE_CXX_FLAGS_RELEASE}"
								   "-DCMAKE_C_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_C_FLAGS_RELWITHDEBINFO}"
								   "-DCMAKE_CXX_FLAGS_RELWITHDEBINFO:STRING=${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
								   "-DCMAKE_C_FLAGS_MINSIZEREL:STRING=${CMAKE_C_FLAGS_MINSIZEREL}"
								   "-DCMAKE_CXX_FLAGS_MINSIZEREL:STRING=${CMAKE_CXX_FLAGS_MINSIZEREL}"
									#-DGMOCK_CREATE_SHARED_LIBRARY=1
									-Dgmock_force_shared_crt=ON
						# Disable install step
						INSTALL_COMMAND ""
						# Wrap configure and build steps in a script to log output
						LOG_CONFIGURE 1
						LOG_BUILD 1
						LOG_TEST 1
						)

	# Specify include dir
	ExternalProject_Get_Property(googlemock source_dir)
	ExternalProject_Get_Property(googlemock binary_dir)
	set(GMOCK_DIR ${source_dir})
	set(GMOCK_INCLUDE_DIR "${GMOCK_DIR}/include" )
	set(GMOCK_LIB_DIR ${binary_dir})
	if (MSVC)
		link_directories(${GMOCK_LIB_DIR}/$(ConfigurationName)/)
	else (MSVC)
		link_directories(${GMOCK_LIB_DIR})
	endif (MSVC)
	include_directories(${GMOCK_INCLUDE_DIR})
endif()
