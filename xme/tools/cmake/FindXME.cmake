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
# $Id: FindXME.cmake 6578 2014-01-31 10:44:02Z geisinger $
#

# This CMake script detects the CHROMOSOME environment and defines some
# general purpose macros for use with CHROMOSOME. For details on the
# defined macros, see the documentation directly above the respective
# definition below.

# 2.8.3: CMAKE_CURRENT_LIST_DIR
# 2.8.5:
#  - string (FIND)
#  - Linked resources for Eclipse CDT projects.
#    See: <http://www.cmake.org/pipermail/cmake/2011-February/042703.html>
#
cmake_minimum_required (VERSION 2.8.5)

if (XME_CONFIGURED)
	message (FATAL_ERROR "XME seems to be already configured. Please make sure that find_package(XME) is only run once. You may use the XME_CONFIGURED variable to detect whether XME was already configured.")
endif ()

#
# Buildsystem self-tests:
# CHROMOSOME extends CMake by a large set of macros and functions.
# It also comes with a built-in testsuite for some of these elements.
#

# Cache entry:  XME_ENABLE_BUILDSYSTEM_SELFTEST
# Used by:      XME build system
# Purpose:      Controls whether self-tests of the XME CMake buildsystem are
#               enabled or not.
# Usage:        Buildsystem self-tests are enabled by default. To disable, define
#               a CMake cache entry with the name XME_ENABLE_BUILDSYSTEM_SELFTEST
#               and set it to a value that is considered FALSE by CMake (e.g.,
#               "0", "FALSE", "OFF").
# Command line: cmake -G "..." -DXME_ENABLE_BUILDSYSTEM_SELFTEST=0
# CMake GUI:    Add Entry -> XME_ENABLE_BUILDSYSTEM_SELFTEST, BOOL, OFF
#
if (NOT DEFINED XME_ENABLE_BUILDSYSTEM_SELFTEST)
	set (XME_ENABLE_BUILDSYSTEM_SELFTEST "TRUE")
endif ()
set (XME_ENABLE_BUILDSYSTEM_SELFTEST "${XME_ENABLE_BUILDSYSTEM_SELFTEST}" CACHE BOOL "Whether XME CMake buildsystem self-tests are enabled or not.")

# Set CMAKE_INSTALL_PREFIX (all install rules will install their files relative to this path)
set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")

# Setup output directories
if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/target	CACHE PATH "Target directory for all libraries")
endif()

if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/target	CACHE PATH "Target directory for all executables")
endif()

if(NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/target	CACHE PATH "Target directory for all static libraries")
endif()

# Include XME utility scripts
include(XMEUtils)
include(XMESystemTest)

# Set root dir: CACHE Variable XME_ROOT
if (NOT DEFINED XME_ROOT)
	set (XME_ROOT "${CMAKE_CURRENT_LIST_DIR}/../..")
endif (NOT DEFINED XME_ROOT)
xme_simplify_path (XME_ROOT "${XME_ROOT}")
set (XME_ROOT "${XME_ROOT}" CACHE PATH "CHROMOSOME root directory")

# Set examples dir (CHROMOSOME examples): CACHE Variable XME_EXAMPLES_DIR (via find_file)
if (NOT DEFINED XME_EXAMPLES_DIR)
	find_file (XME_EXAMPLES_DIR "examples" PATHS "${XME_ROOT}" DOC "CHROMOSOME examples directory" NO_DEFAULT_PATH)
endif (NOT DEFINED XME_EXAMPLES_DIR)

# Set external dir (adapted vendor sources for CHROMOSOME): CACHE Variable XME_EXTERNAL_DIR (via find_file)
if (NOT DEFINED XME_EXTERNAL_DIR)
	find_file (XME_EXTERNAL_DIR "external" PATHS "${XME_ROOT}" DOC "CHROMOSOME external directory" NO_DEFAULT_PATH)
endif (NOT DEFINED XME_EXTERNAL_DIR)

# Set src dir (CHROMOSOME sources): CACHE Variable XME_SRC_DIR (via find_file)
if (NOT DEFINED XME_SRC_DIR)
	find_file (XME_SRC_DIR "xme" PATHS "${XME_ROOT}" DOC "CHROMOSOME source directory" NO_DEFAULT_PATH)
endif (NOT DEFINED XME_SRC_DIR)

# Set tools dir (CHROMOSOME tools): CACHE Variable XME_TOOLS_DIR (via find_file)
if (NOT DEFINED XME_TOOLS_DIR)
	find_file (XME_TOOLS_DIR "tools" PATHS "${XME_ROOT}" DOC "CHROMOSOME tools directory" NO_DEFAULT_PATH)
endif (NOT DEFINED XME_TOOLS_DIR)

# Print version information
file (READ "${XME_ROOT}/VERSION.txt" XME_VERSION)
if (XME_VERSION)
    message (STATUS "Welcome to ${XME_VERSION}!")
endif ()

# Include other CMake scripts
include(Lint)

# Find related packages
find_package (Doxygen)

# Add install rule for LICENSE.txt
install(FILES "${XME_ROOT}/LICENSE.txt"
		DESTINATION	".")

# Add install rule for tools
install(DIRECTORY ${XME_TOOLS_DIR}
		DESTINATION "src")

# Add install rule for xme source directory (except ports which will be added in 
# xme_add_subdirectory in order to only install the needed platform specific sources)
install(DIRECTORY ${XME_SRC_DIR}
		DESTINATION "src"
		PATTERN "ports/*" EXCLUDE)

# Add install rule for default build directory.
# For example project resisting in ${XME_EXAMPLES_DIR}/some/project,
# the default build directory is ${XME_EXAMPLES_DIR}/build/some/project.
# The default build directory will only be installed if the respective
# project is contained in ${XME_EXAMPLES_DIR}. For packaging to not drop
# the default build directory, a dummy file is copied into it.
string (LENGTH "${XME_EXAMPLES_DIR}/" __LEN__)
string (LENGTH "${CMAKE_PARENT_LIST_FILE}" __LEN2__)
if (__LEN__ LESS __LEN2__)
	set (__MIN_LEN__ ${__LEN__})
else (__LEN__ LESS __LEN2__)
	set (__MIN_LEN__ ${__LEN2__})
endif (__LEN__ LESS __LEN2__)
string (SUBSTRING "${CMAKE_PARENT_LIST_FILE}" 0 ${__MIN_LEN__} __SUBSTR__)
if ("${XME_EXAMPLES_DIR}/" STREQUAL __SUBSTR__)
	# Retrieve relative path of project below the examples directory
	string (SUBSTRING "${CMAKE_PARENT_LIST_FILE}" ${__LEN__} -1 __SUBSTR__)
	string (REPLACE "/CMakeLists.txt" "" __SUBSTR__ "${__SUBSTR__}")

	xme_message (DEBUG "Installing default build directory 'examples/build/${__SUBSTR__}' for this project")

	install(FILES "${CMAKE_CURRENT_LIST_DIR}/dummy.in"
			DESTINATION "src/examples/build/${__SUBSTR__}"
			RENAME "dummy")
endif ("${XME_EXAMPLES_DIR}/" STREQUAL __SUBSTR__)

# CACHE Variable XME_FOUND
set (XME_FOUND TRUE CACHE INTERNAL "Whether CHROMOSOME has been found")

# Global property: XME_CURRENT_SOURCE_DIR
xme_simplify_path (_XME_CURRENT_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR "${_XME_CURRENT_SOURCE_DIR}")

# Global property XME_COMPONENTS
xme_set (PROPERTY_GLOBAL XME_COMPONENTS "")

# Global property XME_DOCUMENTATION_FILES
xme_set (PROPERTY_GLOBAL XME_DOCUMENTATION_FILES "")



#
# Nonfatal assertions:
# CHROMOSOME offers a programmer a number of mechanisms to notify
# higher-level code of failures. One of these mechanisms are
# assertion checks, which can be used to ensure consistent
# preconditions before the program continues. An assertion check
# is a boolean condition.
# By default, assertion failures (i.e., an assertion check that
# evaluates to false) is fatal, i.e., it either terminates the
# program with a respective message, drives the program into an
# infinite loop in order to retain the state for further analysis
# or simply aborts program execution. However, this behavior might
# not always be desirable. Especially in unit testing, the testsuite
# should not abort on the first assertion failure, but record the
# failure and then continue with the remaining tests if desired.
# In fact, a unit test might itself want to test whether a certain
# invocation actually produces an assertion failure (usually called
# death testing).
# In order to support all these usage patterns, CHROMOSOME allows to
# fine-tune the behavior of assertion failures as follows.
# A nonfatal assertion failure prints a respective message to stdout
# and program execution continues in "unexpected" state. This means
# that control flow immediately exits from the current function and
# returns XME_STATUS_UNEXPECTED where possible.
# For details, see the implementation in xme/core/defines.h and
# xme/ports/software/os/generic-os/defines/*/xme/defines_arch.h/.c.

# Cache entry:  XME_ENABLE_NONFATAL_ASSERTIONS
# Used by:      XME_ASSERT(), XME_ASSERT_RVAL(), XME_ASSERT_NORVAL()
# Purpose:      Controls how assertion failures affect program execution.
#               If this value is unset, then assertion failures are
#               fatal except if unit testing is enabled. Otherwise, if
#               the value is set, it determines whether assertion
#               failures are nonfatal or fatal.
# Usage:        Nonfatal assertions are disabled by default except if
#               XME_UNITTEST is defined. To enable, define a CMake
#               cache entry with the name XME_ENABLE_NONFATAL_ASSERTIONS
#               and set it to a value that is considered TRUE by CMake
#               (e.g., "1", "TRUE", "ON"). To disable, set the value of
#               the cache entry to a value considered FALSE by CMake
#               (e.g., "0", "FALSE", "OFF").
# Command line: cmake -G "..." -DXME_ENABLE_NONFATAL_ASSERTIONS=1
#               cmake -G "..." -DXME_ENABLE_NONFATAL_ASSERTIONS=0
# CMake GUI:    Add Entry -> XME_ENABLE_NONFATAL_ASSERTIONS, BOOL, ON
#               Add Entry -> XME_ENABLE_NONFATAL_ASSERTIONS, BOOL, OFF

include(GTest)
if (NOT DEFINED XME_ENABLE_NONFATAL_ASSERTIONS)
    set (XME_ENABLE_NONFATAL_ASSERTIONS "AUTO")
endif ()
set (XME_ENABLE_NONFATAL_ASSERTIONS "${XME_ENABLE_NONFATAL_ASSERTIONS}" CACHE STRING "Whether assertion failures, triggered by XME_ASSERT(), XME_ASSERT_RVAL() or XME_ASSERT_NORVAL(), are nonfatal or fatal. Set to AUTO, TRUE or FALSE. Set to AUTO to let the build system automatically decide based on whether unit testing is enabled or not.")
set_property (CACHE XME_ENABLE_NONFATAL_ASSERTIONS PROPERTY STRINGS AUTO TRUE FALSE)
if (XME_ENABLE_NONFATAL_ASSERTIONS STREQUAL "AUTO")
    if (XME_UNITTEST)
        xme_message (NOTE "XME_ENABLE_NONFATAL_ASSERTIONS is set to AUTO and unit tests are enabled, assertion failures will be nonfatal")
        add_definitions (-DXME_ASSERT_NONFATAL_MODE)
    else ()
        xme_message (NOTE "XME_ENABLE_NONFATAL_ASSERTIONS is set to AUTO and unit tests are disabled, assertion failures will be fatal")
    endif ()
elseif (XME_ENABLE_NONFATAL_ASSERTIONS)
    xme_message (NOTE "XME_ENABLE_NONFATAL_ASSERTIONS is set to TRUE, assertion failures will be nonfatal")
    add_definitions (-DXME_ASSERT_NONFATAL_MODE)
else ()
    xme_message (NOTE "XME_ENABLE_NONFATAL_ASSERTIONS is set to FALSE, assertion failures will be fatal")
endif ()



#
# Deprecation:
# CHROMOSOME wraps a large amount of functions (even from the C standard
# library, e.g., malloc()) to provide a common abstraction layer on all
# target platforms.
# This makes it necessary to mark direct calls to these functions as
# "deprecated" (really meaning that it is not recommended to call them
# directly). This feature is currently disabled by default, but will be
# enabled in the future and can be temporarily turned on by either
# changing the default value below or modifying the respective CMake
# cache entry (recommended). For details, see xme/core/deprecated.h and
# xme/ports/software/os/*/xme/core/deprecated_arch.h/.c.
#

# Cache entry:  XME_ENABLE_DEPRECATION
# Used by:      xme_core_fallback
# Purpose:      Controls whether deprecation is enabled or not.
#               The XME hardware abstraction layer (HAL) wraps a large number of
#               standard library functions. In order to keep software components
#               platform-independent, software components should not call the
#               wrapped functions directly. Enabling deprecation will cause the
#               original functions to be flagged as deprecated, causing compilation
#               warnings/errors when they are used in code. This allows to replace
#               the respective functions by their wrapped alternatives.
# Usage:        Deprecation is enabled by default. To disable, define a CMake
#               cache entry with the name XME_ENABLE_DEPRECATION and set it to a
#               value that is considered FALSE by CMake (e.g., "0", "FALSE", "OFF").
# Command line: cmake -G "..." -DXME_ENABLE_DEPRECATION=0
# CMake GUI:    Add Entry -> XME_ENABLE_DEPRECATION, BOOL, OFF
#
if (NOT DEFINED XME_ENABLE_DEPRECATION)
	set (XME_ENABLE_DEPRECATION "TRUE")
endif ()
set (XME_ENABLE_DEPRECATION "${XME_ENABLE_DEPRECATION}" CACHE BOOL "Whether deprecation is enabled or not. If enabled (TRUE), it will be illegal to call functions from the standard library that are wrapped by the XME hardware abstraction library (HAL). This is used to ensure components being developed in a platform-independent way.")
if (XME_ENABLE_DEPRECATION)
    xme_message (NOTE "XME_ENABLE_DEPRECATION is set, enabling deprecation of standard functions and platform dependent functions")
    add_definitions (-DXME_ENABLE_DEPRECATION)
else ()
    xme_message (NOTE "XME_ENABLE_DEPRECATION is *not* set, disabling deprecation of standard functions and platform dependent functions")
endif ()

#
# Compilation tests:
# The CHROMOSOME build system supports so-called compilation tests.
# Those are tests that are rarely executed by developers, but rather
# on automated build systems. Their purpose is to check whether a
# certain piece of code compiles or does not compile, depending on
# the intended functionality. This define controls whether compilation
# tests are enabled or not.
#

# Cache entry:  XME_ENABLE_COMPILATIONTESTS
# Used by:      XME build system
# Purpose:      Controls whether compilation tests are enabled or not.
#               Compilation tests are used to verify whether a code snippet compiles
#               or fails compiling, depending on what is expected.
# Usage:        Compilation tests are disabled by default. To enable, define a CMake
#               cache entry with the name XME_ENABLE_COMPILATIONTESTS and set it to
#               a value that is considered TRUE by CMake (e.g., "1", "TRUE", "ON").
# Command line: cmake -G "..." -DXME_ENABLE_COMPILATIONTESTS=1
# CMake GUI:    Add Entry -> XME_ENABLE_COMPILATIONTESTS, BOOL, ON
#
if (NOT DEFINED XME_ENABLE_COMPILATIONTESTS)
	set (XME_ENABLE_COMPILATIONTESTS "FALSE")
endif ()
set (XME_ENABLE_COMPILATIONTESTS "${XME_ENABLE_COMPILATIONTESTS}" CACHE BOOL "Whether compilation tests are enabled or not. If enabled (TRUE), additional targets will be created for test-compiling code snippets. Depending on the code snippet in question, it may be legal or wrong for compilation of respective the target to fail. Which situation is expected is determined by the suffix of the target: targets named *_expectPass indicate that compilation is expected to succeed, while *_expectFail indicates that compilation is meant to fail.")
if (XME_ENABLE_COMPILATIONTESTS)
    xme_message (NOTE "XME_ENABLE_COMPILATIONTESTS is set, enabling compilation tests")
    add_definitions (-DXME_ENABLE_COMPILATIONTESTS)
else ()
    xme_message (NOTE "XME_ENABLE_COMPILATIONTESTS is *not* set, disabling compilation tests")
endif ()

#
# Code example builds:
# The Doxygen documentation of CHROMOSOME lists various code examples that
# are ready for copy & paste in order to allow a user to experiment with
# XME more easily. In order to retain the consistency of the code examples
# with the actual code, they can be scheduled for build.
# The XME_ENABLE_CODE_EXAMPLE_BUILDS variable controls this behavior.
#

# Cache entry:  XME_ENABLE_CODE_EXAMPLE_BUILDS
# Used by:      XME build system
# Purpose:      Controls whether code examples should be scheduled for build.
# Usage:        Code example builds are disabled by default. To enable, define
#               a CMake cache entry with the name XME_ENABLE_CODE_EXAMPLE_BUILDS
#               and set it to a value that is considered TRUE by CMake (e.g.,
#               "1", "TRUE", "ON").
if (NOT DEFINED XME_ENABLE_CODE_EXAMPLE_BUILDS)
	set (XME_ENABLE_CODE_EXAMPLE_BUILDS "FALSE")
endif ()
set (XME_ENABLE_CODE_EXAMPLE_BUILDS "${XME_ENABLE_CODE_EXAMPLE_BUILDS}" CACHE BOOL "Whether code example builds are enabled or not. If enabled (TRUE), a target is generated for every code example that has been specified with the xme_code_example() CMake function where the 'BUILD' parameter was set. The generated targets are of the form '<componentName>_<exampleName>CodeExample'.")
if (XME_ENABLE_CODE_EXAMPLE_BUILDS)
    xme_message (NOTE "XME_ENABLE_CODE_EXAMPLE_BUILDS is set, enabling code example builds")
    add_definitions (-DXME_ENABLE_CODE_EXAMPLE_BUILDS)
else ()
    xme_message (NOTE "XME_ENABLE_CODE_EXAMPLE_BUILDS is *not* set, disabling code example builds")
endif ()

#
# Console colors:
# On some platforms, messages printed on the console using XME_LOG() can be colored
# according to their severity. A CMake cache variable can be used to turn off this
# behavior in case colored console output is not desired.
#

# Cache entry:  XME_ENABLE_CONSOLE_COLORS
# Used by:      xme_core_log
# Purpose:      Controls whether the console output should include color codes or not.
#               Depending on the implementation, using color codes might yield escape
#               characters in standard or error output.
# Usage:        Console colors are enabled by default. To disable, define a CMake
#               cache entry with the name XME_ENABLE_CONSOLE_COLORS and set it to a
#               value that is considered FALSE by CMake (e.g., "0", "FALSE", "OFF").
# Command line: cmake -G "..." -DXME_ENABLE_CONSOLE_COLORS=0
# CMake GUI:    Add Entry -> XME_ENABLE_CONSOLE_COLORS, BOOL, OFF
#
if (NOT DEFINED XME_ENABLE_CONSOLE_COLORS)
    set (XME_ENABLE_CONSOLE_COLORS "TRUE")
endif ()
set (XME_ENABLE_CONSOLE_COLORS "${XME_ENABLE_CONSOLE_COLORS}" CACHE BOOL "Whether colored console output is enabled or not. If enabled (TRUE), messages output with XME_LOG() might be colored depending on their severity (e.g., warnings in yellow, error messages in red) if the respective platform support it. Disabling console colors (FALSE) ensures that no color-related escape characters are present in standard or error output of the application. This is useful for storing console output in text files or for automatic analysis of console output, for example.")
if (XME_ENABLE_CONSOLE_COLORS)
    xme_message (NOTE "XME_ENABLE_CONSOLE_COLORS is set, enabling console colors")
    add_definitions (-DXME_ENABLE_CONSOLE_COLORS)
else ()
    xme_message (NOTE "XME_ENABLE_CONSOLE_COLORS is *not* set, disabling console colors")
endif ()

#
# Map files:
# Most compilers support the generation of map files (ususally *.map) during linking,
# one for every binary being built. Those files provide information about which symbols
# are compiled into the respective binary and can be used for debugging purposes.
# Use this cache entry to control whether map files should be generated or not.
# Notice that this will only have an effect if the CHROMOSOME build system supports
# generation of map files for the selected toolchain.
#

# Cache entry:  XME_ENABLE_MAP_FILES
# Used by:      XME build system
# Purpose:      Controls whether the linked should generate a linker address map file
#               for every binary being linked. Notice that this will only have an
#               effect if the CHROMOSOME build system supports generation of map files
#               for the selected toolchain.
# Usage:        Map files are disabled by default. To enable, define a CMake
#               cache entry with the name XME_ENABLE_MAP_FILES and set it to a
#               value that is considered TRUE by CMake (e.g., "1", "TRUE", "ON").
# Command line: cmake -G "..." -DXME_ENABLE_MAP_FILES=1
# CMake GUI:    Add Entry -> XME_ENABLE_MAP_FILES, BOOL, ON
#
if (NOT DEFINED XME_ENABLE_MAP_FILES)
    set (XME_ENABLE_MAP_FILES "FALSE")
endif ()
set (XME_ENABLE_MAP_FILES "${XME_ENABLE_MAP_FILES}" CACHE BOOL "Whether map files should be generated for every binary being linked. If enabled (TRUE) and the CHROMOSOME build system supports generation of map files for the selected toolchain, a map file (*.map) will be output in the build directory for every executable being linked.")
if (XME_ENABLE_MAP_FILES)
    xme_message (NOTE "XME_ENABLE_MAP_FILES is set, enabling generation of map files")
    add_definitions (-DXME_ENABLE_MAP_FILES)
else ()
    xme_message (NOTE "XME_ENABLE_MAP_FILES is *not* set, disabling generation of map files")
endif ()

#
# Build options:
# Build options are parameters that influence the availability of functions in
# the CHROMOSOME core and the amount of resources available by default.
# A typical use case is the definition of maximum data structure and buffer
# sizes that may vary depending on the (hardware) resource availability in the
# target system. In order to customize a CHROMOSOME node, the build system
# allows build options to be overwritten based on application-specific needs.
# This flag controls whether a list of build options should be printed during
# the configuration run in order to provide the developer an overview of all
# available options.
#

# Cache entry:  XME_ENABLE_BUILD_OPTION_DUMP
# Used by:      XME build system
# Purpose:      Controls whether a list of build options and associated values
#               should be printed during configuration of the build system.
# Usage:        Dumping of build options is disabled by default. To enable,
#               define a CMake cache entry with the name
#               XME_ENABLE_BUILD_OPTION_DUMP and set it to a value that is
#               considered TRUE by CMake (e.g., "1", "TRUE", "ON").
# Command line: cmake -G "..." -DXME_ENABLE_BUILD_OPTION_DUMP=1
# CMake GUI:    Add Entry -> XME_ENABLE_BUILD_OPTION_DUMP, BOOL, ON
#
if (NOT DEFINED XME_ENABLE_BUILD_OPTION_DUMP)
    set (XME_ENABLE_BUILD_OPTION_DUMP "FALSE")
endif ()
set (XME_ENABLE_BUILD_OPTION_DUMP "${XME_ENABLE_BUILD_OPTION_DUMP}" CACHE BOOL "Whether a list of build options should be dumped during the configuration run.")
if (XME_ENABLE_BUILD_OPTION_DUMP)
    xme_message (NOTE "XME_ENABLE_BUILD_OPTION_DUMP is set, enabling dumping of build options")
    add_definitions (-DXME_ENABLE_BUILD_OPTION_DUMP)
else ()
    xme_message (NOTE "XME_ENABLE_BUILD_OPTION_DUMP is *not* set, disabling dumping of build options")
endif ()

#
# Handle chromosome build options
#  - Provide default value
#  - Create include files since specifying all options on the command line
#    will exceed the command line limit (8192 on Windows).
#

# Configure options directory and add it to the include path
xme_set(PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/options")
xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
xme_include_directory("${_XME_OPTIONS_INCLUDE_DIR}")

# Global property XME_TARGET_IDENTIFIER_DEFINED
xme_defined (XME_TARGET_IDENTIFIER_DEFINED PROPERTY_GLOBAL XME_TARGET_IDENTIFIER)
if (NOT ${XME_TARGET_IDENTIFIER_DEFINED})
    # Check if XME_TARGET_IDENTIFIER has been set. This is useful in
    # order to select of a specific build host on the command line using
    # cmake -DXME_TARGET_IDENTIFIER=<...>.
    # If no target identifier is specified, XME attempts to automatically
    # detect it. Automatic detection is guided by xme/ports/autoDetect.cmake.
    #
    # N.B.: -DXME_TARGET_IDENTIFIER defines a CMake variable, which is
    #        evaluated here in order to set a property of the same name.
    if (DEFINED XME_TARGET_IDENTIFIER)
        xme_message (NOTE "Setting XME_TARGET_IDENTIFIER to '${XME_TARGET_IDENTIFIER}' (passed on command line)")
    else ()
        # Invoke the automatic detection script which is located in
        # the directory where the individual target files are located
        set (AUTODETECT_SCRIPT "${XME_SRC_DIR}/ports/targets/autoDetect.cmake")
        if (EXISTS ${AUTODETECT_SCRIPT})
            include (${AUTODETECT_SCRIPT})
            if (DEFINED XME_TARGET_IDENTIFIER)
                xme_message (NOTE "XME_TARGET_IDENTIFIER not defined, setting to '${XME_TARGET_IDENTIFIER}' according to automatic detection.")
            else ()
                xme_message (FATAL_ERROR "XME_TARGET_IDENTIFIER not defined and auto-detection script did not set a proper identifier! Consult \"${AUTODETECT_SCRIPT}\" for more information.")
            endif ()
        else ()
            string (TOLOWER ${CMAKE_SYSTEM_NAME} XME_TARGET_IDENTIFIER)
            xme_message (NOTE "XME_TARGET_IDENTIFIER not defined and no auto-detection script present. Assuming '${CMAKE_SYSTEM_NAME}' and setting to '${XME_TARGET_IDENTIFIER}'.")
        endif ()
    endif ()
    xme_set (PROPERTY_GLOBAL XME_TARGET_IDENTIFIER ${XME_TARGET_IDENTIFIER})
endif (NOT ${XME_TARGET_IDENTIFIER_DEFINED})
xme_get(_XME_TARGET_IDENTIFIER PROPERTY_GLOBAL XME_TARGET_IDENTIFIER)
xme_message (DEBUG "XME_TARGET_IDENTIFIER: '${_XME_TARGET_IDENTIFIER}'.")

# sanity check
if (NOT IS_DIRECTORY "${XME_SRC_DIR}/ports/targets/${_XME_TARGET_IDENTIFIER}")
	xme_get_supported_targets (XME_SUPPORTED_TARGETS)
	set (SUPPORTED_TARGETS)
	foreach (_TARGET ${XME_SUPPORTED_TARGETS})
		if (SUPPORTED_TARGETS)
			set (SUPPORTED_TARGETS "${SUPPORTED_TARGETS}, ${_TARGET}")
		else (SUPPORTED_TARGETS)
			set (SUPPORTED_TARGETS "${_TARGET}")
		endif (SUPPORTED_TARGETS)
	endforeach (_TARGET)
	xme_message (FATAL_ERROR "Unsupported target '${_XME_TARGET_IDENTIFIER}'! Please set XME_TARGET_IDENTIFIER to one of the targets listed in xme/ports/targets, namely: ${SUPPORTED_TARGETS}")
endif(NOT IS_DIRECTORY "${XME_SRC_DIR}/ports/targets/${_XME_TARGET_IDENTIFIER}")

# Build option files
if (XME_ENABLE_BUILD_OPTION_DUMP)
    xme_message (NOTE "Dumping active build options (unsorted, turn off by undefining XME_ENABLE_BUILD_OPTION_DUMP):")
endif ()
if (EXISTS "${CMAKE_SOURCE_DIR}/Options.cmake")
    xme_add_subdirectory(${CMAKE_SOURCE_DIR} FALSE "Options.cmake")
endif ()
xme_add_subdirectory(${XME_SRC_DIR}/ports/targets/${_XME_TARGET_IDENTIFIER} FALSE "Options.cmake")
xme_add_subdirectory(${XME_SRC_DIR} FALSE "Options.cmake")
_xme_build_option_file_close_all ()

# Check whether we have a valid image file specification
xme_defined (_XME_IMAGE_FORMAT_DEFINED PROPERTY_GLOBAL XME_IMAGE_FORMAT)
if (NOT ${_XME_IMAGE_FORMAT_DEFINED})
	xme_message (DEBUG "Image format not specified: will not run objcopy to create target image.")
else (NOT ${_XME_IMAGE_FORMAT_DEFINED})
	# Support more output formats as needed
	xme_get (_XME_IMAGE_FORMAT PROPERTY_GLOBAL XME_IMAGE_FORMAT)
	if (_XME_IMAGE_FORMAT STREQUAL "ihex")
		xme_set (PROPERTY_GLOBAL XME_IMAGE_EXTENSION "hex")
	elseif (_XME_IMAGE_FORMAT STREQUAL "srec")
		xme_set (PROPERTY_GLOBAL XME_IMAGE_EXTENSION "srec")
	elseif (_XME_IMAGE_FORMAT STREQUAL "binary")
		xme_set (PROPERTY_GLOBAL XME_IMAGE_EXTENSION "bin")
	else (_XME_IMAGE_FORMAT STREQUAL "binary")
		xme_message (WARNING "Unkown image format: '${_XME_IMAGE_FORMAT}'")
	endif (_XME_IMAGE_FORMAT STREQUAL "ihex")
endif (NOT ${_XME_IMAGE_FORMAT_DEFINED})

xme_defined (_XME_DEBUGGER_TARGET_DEFINED PROPERTY_GLOBAL XME_DEBUGGER_TARGET)
if (NOT ${_XME_DEBUGGER_TARGET_DEFINED})
	xme_message (DEBUG "XME_DEBUGGER_TARGET not defined. Defaulting to 'remote localhost:3333'.")
	xme_set (PROPERTY_GLOBAL XME_DEBUGGER_TARGET "remote localhost:3333")
endif (NOT ${_XME_DEBUGGER_TARGET_DEFINED})

# Generic build settings
xme_include_directory(${CMAKE_CURRENT_SOURCE_DIR})
xme_lib_directory(${CMAKE_CURRENT_SOURCE_DIR})

# Add target specific implementations
xme_add_subdirectory(${XME_SRC_DIR}/ports/targets/${_XME_TARGET_IDENTIFIER} TRUE)

# Add CHROMOSOME sources to the current project
xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR ${XME_SRC_DIR})

# Add chromosome implementations
xme_add_subdirectory (${XME_SRC_DIR} TRUE)

# Add external directory as a subproject in eclipse
xme_add_subdirectory (${XME_EXTERNAL_DIR} TRUE)

xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR ${_XME_CURRENT_SOURCE_DIR})

# Add custom search paths for CHROMOSOME components (typically used for third-party components)
foreach (_CD_ ${XME_COMPONENT_DIRS})
	xme_resolve_path(_CDS_ ${_CD_})
	xme_message (DEBUG "Adding custom component directory '${_CD_}' (which resolves to '${_CDS_}')")
	xme_add_subdirectory (${_CDS_})
endforeach (_CD_)

# Add compilation tests and code example builds
_xme_create_compilation_tests ()
_xme_create_code_example_builds ()

# XME configuration finished
set (XME_CONFIGURED TRUE)
