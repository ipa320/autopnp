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
# $Id: XMEUtils.cmake 7723 2014-03-10 09:31:46Z geisinger $
#

# This CMake script contains all helper functions which are used in FindXME.cmake.
# It was split up in order to provide a good overview over all used functions and macros.
# 

# Cache variable for CMake vebosity
if (NOT DEFINED XME_CMAKE_VEBOSITY)
    set (XME_CMAKE_VEBOSITY "NOTE")
elseif (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG")
    if (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "VERBOSE")
        if (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "NOTE")
            if (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "WARNING")
                if (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "ERROR")
                    if (NOT "${XME_CMAKE_VEBOSITY}" STREQUAL "FATAL_ERROR")
                        message (FATAL_ERROR "XME_CMAKE_VEBOSITY set to invalid value '${XME_CMAKE_VEBOSITY}', must be one of DEBUG, VERBOSE, NOTE, WARNING, ERROR, or FATAL_ERROR!")
                    endif ()
                endif ()
            endif ()
        endif ()
    endif ()
endif ()
set (XME_CMAKE_VEBOSITY "${XME_CMAKE_VEBOSITY}" CACHE STRING "CHROMOSOME CMake log message verbosity, one of DEBUG, VERBOSE, NOTE, WARNING, ERROR, or FATAL_ERROR")
set_property (CACHE XME_CMAKE_VEBOSITY PROPERTY STRINGS DEBUG VERBOSE NOTE WARNING ERROR FATAL_ERROR)



# Macro:      xme_message ( <severity> <message> ...)
#
# Purpose:    Outputs the given message(s) if the severity matches the global XME_CMAKE_VEBOSITY setting.
#
#             The message(s) is/are output if the severity of the message is at least as high as the
#             value of the cache variable XME_CMAKE_VEBOSITY. In particular this means that:
#              - Messages with severity DEBUG are output if XME_CMAKE_VEBOSITY is set to DEBUG.
#              - Messages with severity VERBOSE are output if XME_CMAKE_VEBOSITY is set to DEBUG or VERBOSE.
#              - Messages with severity NOTE are output if XME_CMAKE_VERBOSITY is set to DEBUG, VERBOSE or NOTE.
#              - Messages with severity WARNING are output if XME_CMAKE_VEBOSITY is set to DEBUG, VERBOSE, NOTE or WARNING.
#              - Messages with severity ERROR are output if XME_CMAKE_VEBOSITY is set to DEBUG, VERBOSE, NOTE, WARNING or ERROR.
#              - Messages with severity FATAL_ERROR are output if XME_CMAKE_VEBOSITY is set to DEBUG, VERBOSE, NOTE, WARNING,
#                ERROR or FATAL_ERROR.
#
# Parameters: <severity>      One of the values DEBUG, VERBOSE, WARNING, NOTE, ERROR or FATAL_ERROR with the
#                             semantics explained above.
#                             FATAL_ERROR causes the configuration run to be aborted at this point.
#                             ERROR causes the generation run to be disabled.
#                             WARNING, NOTE, VERBOSE and DEBUG allow the configuration and generation run to succeed.
#
#             <message> ...   Message(s) to display if the XME_CMAKE_VEBOSITY cache variable matches.
#
function (xme_message SEVERITY)
	if (ARGC LESS 2)
		message (FATAL_ERROR "Invalid number of arguments passed to xme_message(). Usage: xme_message( <severity> <message> [ <message2...> ] ), where <severity> is one of DEBUG, VERBOSE, NOTE, WARNING, ERROR or FATAL_ERROR.")
	endif ()

	set (ARGS ${ARGV})
	list (REMOVE_AT ARGS 0)
	if (${SEVERITY} STREQUAL "DEBUG")
		if ("${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG")
			message (STATUS ${ARGS})
		endif ()
	elseif (${SEVERITY} STREQUAL "VERBOSE")
		if ("${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "VERBOSE")
			message (STATUS ${ARGS})
		endif ()
	elseif (${SEVERITY} STREQUAL "NOTE")
		if ("${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "VERBOSE" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "NOTE")
			message (STATUS ${ARGS})
		endif ()
	elseif (${SEVERITY} STREQUAL "WARNING")
		if ("${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "VERBOSE" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "NOTE" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "WARNING")
			message (WARNING ${ARGS})
		endif ()
	elseif (${SEVERITY} STREQUAL "ERROR")
		if ("${XME_CMAKE_VEBOSITY}" STREQUAL "DEBUG" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "VERBOSE" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "NOTE" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "WARNING" OR "${XME_CMAKE_VEBOSITY}" STREQUAL "ERROR")
			message (SEND_ERROR ${ARGS})
		endif ()
	elseif (${SEVERITY} STREQUAL "FATAL_ERROR")
		message (FATAL_ERROR ${ARGS})
	else ()
		message (SEND_ERROR "Invalid message severity '${SEVERITY}' specified in call to xme_message()!")
		return()
	endif ()
endfunction (xme_message)

# Asserts that the given condition holds.
macro (xme_assert)
	if (${ARGC} LESS 1)
		xme_message (ERROR "Too few arguments to macro xme_assert()!")
	else ()
		if (${ARGV})
		else (${ARGV})
			set (__MESSAGE__ "XME assertion failed:")
			foreach (__ARG__ ${ARGV})
				set (__MESSAGE__ "${__MESSAGE__} ${__ARG__}")
			endforeach (__ARG__)
			xme_message (ERROR "${__MESSAGE__}")
		endif (${ARGV})
	endif()
endmacro (xme_assert)

# Remove trailing 0 (as workaround for hex2dec(), see below)
macro (shortenhex IN OUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro shortenhex()!")
	endif ()

	string (LENGTH ${IN} _POS)
	math(EXPR _POS "${_POS} - 1")

	string(SUBSTRING  ${IN} ${_POS} -1 _CHAR)
	if (NOT ${CHAR} EQUAL 0)
		xme_message (FATAL_ERROR "Addresses must end in 0 (due to overflow in CMake's math() command.")
	endif (NOT ${CHAR} EQUAL 0)

	string(SUBSTRING ${IN} 0 ${_POS} ${OUT})
endmacro (shortenhex IN OUT)

# Hexadecimal (as string) to decimal (as number) conversion
macro(hex2dec HEX DEC)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro hex2dec()!")
	endif ()

	if (${HEX} STREQUAL "0x80000000")
		# TODO: Work around the fact that CMake's math implementation works on int32_t.
		#       Fortunately, the largest hex value we currently need to manipulate is
		#       0x08000000, so shortenhex() is currently not needed
		set (${DEC} 2147483648)
	else (${HEX} STREQUAL "0x80000000")
		set (_DIGITS "0123456789ABCDEF")
		set (_DEC 0)

		string(TOUPPER ${HEX} _HEX)
		string(SUBSTRING ${_HEX} 0 2 _PREF)
		if (NOT "0X" STREQUAL _PREF)
			xme_message (FATAL_ERROR "hex2dec: Not a hexadecimal number: '${HEX}'")
		endif (NOT "0X" STREQUAL _PREF)

		set (I 2)
		string (LENGTH ${_HEX} N)

		while (${I} LESS ${N})
			string(SUBSTRING ${_HEX} ${I} 1 _CHAR)
			string(FIND ${_DIGITS} ${_CHAR} _POS)

			if (_POS EQUAL -1)
				xme_message (FATAL_ERROR "hex2dec: Not a hexadecimal number: '${HEX}'")
			endif (_POS EQUAL -1)
			math(EXPR _DEC "${_DEC}*16 + ${_POS}")
			math(EXPR I "${I} + 1")

			if (${_DEC} LESS 0)
				xme_message (FATAL_ERROR "hex2dec: An overflow occurred. Maximum argument: 0x7fffffff.")
			endif (${_DEC} LESS 0)
		endwhile (${I} LESS ${N})

		set (${DEC} ${_DEC})
	endif (${HEX} STREQUAL "0x80000000")
endmacro(hex2dec)

# Decimal (as number) to hexadecimal (as string) conversion
macro(dec2hex DEC HEX)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro dec2hex()!")
	endif ()

	set (_DIGITS "0123456789ABCDEF")

	if (DEC LESS 0)
		xme_message (FATAL_ERROR "dec2hex: Not a non-negative integer: ${DEC}")
	endif (DEC LESS 0)

	if (${DEC} EQUAL 0)
		set (${HEX} "0x0")
	else (${DEC} EQUAL 0)
		set (_HEX "")
		set (_DEC ${DEC})
		while (_DEC GREATER 0)
			math (EXPR _DIGVAL "${_DEC} % 16")
			string(SUBSTRING ${_DIGITS} ${_DIGVAL} 1 _DIG)
			set (_HEX "${_DIG}${_HEX}")
			math(EXPR _DEC "${_DEC} / 16")
		endwhile (_DEC GREATER 0)

		set (${HEX} "0x${_HEX}")
	endif (${DEC} EQUAL 0)
endmacro(dec2hex)

# Macro:      xme_pad_string ( <variable> <count> [ <character> ] )
#
# Purpose:    Pads the string stored in the given variable with the specified character (default space)
#             on the right until it is at least count characters long.
#
function (xme_pad_string VARIABLE COUNT)
    if (${ARGC} GREATER 3)
        xme_message (WARNING "Too many arguments to function xme_pad_string()!")
    endif ()

    set (CHAR " ")
    if (${ARGC} EQUAL 3)
        string (SUBSTRING ${ARGV2} 0 1 CHAR)
    endif ()

    set (VAR ${${VARIABLE}})
    string (LENGTH ${VAR} LEN)
    while (${LEN} LESS ${COUNT})
        set (VAR "${VAR}${CHAR}")
        math (EXPR LEN "${LEN} + 1")
    endwhile ()

    set (${VARIABLE} ${VAR} PARENT_SCOPE)
endfunction (xme_pad_string)

# Macro:      xme_simplify_path ( <output> <path> )
#
# Purpose:    Simplifies the given input path by replacing each occurrence of "." and/or
#             ".." in conjunction with a specified parent directory. 
#             Also normalizes given input path by removing a trailing slash "/" (except when the path
#             ends with ":/", e.g. "C:/", or when the path is "/").
#             Only works directories with "/" as delimiter.
#
#             Here are some examples (see testsuite below for more examples):
#              - "/some/./dir" --> "/some/dir"
#              - "/some/path/.." --> "/some"
#              - "/some/path/name/../../dir" --> "/some/dir"
#              - "/some/../../dir" --> "/../dir"
#              - "some/../../dir" --> "../dir"
#              - "some/dir/" --> "some/dir"
#              - "/some/dir/" --> "some/dir"
#              - "./" --> "."
#              - "C:/" --> "C:/"
#              - "C:/some/" --> "C:/some"
#
# Parameters: <output> Variable where result is stored.
#             <path>   Given input path.
# 
macro (xme_simplify_path OUTPUT PATH)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_simplify_path()!")
	endif ()

	set (VALUE ${PATH})

	while (1)
		if (${VALUE} MATCHES "^[.]/")
			string (REGEX REPLACE "^[.]/" "" VALUE ${VALUE})
		else (${VALUE} MATCHES "^[.]/")
			break()
		endif (${VALUE} MATCHES "^[.]/")
	endwhile (1)

	while (1)
		if (${VALUE} MATCHES "/[.]$")
			string (REGEX REPLACE "/[.]$" "" VALUE ${VALUE})
		else (${VALUE} MATCHES "/[.]$")
			break()
		endif (${VALUE} MATCHES "/[.]$")
	endwhile (1)

	while (1)
		if (${VALUE} MATCHES "/[.]/")
			string (REGEX REPLACE "/[.]/" "/" VALUE ${VALUE})
		else (${VALUE} MATCHES "/[.]/")
			break()
		endif (${VALUE} MATCHES "/[.]/")
	endwhile (1)

	while (1)
		if (${VALUE} MATCHES "/[^/.]+/[.][.]($|/)")
			string (REGEX REPLACE "/[^/.]+/[.][.]($|/)" "\\1" VALUE ${VALUE})
		else (${VALUE} MATCHES "/[^/.]+/[.][.]($|/)")
			if (${VALUE} MATCHES "^[^/.]+/[.][.]($|/)")
				string (REGEX REPLACE "^[^/.]+/[.][.]($|/)" "" VALUE ${VALUE})
			else (${VALUE} MATCHES "^[^/.]+/[.][.]($|/)")
				break ()
			endif (${VALUE} MATCHES "^[^/.]+/[.][.]($|/)")
		endif (${VALUE} MATCHES "/[^/.]+/[.][.]($|/)")
	endwhile (1)

	# Remove trailing "/" except after a ":" (e.g. "C:/") or when it is the only character ("/" for root directory under Unix "/")
	if (NOT ${VALUE} MATCHES ":/$|^/$")
		if (${VALUE} MATCHES "/$")
			string (REGEX REPLACE "/$" "" VALUE ${VALUE})
		endif (${VALUE} MATCHES "/$")
	endif (NOT ${VALUE} MATCHES ":/$|^/$")

	if ("${VALUE}" STREQUAL "")
		set (${OUTPUT} ".")
	else ("${VALUE}" STREQUAL "")
		set (${OUTPUT} ${VALUE})
	endif ("${VALUE}" STREQUAL "")

	#xme_message (DEBUG "Simplified path from '${PATH}' to '${${OUTPUT}}'")
endmacro (xme_simplify_path)

macro (xme_resolve_path OUTPUT PATH)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_resolve_path()!")
	endif ()

	if (IS_ABSOLUTE ${PATH})
		set (_PATH_ ${PATH})
	else (IS_ABSOLUTE ${PATH})
		xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
		set (_PATH_ "${_XME_CURRENT_SOURCE_DIR}/${PATH}")
	endif (IS_ABSOLUTE ${PATH})

	xme_simplify_path(${OUTPUT} ${_PATH_})

	#xme_message (DEBUG "Resolved path from '${PATH}' to '${${OUTPUT}}'")
endmacro (xme_resolve_path)

macro (xme_defined RVAL TYPE NAME)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to macro xme_defined()!")
	endif ()

	if ("VARIABLE" STREQUAL ${TYPE})
		if (DEFINED ${NAME})
			set (${RVAL} 1)
		else (DEFINED ${NAME})
			set (${RVAL} 0)
		endif (DEFINED ${NAME})
	elseif ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		get_property (${RVAL} GLOBAL PROPERTY ${NAME} SET)
	else ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		xme_message (FATAL_ERROR "Invalid value for TYPE in function xme_defined: '${TYPE}'")
	endif ("VARIABLE" STREQUAL ${TYPE})
endmacro (xme_defined)

macro (xme_set TYPE NAME VAL)
	if (${ARGC} GREATER 3)
		xme_message(WARNING "Too many arguments to macro xme_set()!")
	endif ()

	if ("VARIABLE" STREQUAL ${TYPE})
		set (${NAME} ${VAL})
	elseif ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		set_property (GLOBAL PROPERTY ${NAME} ${VAL})
	else ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		xme_message (FATAL_ERROR "Invalid value for TYPE in function xme_set: '${TYPE}'")
	endif ("VARIABLE" STREQUAL ${TYPE})
endmacro (xme_set)

macro (xme_weak_set TYPE NAME VAL)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to macro xme_weak_set()!")
	endif ()

	xme_defined(_DEFINED ${TYPE} ${NAME})
	if (NOT ${_DEFINED})
		xme_set(${TYPE} ${NAME} ${VAL})
	endif (NOT ${_DEFINED})
endmacro (xme_weak_set)

macro (xme_get RVAL TYPE NAME)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to macro xme_get()!")
	endif ()

	if ("VARIABLE" STREQUAL ${TYPE})
		set (${RVAL} ${${NAME}})
	elseif ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		get_property (${RVAL} GLOBAL PROPERTY ${NAME})
		#xme_message (DEBUG "GET PROP ${NAME} --> ${${RVAL}}")
	else ("PROPERTY_GLOBAL" STREQUAL ${TYPE})
		xme_message (FATAL_ERROR "Invalid value for TYPE in function xme_get: '${TYPE}'")
	endif ("VARIABLE" STREQUAL ${TYPE})
endmacro(xme_get)

# Macro:      xme_append ( [ VARIABLE | PROPERTY_GLOBAL ] <name> <value> [ UNIQUE ] )
#
# Purpose:    Appends the given value(s) to a variable or global property.
#             If the element does not exist, it will be defined and set to the given value(s).
#             If multiple values should be appended, the syntax "elem1;elem2,elem3" should be
#             used.
#
# Parameters:
#             VARIABLE   Specifies that a variable of the given name is to be changed.
#
#             PROPERTY_GLOBAL
#                        Specifies that a global property of the given name is to be changed.
#
#             <name>     Name of the variable or global property (depending on the value of the first
#                        argument). The variable or global property is interpreted as a list.
#
#             <value>    Value(s) to append to the list.
#
#             UNIQUE     If specified, the element will only be appended to the list if it is not yet
#                        contained in the list.
#
macro (xme_append TYPE NAME VALUE)
    if (${ARGC} GREATER 4)
        xme_message (WARNING "Too many arguments to macro xme_append()!")
    endif ()

    set (__UNIQUE__ NO)
    if (${ARGC} EQUAL 4)
        if (NOT ${ARGV3} STREQUAL "UNIQUE")
            xme_message (FATAL_ERROR "Invalid arguments passed to macro xme_append()! Usage: xme_append ( [ VARIABLE | PROPERTY_GLOBAL ] <name> <value> [ UNIQUE ] )")
        else ()
            set (__UNIQUE__ YES)
        endif ()
    endif ()

    foreach (__ELEMENT__ ${VALUE})
        if ("VARIABLE" STREQUAL "${TYPE}")
            list (FIND "${NAME}" "${__ELEMENT__}" __FOUND__)
            if (__UNIQUE__ AND __FOUND__ GREATER -1)
                # Do not append
            else ()
                list (APPEND "${NAME}" "${__ELEMENT__}")
            endif ()
        elseif ("PROPERTY_GLOBAL" STREQUAL "${TYPE}")
            get_property (__LIST__ GLOBAL PROPERTY "${NAME}")
            list (FIND __LIST__ "${__ELEMENT__}" __FOUND__)
            if (__UNIQUE__ AND __FOUND__ GREATER -1)
                # Do not append
            else ()
                list (APPEND __LIST__ "${__ELEMENT__}")
                set_property (GLOBAL PROPERTY "${NAME}" "${__LIST__}")
            endif ()
        else ()
            xme_message (FATAL_ERROR "Invalid TYPE argument passed to macro xme_append(): '${TYPE}'")
        endif ()
    endforeach (__ELEMENT__)
endmacro(xme_append)

# Macro:      xme_contains ( [ VARIABLE | PROPERTY_GLOBAL ] <name> <value> <output> )
#
# Purpose:    Returns whether the given value is contained in the list with the given name.
#
# Parameters:
#             VARIABLE   Specifies that a variable of the given name is to be checked.
#
#             PROPERTY_GLOBAL
#                        Specifies that a global property of the given name is to be checked.
#
#             <name>     Name of the variable or global property (depending on the value of the first
#                        argument). The variable or global property is interpreted as a list.
#
#             <value>    Value(s) to check in the list.
#
#             <output>   Name of a variable that is set to YES or NO depending on whether
#                        the given value was found in the list.
#
macro (xme_contains TYPE NAME VALUE OUTPUT)
    if (NOT ${ARGC} EQUAL 4)
        xme_message (WARNING "Inappropriate number of arguments to macro xme_contains()!")
    endif ()
    
    if ("VARIABLE" STREQUAL "${TYPE}")
        list (FIND "${NAME}" "${VALUE}" __FOUND__)
        if (__FOUND__ LESS 0)
            set (${OUTPUT} NO)
        else ()
            set (${OUTPUT} YES)
        endif ()
    elseif ("PROPERTY_GLOBAL" STREQUAL "${TYPE}")
        get_property (__LIST__ GLOBAL PROPERTY "${NAME}")
        list (FIND __LIST__ "${VALUE}" __FOUND__)
        if (__FOUND__ LESS 0)
            set (${OUTPUT} NO)
        else ()
            set (${OUTPUT} YES)
        endif ()
    else ()
        xme_message (FATAL_ERROR "Invalid TYPE argument passed to macro xme_contains(): '${TYPE}'")
    endif ()
endmacro (xme_contains)

macro (xme_build_option_tag FILE_NAME TAG)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_build_option_tag()!")
	endif ()

	string (REGEX REPLACE "/|\\.|\\\\" "_" _TMP ${FILE_NAME})
	string (TOUPPER ${_TMP} _TMP)
	set (${TAG} ${_TMP})
endmacro (xme_build_option_tag)

# Macro:      xme_build_option_include ( <include-file> <option-file> )
#
# Purpose:    Adds an #include directive to a build option file. This allows to add additional
#             includes to a build option files.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
# Parameters: <include-file>             Include file to add to the respective build option file.
#
#             <option-file>              The name (and optionally relative path) of the build option file
#                                        to add the #include directive to. A file with that name will be
#                                        automatically created if it does not yet exist.
#
macro (xme_build_option_include INCLUDE_FILE OPTION_FILE)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_build_option_include()!")
	endif ()

	xme_build_option_tag (${OPTION_FILE} "TAG")
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_CLOSED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_CLOSED)
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_OPENED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED)

	if (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})
		message (FATAL_ERROR "Unable to add include '${INCLUDE_FILE}' to '${OPTION_FILE}': build options file has already been closed!")
	endif (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})

	if (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})
		_xme_build_option_file_open (${OPTION_FILE})
	endif (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})

	xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	set (_XME_OPTIONS_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}.tmp")

	file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#include \"${INCLUDE_FILE}\"\n")
endmacro (xme_build_option_include)

# Macro:      xme_build_option ( <option-name> <default-value> <option-file> <description> [ <prefix> ] )
#
# Purpose:    Adds a build option to a build option file. This allows to set up platform-specific
#             definitions that can be used from within all source and header files within CHROMOSOME.
#             A typical use case is the definition of maximum data structure and buffer sizes that
#             may vary depending on the resource availability in the target system.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
# Parameters: <option-name>              Name of the build option to set. It should be in all caps and
#                                        have a meaningful prefix, because the identifier will be
#                                        visible at global scope!
#
#             <default-value>            Default value for the build option.
#
#             <option-file>              The name (and optionally relative path) of the build option file
#                                        to add the build option to.
#
#             <description>              Description of the build option (for documentation purposes).
#
#             [ <prefix> ]               Optional parameter specifying a prefix to remove in the option
#                                        name. A file with that name will be automatically created if
#                                        it does not yet exist.
#
macro (xme_build_option OPTION_NAME DEFAULT_VALUE OPTION_FILE DESCRIPTION)
	if (${ARGC} GREATER 5)
		xme_message (WARNING "Too many arguments to macro xme_build_option()!")
	endif ()

	# Optional parameters
	if (${ARGC} GREATER 4)
		set(_OPTION_PREFIX ${ARGV4})

		# Remove prefix from generated option
		# TODO: This does not only strip the prefix if it is a prefix, but strips any occurence of it in the name!
		string (REPLACE ${_OPTION_PREFIX} "" OPTION_NAME_DEFINE ${OPTION_NAME})
	else (${ARGC} GREATER 4)
		set (OPTION_NAME_DEFINE ${OPTION_NAME})
	endif (${ARGC} GREATER 4)

	if (${ARGC} GREATER 5)
		xme_message (FATAL_ERROR "Too many options for macro xme_build_option.\n")
	endif (${ARGC} GREATER 5)

	xme_build_option_tag (${OPTION_FILE} "TAG")
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_CLOSED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_CLOSED)
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_OPENED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED)

	if (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})
		xme_message (FATAL_ERROR "Unable to add option '${OPTION_NAME}' to '${OPTION_FILE}': build options file has already been closed!")
	endif (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})

	if (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})
		_xme_build_option_file_open (${OPTION_FILE})
	endif (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})

	xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	set (_XME_OPTIONS_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}.tmp")

	# Provide default values for XME options
	xme_defined (_DEFINED PROPERTY_GLOBAL ${OPTION_NAME})
	if (NOT ${_DEFINED})
		xme_message (VERBOSE "Setting build option '${OPTION_NAME}' to '${DEFAULT_VALUE}' (default value).")

		# Write default value to global property so it can be accessed in other CMake scripts
		xme_set (PROPERTY_GLOBAL ${OPTION_NAME} "${DEFAULT_VALUE}")
		xme_get (_VALUE PROPERTY_GLOBAL ${OPTION_NAME})
	else (NOT ${_DEFINED})
		xme_get (_VALUE PROPERTY_GLOBAL ${OPTION_NAME})
		xme_message (VERBOSE "Setting build option '${OPTION_NAME}' to '${_VALUE}' (user supplied value).")
	endif (NOT ${_DEFINED})

    file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "// Description: ${DESCRIPTION}\n")
    file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "// Default value: ${DEFAULT_VALUE}\n")
    xme_defined (_XME_OPTIONS_COMMENT_DEFINED PROPERTY_GLOBAL "XME_OPTIONS_COMMENT_${OPTION_NAME}")
    if (_XME_OPTIONS_COMMENT_DEFINED)
        xme_get (_OVERWRITTEN_IN PROPERTY_GLOBAL "XME_OPTIONS_OVERWRITTEN_IN_${OPTION_NAME}")
        xme_get (_COMMENT PROPERTY_GLOBAL "XME_OPTIONS_COMMENT_${OPTION_NAME}")
        file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "// Overwritten in ${_OVERWRITTEN_IN}\n")
        if (_COMMENT)
            file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "// Reason: ${_COMMENT}\n")
        endif ()
    endif ()
    file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#ifndef ${OPTION_NAME_DEFINE}\n")
    file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "    #define ${OPTION_NAME_DEFINE} ${_VALUE}\n")
    file (APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#endif\n")
    
    if (XME_ENABLE_BUILD_OPTION_DUMP)
        xme_message (NOTE " * ${OPTION_NAME_DEFINE} = ${_VALUE}")
    endif ()
endmacro (xme_build_option)

# Macro:      xme_build_option_check ( <option-name> <expected-value> <option-file> )
#
# Purpose:    Checks whether a certain build option is set to an expected value and raises a build
#             error if it is not. A typical use case is to verify that certain platform-specific
#             defines are set for all source and header files.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
# Parameters: <option-name>              Name of the build option to check.
#
#             <expected-value>           Expected value of the build option.
#
#             <option-file>              The name (and optionally relative path) of the build option file
#                                        to check the build option in. A file with that name will be
#                                        automatically created if it does not yet exist.
#
macro (xme_build_option_check OPTION_NAME EXPECTED_VALUE OPTION_FILE)
	if (${ARGC} GREATER 4)
		xme_message (WARNING "Too many arguments to macro xme_build_option_check()!")
	endif ()

	# Optional parameters
	if (${ARGC} GREATER 3)
		set(_OPTION_PREFIX ${ARGV3})

		# Remove prefix from generated option
		string (REPLACE ${_OPTION_PREFIX} "" OPTION_NAME_DEFINE ${OPTION_NAME})
	else (${ARGC} GREATER 3)
		set (OPTION_NAME_DEFINE ${OPTION_NAME})
	endif (${ARGC} GREATER 3)

	xme_build_option_tag (${OPTION_FILE} "TAG")
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_CLOSED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_CLOSED)
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_OPENED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED)

	if (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})
		xme_message (FATAL_ERROR "Unable to assert option '${OPTION_NAME}' in '${OPTION_FILE}': build options file has already been closed!")
	endif (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})

	if (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})
		_xme_build_option_file_open (${OPTION_FILE})
	endif (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})

	xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	set (_XME_OPTIONS_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}.tmp")

	if (${EXPECTED_VALUE} STREQUAL "DEFINED")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#ifndef ${OPTION_NAME_DEFINE}\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "    #error \"Define ${OPTION_NAME_DEFINE} should be defined, but it is not defined!\"\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#endif\n")
	elseif (${EXPECTED_VALUE} STREQUAL "UNDEFINED")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#ifdef ${OPTION_NAME_DEFINE}\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "    #error \"Define ${OPTION_NAME_DEFINE} should not be defined, but it is defined!\"\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#endif\n")
	else ()
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#ifndef ${OPTION_NAME_DEFINE}\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "    #error \"Define ${OPTION_NAME_DEFINE} should be set to ${EXPECTED_VALUE}, but it is not defined!\"\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#elif ${OPTION_NAME_DEFINE} != ${EXPECTED_VALUE}\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "    #error \"Define ${OPTION_NAME_DEFINE} should be set to ${EXPECTED_VALUE}, but it is set to a different value!\"\n")
		file(APPEND ${_XME_OPTIONS_INCLUDE_FILE} "#endif\n")
	endif ()
endmacro (xme_build_option_check)

# Macro:      _xme_build_option_file_open ( <option-file> )
#
# Purpose:    Opens a build option file with the given name.
#             This macro is for internal use only.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
# Parameters: <option-file>              The name (and optionally relative path) of the build option file
#                                        to open. A file with that name will be automatically created if
#                                        it does not yet exist.
#
macro (_xme_build_option_file_open OPTION_FILE)
	if (${ARGC} GREATER 1)
		xme_message (WARNING "Too many arguments to macro _xme_build_option_file_open()!")
	endif ()

	xme_build_option_tag (${OPTION_FILE} "TAG")

	xme_defined (_XME_OPTIONS_INCLUDE_FILE_CLOSED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_CLOSED)
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_OPENED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED)

	if (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})
		xme_message (FATAL_ERROR "Unable to open option file '${OPTION_FILE}': it has already been closed!")
	endif (${_XME_OPTIONS_INCLUDE_FILE_CLOSED})

	if (${_XME_OPTIONS_INCLUDE_FILE_OPENED})
		xme_message (FATAL_ERROR "Unable to open option file '${OPTION_FILE}': it has already been opened!")
	endif (${_XME_OPTIONS_INCLUDE_FILE_OPENED})

	xme_set (PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED 1)

	# Remember that this file is a build options file and that it is currently open
	xme_get (_XME_OPTIONS_FILE_LIST PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST")
	xme_get (_XME_OPTIONS_FILE_LIST_OPEN PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN")
	list (APPEND _XME_OPTIONS_FILE_LIST ${OPTION_FILE})
	list (APPEND _XME_OPTIONS_FILE_LIST_OPEN ${OPTION_FILE})
	xme_set (PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST" "${_XME_OPTIONS_FILE_LIST}")
	xme_set (PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN" "${_XME_OPTIONS_FILE_LIST_OPEN}")

	xme_get (_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	set (_XME_OPTIONS_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}.tmp")

	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "// DO NOT EDIT! DO NOT EDIT!  DO NOT EDIT!  DO NOT EDIT! DO NOT EDIT!\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "//\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "// This file will be re-generated by FindXME.cmake each time one of the build options is changed.\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "// Instead of modifying settings here, set the according XME property, adjust the default value\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "// set by the xme_build_option() CMake macro or add the according definition to the compiler\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "// command line (e.g., using add_definitions(-DKEY=VALUE)).\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "//\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "#ifndef ${TAG}\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "#define ${TAG}\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "\n")
endmacro (_xme_build_option_file_open)

macro (xme_build_option_set NAME VAL MSG)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to macro xme_build_option_set()!")
	endif ()

	xme_message (VERBOSE "Forcing option ${NAME} to ${VAL}: ${MSG}")
	xme_set (PROPERTY_GLOBAL ${NAME} ${VAL})
    xme_set (PROPERTY_GLOBAL "XME_OPTIONS_OVERWRITTEN_IN_${NAME}" ${CMAKE_CURRENT_LIST_FILE})
    xme_set (PROPERTY_GLOBAL "XME_OPTIONS_COMMENT_${NAME}" ${MSG})
endmacro (xme_build_option_set)

macro (xme_build_option_weak_set NAME VAL)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_build_option_weak_set()!")
	endif ()

	xme_weak_set (PROPERTY_GLOBAL ${NAME} ${VAL})
endmacro (xme_build_option_weak_set)

# Macro:      _xme_build_option_file_close ( <option-file> )
#
# Purpose:    Closes a build option file with the given name.
#             This macro is for internal use only.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
# Parameters: <option-file>              The name (and optionally relative path) of the build option file
#                                        to close. It must have been previously opened using
#                                        _xme_build_option_file_open().
#
macro (_xme_build_option_file_close OPTION_FILE)
	if (${ARGC} GREATER 1)
		xme_message (WARNING "Too many arguments to macro _xme_build_option_file_close()!")
	endif ()

	xme_build_option_tag (${OPTION_FILE} "TAG")
	xme_defined (_XME_OPTIONS_INCLUDE_FILE_OPENED PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_OPENED)

	if (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})
		xme_message (FATAL_ERROR "Cannot close option file '${OPTION_FILE}': it has never been opened.")
	endif (NOT ${_XME_OPTIONS_INCLUDE_FILE_OPENED})

	xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	set (_XME_OPTIONS_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}.tmp")

	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "\n")
	file (APPEND "${_XME_OPTIONS_INCLUDE_FILE}" "#endif // #ifndef ${TAG}\n")

	xme_set (PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_FILE_${TAG}_CLOSED TRUE)

	# Remember that this build options file is now closed
	xme_get (_XME_OPTIONS_FILE_LIST_OPEN PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN")
	list (REMOVE_ITEM _XME_OPTIONS_FILE_LIST_OPEN ${OPTION_FILE})
	xme_set (PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN" "${_XME_OPTIONS_FILE_LIST_OPEN}")

	# Only update build option file if it has actually changed.
	# Since many source files may directly or indirectly depend on this build option file,
	# this prevents those files from being rebuilt when it is not really required.
	set (_XME_OPTIONS_REAL_INCLUDE_FILE "${_XME_OPTIONS_INCLUDE_DIR}/${OPTION_FILE}")
	execute_process (COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_XME_OPTIONS_INCLUDE_FILE}" "${_XME_OPTIONS_REAL_INCLUDE_FILE}")
	execute_process (COMMAND ${CMAKE_COMMAND} -E remove "${_XME_OPTIONS_INCLUDE_FILE}")
endmacro (_xme_build_option_file_close)

# Macro:      _xme_build_option_file_close_all ( <option-file> )
#
# Purpose:    Closes all currently open build option files.
#             This macro is for internal use only.
#
#             Build options are used to collect platform-specific definitions. They are collected
#             in so-called build option files that are generated during the CMake configuration run
#             and placed in the build directory.
#
macro (_xme_build_option_file_close_all)
	if (${ARGC} GREATER 0)
		xme_message (WARNING "Too many arguments to macro _xme_build_option_file_close_all()!")
	endif ()

	xme_get (_XME_OPTIONS_FILE_LIST_OPEN PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN")

	foreach (_OPTION_FILE ${_XME_OPTIONS_FILE_LIST_OPEN})
		_xme_build_option_file_close (${_OPTION_FILE})
	endforeach (_OPTION_FILE)

	# Clear the list of currently open build options files
	set (_XME_OPTIONS_FILE_LIST_OPEN)
	xme_set (PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST_OPEN" "${_XME_OPTIONS_FILE_LIST_OPEN}")

	# Remove all files in the build options directory that are not supposed to be there
	# (this is for consistency that a file actually vanishes after it is not used any more
	# as a build options file)
	xme_get(_XME_OPTIONS_INCLUDE_DIR PROPERTY_GLOBAL XME_OPTIONS_INCLUDE_DIR)
	file (GLOB_RECURSE _ALL_FILES RELATIVE "${_XME_OPTIONS_INCLUDE_DIR}" "${_XME_OPTIONS_INCLUDE_DIR}/*")

	xme_get (_XME_OPTIONS_FILE_LIST PROPERTY_GLOBAL "XME_OPTIONS_FILE_LIST")
	foreach (_FILE ${_XME_OPTIONS_FILE_LIST})
		list (REMOVE_ITEM _ALL_FILES ${_FILE})
	endforeach (_FILE)

	foreach (_FILE ${_ALL_FILES})
		xme_message (DEBUG "Removing unused build option file '${_XME_OPTIONS_INCLUDE_DIR}/${_FILE}'")
		file (REMOVE "${_XME_OPTIONS_INCLUDE_DIR}/${_FILE}")
	endforeach (_FILE)
endmacro (_xme_build_option_file_close_all)

macro (xme_build_option_assert OPTION_NAME MSG)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_build_option_assert()!")
	endif ()

	xme_defined(_DEFINED PROPERTY_GLOBAL ${OPTION_NAME})
	if (NOT ${_DEFINED})
		xme_message (FATAL_ERROR "xme_build_option_set(PROPERTY_GLOBAL ${OPTION_NAME} <...>) is required: ${MSG}")
	endif (NOT ${_DEFINED})
endmacro (xme_build_option_assert)

macro (xme_get_supported_targets OUTPUT)
	if (${ARGC} GREATER 1)
		xme_message (WARNING "Too many arguments to macro xme_get_supported_targets()!")
	endif ()

	set (${OUTPUT})
	file (GLOB _XME_SUPPORTED_TARGETS "" "${XME_SRC_DIR}/ports/targets/*")
	foreach (_TARGET ${_XME_SUPPORTED_TARGETS})
		string (REPLACE "${XME_SRC_DIR}/ports/targets/" "" _TARGET ${_TARGET})
		list (APPEND ${OUTPUT} ${_TARGET})
	endforeach (_TARGET)
endmacro (xme_get_supported_targets)

macro (xme_is_header_file FILENAME OUTPUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_is_header_file()!")
	endif ()

	if (${FILENAME} MATCHES "\\.(h|hh|h\\+\\+|hm|hpp|hxx)$")
		set (${OUTPUT} TRUE)
	else ()
		set (${OUTPUT} FALSE)
	endif ()
endmacro (xme_is_header_file)

macro (xme_is_source_file FILENAME OUTPUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_is_source_file()!")
	endif ()

	if (${FILENAME} MATCHES "\\.(c|C|c\\+\\+|cc|cpp|cxx)$")
		set (${OUTPUT} TRUE)
	else ()
		set (${OUTPUT} FALSE)
	endif ()
endmacro (xme_is_source_file)

macro (xme_is_asm_file FILENAME OUTPUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to macro xme_is_asm_file()!")
	endif ()

	if (${FILENAME} MATCHES "\\.(s|S)$")
		set (${OUTPUT} TRUE)
	else ()
		set (${OUTPUT} FALSE)
	endif ()
endmacro (xme_is_asm_file)

# Macro:      xme_install_rec ( <destination> <prefix> [ <absolute-file-paths> ... ] )
#
# Purpose:    For internal use only. Adds install rules for the given files. Files will be installed
#             into <CMAKE_INSTALL_PREFIX>/<destination>/<given file path without <prefix>>.
#             For every given file path, this function walks through the parent directories and installs
#             all found files, until the <prefix> directory is reached or until a directory with a 
#             CMakeLists.txt is found.
#
# Parameters: <destination> Subdirectory inside installation directory where the given files will be installed.
#             <prefix>      See above.
#             [ <absolute-file-paths> ... ]
#                           Files to install with absolute paths. Must reside somewhere under <prefix>.
# 
macro (xme_install_rec)
	if (${ARGC} LESS 2)
		xme_message (WARNING "Too few arguments to macro xme_install_rec()!")
	endif ()

	if (${ARGC} GREATER 1)   
		set (__FILES__ ${ARGV})
		list(GET __FILES__ 0 __DESTINATION__)
		list(GET __FILES__ 1 __PREFIX__)
		list (REMOVE_AT __FILES__ 0 1)

		# Normalize prefix path (so that string comparisons can be used to check directory equality)
		xme_simplify_path(__PREFIX__ ${__PREFIX__})
		
		# Install every file into the install directory preserving the directory structure 
		foreach (__FILE__ ${__FILES__})
			# Normalize file path so (so that string comparisons can be used to check directory equality)
			xme_simplify_path(__FILE__ ${__FILE__})
			string (FIND ${__FILE__} ${__PREFIX__} __POS__)
			# Ignore file if it is not located in PREFIX:
			#
			# This is needed to avoid an endless loop when walking up the
			# path in the while loop below.
			if (__POS__ EQUAL -1)
				xme_message (WARNING "${__FILE__} is not located in ${__PREFIX__}. Skipped generation of install rule.")
				break()
			endif (__POS__ EQUAL -1)
			
			set (__FILE_IDEST__ "")
			# Remove ${XME_ROOT} & filename (including the last "/") to get path for install destination
			string (REGEX REPLACE "^${__PREFIX__}" "" __FILE_IDEST__ ${__FILE__})
			string (REGEX REPLACE "/[^/]+$" "" __FILE_IDEST__ ${__FILE_IDEST__})
			
			install(FILES ${__FILE__}
					DESTINATION "${__DESTINATION__}${__FILE_IDEST__}")
			
			# Store parent directory in __DIR__
			string (FIND "${__PREFIX__}${__FILE_IDEST__}" "/" __POS__ REVERSE)
			string (SUBSTRING "${__PREFIX__}${__FILE_IDEST__}" 0 ${__POS__} __DIR__)
			
			# Walk through parent directories until __PREFIX__ is reached or a CMakeLists.txt is found
			while (1)
				if (EXISTS "${__DIR__}/CMakeLists.txt")
					break()
				endif (EXISTS "${__DIR__}/CMakeLists.txt")

				if ("${__DIR__}" STREQUAL "${__PREFIX__}")
					break()
				endif ("${__DIR__}" STREQUAL "${__PREFIX__}")

				# Get all contents of current directory (files and directories)
				file (GLOB __DIR_CONTENTS__ "${__DIR__}/*")
				
				# Filter __DIR_CONTENTS__ for files only and store them in __DIR_FILES__
				set (__DIR_FILES__ "")
				foreach (__F__ ${__DIR_CONTENTS__})
					if (NOT IS_DIRECTORY ${__F__})
						set (__DIR_FILES__ ${__DIR_FILES__} ${__F__})
					endif (NOT IS_DIRECTORY ${__F__})
				endforeach (__F__)
				
				# Add install rule for each file
				foreach (__DIR_FILE__ ${__DIR_FILES__})
					set (__DIR_FILE_IDEST__ "")
					# Remove __PREFIX__ & filename (including the last "/") to get path for install destination
					string (REGEX REPLACE "^${__PREFIX__}" "" __DIR_FILE_IDEST__ ${__DIR_FILE__})
					string (REGEX REPLACE "/[^/]+$" "" __DIR_FILE_IDEST__ ${__DIR_FILE_IDEST__})

					install(FILES ${__DIR_FILE__}
							DESTINATION	"${__DESTINATION__}${__DIR_FILE_IDEST__}")
				endforeach (__DIR_FILE__)
				
				# Store parent directory in __DIR__
				set (__OLDDIR__ ${__DIR__})
				string (FIND "${__DIR__}" "/" __POS__ REVERSE)
				string (SUBSTRING "${__DIR__}" 0 ${__POS__} __DIR__)
				# Check if __DIR__ changed
				if ("${__DIR__}" STREQUAL "${__OLDDIR__}")
					xme_message (ERROR "Entered infinite loop when trying to install file: '${__FILE__}' One possible error is that the file is not located in the prefix directory or one of its subdirectories. Prefix directory is: '${__PREFIX__}'")
					break()
				endif ("${__DIR__}" STREQUAL "${__OLDDIR__}")
			endwhile (1)
			
    	endforeach (__FILE__)
    endif (${ARGC} GREATER 1)
endmacro (xme_install_rec)

# Define some useful macros
function (xme_add_subdirectory DIRNAME)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to function xme_add_subdirectory()!")
	endif ()

	# Handle optional parameters
	set (_USE_ADD_SUBDIRECTORY FALSE)
	set (_CMAKE_SCRIPT_NAME "CMakeLists.txt")
	if (${ARGC} GREATER 1)
		set (_USE_ADD_SUBDIRECTORY ${ARGV1})
		if (${ARGC} GREATER 2)
			set (_CMAKE_SCRIPT_NAME ${ARGV2})
			if (${_USE_ADD_SUBDIRECTORY} AND NOT (${_CMAKE_SCRIPT_NAME} STREQUAL "CMakeLists.txt"))
				xme_message (FATAL_ERROR "xme_add_subdirectory: A custom script name is currently only supported in 'include' mode.")
			endif (${_USE_ADD_SUBDIRECTORY} AND NOT (${_CMAKE_SCRIPT_NAME} STREQUAL "CMakeLists.txt"))
		endif (${ARGC} GREATER 2)
	endif (${ARGC} GREATER 1)

	if (IS_ABSOLUTE ${DIRNAME})
		xme_get (__XME_OLD_SOURCE_DIR__ PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
		xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR "${DIRNAME}")
		
		# Store dir in global property for later use
	    xme_append (PROPERTY_GLOBAL XME_SOURCE_DIR_LIST "${DIRNAME}" UNIQUE)
	    
		if (${_USE_ADD_SUBDIRECTORY})
			xme_message (DEBUG "--> Entering directory '${DIRNAME}' using add_subdirectory().")

			# Derive valid relative binary dir from absolute source dir specification
			# 1. Abreviate  with xme
			string (REGEX REPLACE "^${XME_ROOT}" "XME_ROOT" _BIN_SUBDIR ${DIRNAME})
			if (${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
				# 2. On windows, change c: to c_ etc.
				string (REGEX REPLACE ":" "_" _BIN_SUBDIR ${_BIN_SUBDIR})
			endif (${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows")
			add_subdirectory (${DIRNAME} "${CMAKE_CURRENT_BINARY_DIR}/${_BIN_SUBDIR}")
		else (${_USE_ADD_SUBDIRECTORY})
			xme_message (DEBUG "--> Entering directory '${DIRNAME}' using include().")
			include ("${DIRNAME}/${_CMAKE_SCRIPT_NAME}")
		endif (${_USE_ADD_SUBDIRECTORY})
		
		# Store directory for use after if
		xme_get (__DIR_ABSOLUTE__ PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
		
		xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR "${__XME_OLD_SOURCE_DIR__}")
		xme_message (DEBUG "<-- leaving directory '${DIRNAME}'.")
	else (IS_ABSOLUTE ${DIRNAME})
		xme_get (__XME_OLD_SOURCE__ PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
		set (__XME_NEW_SOURCE_DIR__ "${__XME_OLD_SOURCE__}/${DIRNAME}")
		xme_simplify_path (__XME_NEW_SOURCE_DIR__ "${__XME_NEW_SOURCE_DIR__}")
		xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR "${__XME_NEW_SOURCE_DIR__}")
		
		# Store dir in global property for later use
	    xme_append (PROPERTY_GLOBAL XME_SOURCE_DIR_LIST "${__XME_NEW_SOURCE_DIR__}" UNIQUE)
		
		if (${_USE_ADD_SUBDIRECTORY})
			xme_message (DEBUG "--> Entering directory '${__XME_NEW_SOURCE_DIR__}' using add_subdirectory().")
			add_subdirectory ("${__XME_NEW_SOURCE_DIR__}" "${CMAKE_CURRENT_BINARY_DIR}/${DIRNAME}")
		else (${_USE_ADD_SUBDIRECTORY})
			xme_message (DEBUG "--> Entering directory '${__XME_NEW_SOURCE_DIR__}' using include().")
			include ("${__XME_NEW_SOURCE_DIR__}/${_CMAKE_SCRIPT_NAME}")
		endif (${_USE_ADD_SUBDIRECTORY})
		
		#Store directory for use after if
		xme_get (__DIR_ABSOLUTE__ PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
		
		xme_message (DEBUG "<-- leaving directory '${__XME_NEW_SOURCE_DIR__}'.")
		xme_set (PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR "${__XME_OLD_SOURCE__}") 
		# Note: xme_add_subdirectory() is a function and has a local scope for variables
	endif (IS_ABSOLUTE ${DIRNAME})

	# Add install rule for all files in directory (excluding directories)
	file (GLOB __DIR_CONTENTS__ "${__DIR_ABSOLUTE__}/*")
	set (__FILES__ "")
	foreach (__F__ ${__DIR_CONTENTS__})
		if (NOT IS_DIRECTORY ${__F__})
			set (__FILES__ ${__FILES__} ${__F__})
		endif (NOT IS_DIRECTORY ${__F__})
	endforeach (__F__)
	
	foreach (__FILE__ ${__FILES__})
		set (__FILE_IDEST__ "")
		# Remove ${XME_ROOT} & filename (including the last '/') to get path for install destination
		string (REGEX REPLACE "^${XME_ROOT}" "" __FILE_IDEST__ ${__FILE__})
		string (REGEX REPLACE "/[^/]+$" "" __FILE_IDEST__ ${__FILE_IDEST__})
		
		install(FILES ${__FILE__}
				DESTINATION "src/${__FILE_IDEST__}")
	endforeach (__FILE__)
	
	# Ensure that include paths defined in subdirectories are also
	# available at the current scope.
	#
	# TODO: If the structure looks like this: p/A, p/B, then the directories
	#       defined in p/A will also be applied to p/B. The correct solution
	#       would involve managing a stack of include directories.
	xme_get (_INCLUDE_DIRECTORIES PROPERTY_GLOBAL XME_INCLUDE_DIRECTORIES)

	foreach (_INCLUDE_DIRECTORY ${_INCLUDE_DIRECTORIES})
		# xme_include_directory ensures absolute paths
		#xme_message (DEBUG "Pulled include directory '${_INCLUDE_DIRECTORY}' from '${DIRNAME}'.")
		include_directories (${_INCLUDE_DIRECTORY})
	endforeach (_INCLUDE_DIRECTORY)

	# Ensure that library paths defined in subdirectories are also
	# available at the current scope.
	#
	# TODO: If the structure looks like this: p/A, p/B, then the directories
	#       defined in p/A will also be applied to p/B. The correct solution
	#       would involve managing a stack of library directories.
	xme_get (_LIBRARY_DIRECTORIES PROPERTY_GLOBAL XME_LIBRARY_DIRECTORIES)

	foreach (_LIBRARY_DIRECTORY ${_LIBRARY_DIRECTORIES})
		# xme_library_directory ensures absolute paths
		#xme_message (DEBUG "Pulled library directory '${_LIBRARY_DIRECTORY}' from '${DIRNAME}'.")
		link_directories (${_LIBRARY_DIRECTORY})
	endforeach (_LIBRARY_DIRECTORY)

	# Ensure that defines specified in subdirectories are also
	# available at the current scope.
	#
	# TODO: If the structure looks like this: p/A, p/B, then the defines
	#       specified in p/A will also be applied to p/B. The correct solution
	#       would involve managing a stack of definitions.
	xme_get (_DEFINITIONS PROPERTY_GLOBAL XME_DEFINITIONS)

	foreach (_DEFINITION ${_DEFINITIONS})
		#xme_message (DEBUG "Pulled definition '${_DEFINITION}' from '${DIRNAME}'.")
		add_definitions (${_DEFINITION})
	endforeach (_DEFINITION)
	
endfunction (xme_add_subdirectory)


# Macro:      xme_get_sources_for_component ( <_COMP_NAME> <_OUTPUT> )
#
# Purpose:    returns all source files, which are part of a component and all source files, on 
#             which this component depends (directly and indirectly)
#
#             This function starts at the given component and gathers all of its source files by
#             taking its dependencies into account. It will look recursively through all of the
#             component's dependencies and sums up all source files. 
#
# Parameters: <_COMP_NAME>    the name of the component, for which the sources shall be searched
#
#             <_OUTPUT>       this paramter will be 'returned'. It contains a list of all the 
#                             source files
# 
function (xme_get_sources_for_component _COMP_NAME _OUTPUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to function xme_get_sources_for_component()!")
	endif ()

	# reset the _XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST property
	xme_set (PROPERTY_GLOBAL _XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST "")
	# call the actual function
	_xme_get_sources_for_component ("${_COMP_NAME}" __OUTPUT__)
	set(${_OUTPUT} "${__OUTPUT__}" PARENT_SCOPE)
endfunction(xme_get_sources_for_component)


function (_xme_get_sources_for_component _COMP_NAME _OUTPUT)
	if (${ARGC} GREATER 2)
		xme_message (WARNING "Too many arguments to function _xme_get_sources_for_component()!")
	endif ()

	string (TOUPPER "${_COMP_NAME}" __COMP_NAME_UPPER__)

	# return nothing for components with empty name
	if("${__COMP_NAME_UPPER__}" STREQUAL "")
		set(${_OUTPUT} "" PARENT_SCOPE)
		return()
	endif("${__COMP_NAME_UPPER__}" STREQUAL "")
	
	# return nothing for components, which have already been handled
	xme_get (__XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST__ PROPERTY_GLOBAL _XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST)
	list(FIND __XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST__ ${__COMP_NAME_UPPER__} _FOUND)
	if(_FOUND GREATER -1)
		set(${_OUTPUT} "" PARENT_SCOPE)
		return()
	endif(_FOUND GREATER -1)

	# get the global properties for the current component
#	xme_get (_HEADERS       PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_HEADERS")
	xme_get (_SOURCES_MIXED PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_SOURCES")
	xme_get (_DEPENDENCIES  PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_DEPENDS")
	xme_get (_LINK_COMPS    PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_LINK_COMPONENT")

	# mark the current component as 'already done' to avoid handling it again
	list (APPEND __XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST__ "${__COMP_NAME_UPPER__}")
	xme_set (PROPERTY_GLOBAL _XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST "${__XME_GET_SOURCES_FOR_COMPONENT_DONE_LIST__}")

	# recursive call to all components, on which the current component depends on
	foreach (_DEPENDENCY IN ITEMS ${_DEPENDENCIES})
		_xme_get_sources_for_component("${_DEPENDENCY}" _DEPENDENCY_SOURCES)
		list (APPEND _SOURCES_MIXED "${_DEPENDENCY_SOURCES}")
	endforeach(_DEPENDENCY)

	# recursive call to all components, which have to be linked to the current component
	foreach (_LINK_COMP IN ITEMS ${_LINK_COMPS})
		_xme_get_sources_for_component("${_LINK_COMP}" _LINK_SOURCES)
		list (APPEND _SOURCES_MIXED "${_LINK_SOURCES}")
	endforeach(_LINK_COMP)
		
	list(LENGTH _SOURCES_MIXED _LENGTH)
	if(_LENGTH GREATER 0)
		# clean up source list
		# remove duplicate items
		list(REMOVE_DUPLICATES _SOURCES_MIXED)
		# remove empty items
		list(REMOVE_ITEM _SOURCES_MIXED "")
	endif(_LENGTH GREATER 0)
	
	set(${_OUTPUT} "${_SOURCES_MIXED}" PARENT_SCOPE)

	xme_message (DEBUG "component ${__COMP_NAME_UPPER__} has c-sources >>${_SOURCES_C}<<, asm-sources >>${_SOURCES_ASM}<<, dependencies >>${_DEPENDENCIES}<<, links >>${_LINKS}<<, link_comps >>${_LINK_COMPS}<<")
	
endfunction (_xme_get_sources_for_component)

macro (_xme_get_build_options_files OUTPUT DIRECTORY)
    set (${OUTPUT})
    if (EXISTS "${DIRECTORY}/Options.cmake")
        list (APPEND ${OUTPUT} "${DIRECTORY}/Options.cmake")
    endif ()
endmacro (_xme_get_build_options_files)

macro (_xme_get_application_cmake_files OUTPUT DIRECTORY)
    set (${OUTPUT})
    if (EXISTS "${DIRECTORY}/CMakeApplication.txt")
        list (APPEND ${OUTPUT} "${DIRECTORY}/CMakeApplication.txt")
    endif()
endmacro (_xme_get_application_cmake_files)

function (_xme_component_callback_register COMPONENT TYPE CALLBACK)
    # Check type
    if (NOT ${TYPE} STREQUAL "BEFORE_USE" AND
        NOT ${TYPE} STREQUAL "BEFORE_ADD_TARGET" AND
        NOT ${TYPE} STREQUAL "BEFORE_ADD_UNITTESTS" AND
        NOT ${TYPE} STREQUAL "BEFORE_ADD_TEST" AND
        NOT ${TYPE} STREQUAL "BEFORE_LINK_TEST" AND
        NOT ${TYPE} STREQUAL "AFTER_ADD_TEST" AND
        NOT ${TYPE} STREQUAL "AFTER_ADD_UNITTESTS" AND
        NOT ${TYPE} STREQUAL "AFTER_USE")
        xme_message (FATAL_ERROR "Invalid xme_add_component() callback type '${TYPE}' for callback '${CALLBACK}'!")
    endif ()
    
    string (TOUPPER ${COMPONENT} COMPONENT_UPPER)
    
    xme_callback_register ("XME_COMPONENT_${COMPONENT_UPPER}_CALLBACK_${TYPE}" ${CALLBACK})
endfunction (_xme_component_callback_register)

function (_xme_component_callback_execute COMPONENT TYPE)
    string (TOUPPER ${COMPONENT} COMPONENT_UPPER)
    xme_callback_execute ("XME_COMPONENT_${COMPONENT_UPPER}_CALLBACK_${TYPE}" ${ARGN})
endfunction (_xme_component_callback_execute)

function (_xme_resolve_template_variables COMPONENT TEMPLATE RESULT)
    string (REGEX MATCHALL "\\$\\[[^]]+\\]" MATCHLIST ${TEMPLATE})
    set (${RESULT} ${TEMPLATE})
    foreach (MATCH ${MATCHLIST})
        # Strip "$[" and "]"
        string (REGEX REPLACE "^\\$\\[(.*)\\]$" "\\1" MATCH ${MATCH})
        if (NOT ${MATCH})
            xme_message (WARNING "Lazily evaluated variable '${MATCH}', which is required for component '${COMPONENT}', is not defined!")
        else ()
            string (REPLACE "$[${MATCH}]" "${${MATCH}}" ${RESULT} ${${RESULT}})
        endif ()
    endforeach (MATCH)

    # Return result
    set (${RESULT} ${${RESULT}} PARENT_SCOPE)
endfunction (_xme_resolve_template_variables)

# Macro:      xme_add_component ( <component-name>
#                 [ <header.h> ... [ GENERATED ] | <source.c> ... [ GENERATED ] | <dependency> ... ]
#                 [ HEADERS <header1> [ <header2> ...] [ GENERATED ] ]
#                 [ SOURCES <source1> [ <source2> ... ] [ GENERATED ] ]
#                 [ DEPENDS <dependency1> [ <dependency2> ... ] ]
#                 [ PACKAGES <package1> [ <package2> ... ] ]
#                 [ TARGETS ( [ VALUE ] <target1> [ <target2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ]
#                 [ INCLUDE_PATH ( [ VALUE ] <includepath1> [ <includepath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ]
#                 [ LIBRARY_PATH ( [ VALUE ] <librarypath1> [ <librarypath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ]
#                 [ LINK [ CONFIGURATION ( DEBUG | OPTIMIZED | GENERAL ) ] ( [ VALUE ] <library1> [ <library2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ]
#                 [ PROPERTIES <property1> <value1> [ <property2> <value2> ... ] ]
#                 [ CALLBACK <step1> <name1> [ <step2> <name2> ... ] ]
#             )
#
# Purpose:    Defines a software component from a number of source and/or header files and dependencies.
#             Component names should follow the naming conventions specified in the description of the <component-name>
#             parameter below.
#             If multiple components with the same name are defined across the build system, their properties
#             (e.g., source files, header files, dependencies) will be merged in order of definition.
#             This is useful for defining a general interface for a component in one directory of the
#             build system, while the (possibly platform dependent) implementation is defined in a
#             different directory. For examples, see the "xme/hal" directory (generic header files) and
#             the subdirectories of "xme/ports" (platform dependent implementations).
#
#             For each software component with at least one source file, a static linker library will be built.
#             Components with only header files or no files at all are considered "virtual" (such components can
#             serve as an abstraction for a number of dependencies, for example). Since such components do not generate
#             any target files (i.e., static linker libraries), they will not be linked against when referenced by
#             other components (however their dependencies in turn will be considered for linking).
#
# Parameters: <component-name>           Name of the software component. You should use the following naming conventions:
#                                         - Name should only contain the following types of characters: "a-z", "0-9", "_".
#                                         - Name should start with "xme_<level>_", where <level> is one of the following:
#                                            * "adv" if the component is a high-level, platform independent component with
#                                              no direct interaction with the hardware abstraction layer (HAL).
#                                            * "prim" if the component directly calls function from the hardware
#                                              abstraction layer (HAL).
#                                            * "hal" if the component implements hardware abstraction functionality.
#                                            * "core" if the component provides system-internal functionality (should only
#                                              be used by XME core components).
#
#             <header.h> ... [ GENERATED ]
#                                        Name(s) of C header file(s) to use for building the component (including ".h" suffix).
#                                        All files specified in this way will be automatically interpreted as header files
#                                        if their file extension is ".h". If a file name is followed by the GENERATED keyword,
#                                        the macro will not check the file for existence, which would normally produce a
#                                        warning in case the file does not exist. This can be used for files generated during
#                                        the build system configuration process.
#
#             <source.c> ... [ GENERATED ]
#                                        Name(s) of C source file(s) to use for building the component (including ".c" suffix).
#                                        All files specified in this way will be automatically interpreted as source files
#                                        if their file extension is ".c". If a file name is followed by the GENERATED keyword,
#                                        the macro will not check the file for existence, which would normally produce a
#                                        warning in case the file does not exist. This can be used for files generated during
#                                        the build system configuration process.
#
#             <dependency> ...           Name(s) of other XME component(s) that is/are required for this component to work
#                                        properly. This means that the new component will be linked against the referenced
#                                        component and that include directories of the referenced component will be available
#                                        in the new component. All items specified in this way that have no file extension
#                                        will be automatically interpreted as dependencies.
#
#             HEADERS <header1> [ <header2> ...] [ GENERATED ]
#                                        The "HEADERS" keyword indicates that the following items shall be interpreted as
#                                        header files to use for building the component. If a file name is followed by the
#                                        GENERATED keyword, the macro will not check the file for existence, which would
#                                        normally produce a warning in case the file does not exist. This can be used for
#                                        files generated during the build system configuration process.
#
#             SOURCES <source1> [ <source2> ... ] [ GENERATED ]
#                                        The "SOURCES" keyword indicates that the following items shall be interpreted as
#                                        source files to use for building the component. If a file name is followed by the
#                                        GENERATED keyword, the macro will not check the file for existence, which would
#                                        normally produce a warning in case the file does not exist. This can be used for
#                                        files generated during the build system configuration process.
#
#             DEPENDS <dependency1> [ <dependency2> ... ]
#                                        The "DEPENDS" keyword indicate that the following items shall be interpreted as
#                                        names of other CHROMOSOME components that are required for this component to work
#                                        properly. This means that the new component will be linked against the static
#                                        library of the referenced component and that include directories of the referenced
#                                        component will be available in the new component.
#
#             PACKAGES <package1> [ <package2> ... ]
#                                        The "PACKAGES" keyword indicate that the following items shall be interpreted as
#                                        names of CMake packages that are required for this component to work properly.
#                                        This means that a find_package() will be issued as soon as the new component is used
#                                        in a project. Configuring the build system will fail if the respective package has
#                                        not been found, i.e., the package is marked as "REQUIRED". Variables defined by the
#                                        referenced package can be used as input to the INCLUDE_PATH, LIBRARY_PATH and/or
#                                        LINK parameters (see below).
#
#             TARGETS ( [ VALUE ] <target1> [ <target2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] )
#                                        The "TARGETS" keyword lists additional CMake targets that this component depends on.
#                                        These dependencies are lazily evaluated, that is they are only added when the
#                                        component is actually used and the specified PACKAGES have been successfully
#                                        imported. As such, this option can be used to specify dependencies against
#                                        targets from external projects, for example. The value is interpreted as follows:
#                                         - If the "VALUE_OF" keyword is present in front of an item, the name of a variable
#                                           containing the name of one or multiple target names to depend on.
#                                           This is useful if the actual value is not know during initial configuration of the
#                                           build system, for example because it if only valid after a package referenced with
#                                           the "PACKAGES" keyword has been found.
#                                         - If the "TEMPLATE" keyword is present in front of an item, a string possibly
#                                           containing variable names in format "$[<varname>]" that, when the variable name
#                                           is replaced by the respective value, specifies the name of one or multiple target
#                                           names to depend on. As with the "VALUE_OF" keyword, the variable values are
#                                           lazily evaluated (i.e., after loading the respective package).
#                                         - If the "VALUE" keyword if present in front of an item, or neither the "VALUE_OF"
#                                           nor "TEMPLATE" keywords are present in between the "TARGETS" keyword and the
#                                           respective item, the name of a target to depend on.
#
#             INCLUDE_PATH ( [ VALUE ] <includepath1> [ <includepath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] )
#                                        The "INCLUDE_PATH" keyword indicates that the following items shall be interpreted as
#                                        one of the following:
#                                         - If the "VALUE_OF" keyword is present in front of an item, the name of a variable
#                                           containing the name of one or multiple include paths to use when building the
#                                           component.
#                                           This is useful if the actual value is not know during initial configuration of the
#                                           build system, for example because it if only valid after a package referenced with
#                                           the "PACKAGES" keyword has been found.
#                                         - If the "TEMPLATE" keyword is present in front of an item, a string possibly
#                                           containing variable names in format "$[<varname>]" that, when the variable name
#                                           is replaced by the respective value, specifies the name of one or multiple include
#                                           paths to use when building the component. As with the "VALUE_OF" keyword, the
#                                           variable values are lazily evaluated (i.e., after loading the respective package).
#                                         - If the "VALUE" keyword if present in front of an item, or neither the "VALUE_OF"
#                                           nor "TEMPLATE" keywords are present in between the "INCLUDE_PATH" keyword and the
#                                           respective item, the name of an include path to use when building the component.
#
#             LIBRARY_PATH ( [ VALUE ] <librarypath1> [ <librarypath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] )
#                                        The "LIBRARY_PATH" keyword indicate that the following items shall be interpreted as
#                                        one of the following:
#                                         - If the "VALUE_OF" keyword is present in front of an item, the name of a variable
#                                           containing the name of one or multiple library paths to use when building the
#                                           component.
#                                           This is useful if the actual value is not know during initial configuration of the
#                                           build system, for example because it if only valid after a package referenced with
#                                           the "PACKAGES" keyword has been found.
#                                         - If the "TEMPLATE" keyword is present in front of an item, a string possibly
#                                           containing variable names in format "$[<varname>]" that, when the variable name
#                                           is replaced by the respective value, specifies the name of one or multiple library
#                                           paths to use when building the component. As with the "VALUE_OF" keyword, the
#                                           variable values are lazily evaluated (i.e., after loading the respective package).
#                                         - If the "VALUE" keyword if present in front of an item, or neither the "VALUE_OF"
#                                           nor "TEMPLATE" keywords are present in between the "LIBRARY_PATH" keyword and the
#                                           respective item, the name of an library path to use when building the component.
#
#             LINK [ CONFIGURATION ( DEBUG | OPTIMIZED | GENERAL ) ] ( [ VALUE ] <library1> [ <library2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] )
#                                        The "LINK" keyword indicates that the following items shall be interpreted as on of
#                                        the following:
#                                         - If the "VALUE_OF" keyword is present in front of an item, the name of a variable
#                                           containing the name of one or multiple linker libraries (static or dynamic) to use
#                                           when building the component.
#                                           This is useful if the actual value is not know during initial configuration of the
#                                           build system, for example because it if only valid after a package referenced with
#                                           the "PACKAGES" keyword has been found.
#                                         - If the "TEMPLATE" keyword is present in front of an item, a string possibly
#                                           containing variable names in format "$[<varname>]" that, when the variable name
#                                           is replaced by the respective value, specifies the name of one or multiple linker
#                                           libraries (static or dynamic) to use when building the component. As with the
#                                           "VALUE_OF" keyword, the variable values are lazily evaluated (i.e., after loading
#                                           the respective package).
#                                         - If the "VALUE" keyword if present in front of an item, or neither the "VALUE_OF"
#                                           nor "TEMPLATE" keywords are present in between the "LINK" keyword and the respective
#                                           item, the name of a linker library (static or dynamic) to use when building the
#                                           component.
#                                         - If the "CONFIGURATION" keyword is present (in front of the VALUE or VALUE_OF keyword
#                                           or directly following the "LINK" keyword), then the specified linked libraries shall
#                                           apply to the respective configurations:
#                                            * "DEBUG" indicates that the linker libraries only apply to the Debug configuration
#                                              (or to configurations named in the DEBUG_CONFIGURATIONS global property if it is
#                                              set).
#                                            * "OPTIMIZED" indicates that the linker libraries apply to all other configurations.
#                                            * The "GENERAL" keyword corresponds to all configurations and hence has the same
#                                              effect than not using the "CONFIGURATION" keyword at all.
#
#             PROPERTIES <property1> <value1> [ <property2> <value2> ... ]
#                                        The "PROPERTIES" keyword is used to specify target-specific CMake properties for an XME
#                                        component. <propertyN> indicates the name of the property to set and <valueN> the value
#                                        to set. Typical property names are COMPILE_DEFINITIONS, COMPILE_FLAGS and LINK_FLAGS.
#                                        For a full list of properties, see the documentation of CMake or the "Properties on Targets"
#                                        section of `cmake --help-properties`. If the same property name is set multiple times, the
#                                        respective last value counts.
#
#             CALLBACK <type1> <name1> [ <type2> <name2> ... ]
#                                        The "CALLBACK" keyword can be used to specify the names of user-defined CMake macros or
#                                        callback functions that are to be triggered when the respective step is executed.
#                                        <step> can be one of the following:
#                                         - BEFORE_USE: Triggered when a component is referenced for the first time, before any
#                                           other operations are performed. The callback function receives the name of the
#                                           component as parameter.
#                                         - BEFORE_ADD_TARGET: Triggered when a component is referenced for the first time after
#                                           required packages have been found (see PACKAGES keyword) and relevant include and
#                                           library paths have been set up (see INCLUDE_PATH and LIBRARY_PATH properties).
#                                           The callback function receives the name of the component as parameter.
#                                         - BEFORE_ADD_UNITTESTS: Triggered before unit tests for a component are added.
#                                           This hook will not be executed if XME_UNITTEST is set to a false value,
#                                           otherwise it will be executed even if the component does not have any unit tests.
#                                           The callback function receives the name of the component as parameter.
#                                         - BEFORE_ADD_TEST: Triggered when a test is about to be added.
#                                           This hook will not be executed if XME_UNITTEST is set to a false value,
#                                           otherwise it will be executed even if the component does not have any unit tests.
#                                           The callback function receives the name of the component and the type of the test
#                                           as parameters.
#                                         - BEFORE_LINK_TEST: Triggered before libraries are linked to the test executable of
#                                           a component. This hook will not be executed if XME_UNITTEST is set to a false value,
#                                           otherwise it will be executed even if the component does not have any unit tests.
#                                           The callback function receives the name of the component and the type of the test
#                                           as parameters.
#                                         - AFTER_ADD_TEST: Triggered when a test has been added. This hook will not be executed
#                                           if XME_UNITTEST is set to a false value, otherwise it will be executed even if the
#                                           component does not have any unit tests. The callback function receives the name of
#                                           the component and the type of the test as parameters.
#                                         - AFTER_ADD_UNITTESTS: Triggered after unit tests for a component are added.
#                                           This hook will not be executed if XME_UNITTEST is set to a false value,
#                                           otherwise it will be executed even if the component does not have any unit tests.
#                                           The callback function receives the name of the component as parameter.
#                                         - AFTER_USE: Triggered when a component is referenced for the first time after all
#                                           necessary processing steps have been completed. The callback function receives
#                                           the name of the component as parameter.
# 
macro (xme_add_component)
	if (${ARGC} LESS 1)
		xme_message (WARNING "Too few arguments to macro xme_add_component()!")
	endif ()

	set (__USAGE__ "Usage: xme_add_component ( <component-name> [ <header.h> ... [ GENERATED ] | <source.c> ... [ GENERATED ] | <dependency> ... ] [ HEADERS <header1> [ <header2> ...] [ GENERATED ] ] [ SOURCES <source1> [ <source2> ... ] [ GENERATED ] ] [ DEPENDS  <dependency1> [ <dependency2> ... ] ] [ PACKAGES <package1> [ <package2> ... ] ] [ TARGETS ( [ VALUE ] <target1> [ <target2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ] [ INCLUDE_PATH ( [ VALUE ] <includepath1> [ <includepath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ] [ LIBRARY_PATH ( [ VALUE ] <librarypath1> [ <librarypath2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ] [ LINK [ CONFIGURATION ( DEBUG | OPTIMIZED | GENERAL ) ] ( [ VALUE ] <library1> [ <library2> ... ] | VALUE_OF <varname1> [ <varname2> ... ] | TEMPLATE <template1> [ <template2> ... ] ) ] [ PROPERTIES <property1> <value1> [ <property2> <value2> ... ] ] [ CALLBACK <type1> <name1> [ <type2> <name2> ... ] ] )")

	if (${ARGC} LESS 1)
		xme_message (FATAL_ERROR ${__USAGE__})
	endif (${ARGC} LESS 1)

	set (__ARGS__ ${ARGV})
	list (GET __ARGS__ 0 __COMP_NAME__)
	list (REMOVE_AT __ARGS__ 0)

    if (__COMP_NAME__ STREQUAL "HEADERS" OR __COMP_NAME__ STREQUAL "SOURCES" OR
        __COMP_NAME__ STREQUAL "DEPENDS" OR __COMP_NAME__ STREQUAL "PACKAGES" OR
        __COMP_NAME__ STREQUAL "TARGETS" OR __COMP_NAME__ STREQUAL "INCLUDE_PATH" OR
        __COMP_NAME__ STREQUAL "LIBRARY_PATH"  OR __COMP_NAME__ STREQUAL "LINK")

        xme_message(FATAL_ERROR ${__USAGE__})
    endif ()

	set (__COMP_NAME_LOWER__ ${__COMP_NAME__})
	string (TOUPPER ${__COMP_NAME__} __COMP_NAME_UPPER__)

	list (LENGTH __ARGS__ __ARGC__)

	xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
	set (__MODE__ 0)
    set (__NEXTINDEX__ 0)
	set (__BYVALUE__ "VALUE")
	set (__HDR__)
	set (__SRC__)
	set (__ASM_SRC__)
	set (__DEP__)
	set (__PKG__)
    set (__TAR__)
	set (__INP__)
	set (__LIP__)
	set (__LNK__)
    set (__PROP__)
	set (__WARN_IF_NEW_MODE__ FALSE)
	set (__EXPECT_CONFIGURATION__ FALSE)
    set (__EXPECT_PROPERTY_KEY__ FALSE)
    set (__EXPECT_PROPERTY_VALUE__ FALSE)
    set (__EXPECT_CALLBACK_TYPE__ FALSE)
    set (__EXPECT_CALLBACK_NAME__ FALSE)
	set (__CONFIGURATION__ "GENERAL")
    set (__PROPERTY_KEY__)
    set (__HAVE_FILENAME__ FALSE)
	foreach (ARG ${__ARGS__})
        # Increment current argument list index
        math (EXPR __NEXTINDEX__ "${__NEXTINDEX__} + 1")

		if (ARG STREQUAL "HDR" OR ARG STREQUAL "SRC" OR ARG STREQUAL "DEP" OR ARG STREQUAL "PKG" OR ARG STREQUAL "INC_PATH" OR ARG STREQUAL "LIB_PATH" OR ARG STREQUAL "LNK" OR ARG STREQUAL "VALUEOF")
			if (ARG STREQUAL "HDR")
				set (NEWARG "HEADERS")
			elseif (ARG STREQUAL "SRC")
				set (NEWARG "SOURCES")
			elseif (ARG STREQUAL "DEP")
				set (NEWARG "DEPENDS")
			elseif (ARG STREQUAL "PKG")
				set (NEWARG "PACKAGES")
			elseif (ARG STREQUAL "INC_PATH")
				set (NEWARG "INCLUDE_PATH")
			elseif (ARG STREQUAL "LIB_PATH")
				set (NEWARG "LIBRARY_PATH")
			elseif (ARG STREQUAL "LNK")
				set (NEWARG "LINK")
			elseif (ARG STREQUAL "VALUEOF")
				set (NEWARG "VALUE_OF")
			endif ()
			xme_message(FATAL_ERROR "Found old-style keyword '${ARG}' in xme_add_component(), please replace by '${NEWARG}'!")
		elseif (ARG STREQUAL "HEADERS")
			set (__MODE__ 1)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "SOURCES")
			set (__MODE__ 2)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "DEPENDS")
			set (__MODE__ 3)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "PACKAGES")
			set (__MODE__ 4)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
        elseif (ARG STREQUAL "TARGETS")
            set (__MODE__ 10)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
            if (__WARN_IF_NEW_MODE__)
                xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
            endif (__WARN_IF_NEW_MODE__)
            set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "INCLUDE_PATH")
			set (__MODE__ 5)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "LIBRARY_PATH")
			set (__MODE__ 6)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "LINK")
			set (__MODE__ 7)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "CONFIGURATION")
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
			if (NOT __MODE__ EQUAL 7)
				xme_message (FATAL_ERROR "Keyword 'CONFIGURATION' used in an invalid context in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'!")
			endif (NOT __MODE__ EQUAL 7)
			set (__EXPECT_CONFIGURATION__ TRUE)
			set (__WARN_IF_NEW_MODE__ TRUE)
        elseif (ARG STREQUAL "PROPERTIES")
            set (__MODE__ 8)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
            if (__WARN_IF_NEW_MODE__)
                xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
            endif (__WARN_IF_NEW_MODE__)
            set (__EXPECT_PROPERTY_KEY__ TRUE)
            set (__WARN_IF_NEW_MODE__ TRUE)
        elseif (ARG STREQUAL "CALLBACK")
            set (__MODE__ 9)
            set (__EXPECT_CONFIGURATION__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__HAVE_FILENAME__ FALSE)
            if (__WARN_IF_NEW_MODE__)
                xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
            endif (__WARN_IF_NEW_MODE__)
            set (__EXPECT_CALLBACK_TYPE__ TRUE)
            set (__WARN_IF_NEW_MODE__ TRUE)
        elseif (ARG STREQUAL "GENERATED")
            if (__MODE__ EQUAL 0 OR __MODE__ EQUAL 1 OR __MODE__ EQUAL 2)
            else ()
                xme_message(FATAL_ERROR "Invalid arguments passed to xme_add_component(): found 'GENERATED' keyword in non-filename section! Maybe you are missing a parameter or is it set to an empty value?")
            endif ()
            
            if (NOT __HAVE_FILENAME__)
                xme_message(FATAL_ERROR "Invalid arguments passed to xme_add_executable(): missing or invalid file name before 'GENERATED' keyword! Maybe you are missing a parameter or is it set to an empty value?")
            endif ()
            
            # The keyword has already been interpreted (see below)
            
            # Reset __HAVE_FILENAME__
            set (__HAVE_FILENAME__ FALSE)
		elseif ("${ARG}" STREQUAL "VALUE")
            set (__HAVE_FILENAME__ FALSE)
			set (__BYVALUE__ "VALUE")
		elseif (ARG STREQUAL "VALUE_OF")
            set (__HAVE_FILENAME__ FALSE)
			set (__BYVALUE__ "VALUE_OF")
        elseif (ARG STREQUAL "TEMPLATE")
            set (__HAVE_FILENAME__ FALSE)
            set (__BYVALUE__ "TEMPLATE")
		elseif (__EXPECT_CONFIGURATION__)
            set (__HAVE_FILENAME__ FALSE)
			if (${ARG} STREQUAL "DEBUG" OR ${ARG} STREQUAL "OPTIMIZED" OR ${ARG} STREQUAL "GENERAL")
				set (__CONFIGURATION__ ${ARG})
			else (${ARG} STREQUAL "DEBUG" OR ${ARG} STREQUAL "OPTIMIZED" OR ${ARG} STREQUAL "GENERAL")
				xme_message (FATAL_ERROR "Invalid value '${ARG}' specified for 'CONFIGURATION' keyword in definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}'!")
			endif (${ARG} STREQUAL "DEBUG" OR ${ARG} STREQUAL "OPTIMIZED" OR ${ARG} STREQUAL "GENERAL")
			set (__EXPECT_CONFIGURATION__ FALSE)
        elseif (__EXPECT_PROPERTY_KEY__)
            # Property key
            set (__HAVE_FILENAME__ FALSE)
            set (__PROPERTY_KEY__ ${ARG})
            set (__EXPECT_PROPERTY_KEY__ FALSE)
            set (__EXPECT_PROPERTY_VALUE__ TRUE)
        elseif (__EXPECT_PROPERTY_VALUE__)
            # Property value
            set (__HAVE_FILENAME__ FALSE)
            xme_append(VARIABLE __PROP__ "${__PROPERTY_KEY__}|${ARG}")
            set (__EXPECT_PROPERTY_VALUE__ FALSE)
            set (__EXPECT_PROPERTY_KEY__ TRUE)
            set (__WARN_IF_NEW_MODE__ FALSE)
        elseif (__EXPECT_CALLBACK_TYPE__)
            # Callback type
            set (__HAVE_FILENAME__ FALSE)
            set (__CALLBACK_TYPE__ ${ARG})
            set (__EXPECT_CALLBACK_TYPE__ FALSE)
            set (__EXPECT_CALLBACK_NAME__ TRUE)
            set (__WARN_IF_NEW_MODE__ TRUE)
        elseif (__EXPECT_CALLBACK_NAME__)
            # Callback name
            set (__HAVE_FILENAME__ FALSE)
            _xme_component_callback_register(${__COMP_NAME_LOWER__} ${__CALLBACK_TYPE__} ${ARG})
            set (__EXPECT_CALLBACK_NAME__ FALSE)
            set (__EXPECT_CALLBACK_TYPE__ TRUE)
            set (__WARN_IF_NEW_MODE__ FALSE)
		else ()
			set (__WARN_IF_NEW_MODE__ FALSE)

			if (IS_ABSOLUTE ${ARG})
				set (_FILE ${ARG})
			else (IS_ABSOLUTE ${ARG})
				set (_FILE ${_XME_CURRENT_SOURCE_DIR}/${ARG})
			endif (IS_ABSOLUTE ${ARG})

            # Look ahead
            set (__NEXTARG__)
            if (${__ARGC__} GREATER ${__NEXTINDEX__})
                list (GET __ARGS__ ${__NEXTINDEX__} __NEXTARG__)
            endif ()

            # Simplify the file path such that files with relative paths
            # actually pointing to the same file are not added twice as
            # source files. For example, 'some/dir/../source.c' and
            # 'some/otherdir/../source.c' point to the same file.
            # In both cases, the result of path simplification would be
            # 'some/source.c'.
            xme_simplify_path (_FILE ${_FILE})

			if (__MODE__ EQUAL 0)
				# Auto-detect item type
				#  - *.c is treated as a source file
				#  - *.h is treated as a header file
				#  - anything else is treated as a component dependency
				xme_is_header_file(${ARG} __FLAG_HEADER__)
				xme_is_source_file(${ARG} __FLAG_SOURCE__)
				xme_is_asm_file(${ARG} __FLAG_ASM__)

				if (__FLAG_HEADER__ OR __FLAG_SOURCE__ OR __FLAG_ASM__)
                    set (__HAVE_FILENAME__ TRUE)

                    if ("${__NEXTARG__}" STREQUAL "GENERATED")
                        # File probably generated, don't test for existence
                    else ()
                        if (NOT EXISTS ${_FILE})
                            if (IS_ABSOLUTE ${ARG})
                                xme_message (WARNING "Unable to find file '${_FILE}' for component '${__COMP_NAME_LOWER__}'!")
                            else ()
                                xme_message (WARNING "Unable to find file '${ARG}' in '${_XME_CURRENT_SOURCE_DIR}' for component '${__COMP_NAME_LOWER__}'!")
                            endif ()
                        endif ()
                    endif ()

					# Add the file to the list of required files in any case (even if it is missing).
					# If the file is missing, a respective error message will be output by CMake
					# when the build system is generated.
					if (__FLAG_HEADER__)
						# Header file
						list (APPEND __HDR__ ${_FILE})
					elseif (__FLAG_SOURCE__)
						# Source file
						list (APPEND __SRC__ ${_FILE})
					elseif (__FLAG_ASM__)
						# Source file
						list (APPEND __ASM_SRC__ ${_FILE})
					else ()
						xme_message (ERROR "Unable to determine type of file '${ARG}' for component '${__COMP_NAME_LOWER__}'!")
					endif ()
				else (__FLAG_HEADER__ OR __FLAG_SOURCE__ OR __FLAG_ASM__)
					# Dependency
                    set (__HAVE_FILENAME__ FALSE)
					list (APPEND __DEP__ ${ARG})
				endif (__FLAG_HEADER__ OR __FLAG_SOURCE__ OR __FLAG_ASM__)
			elseif (__MODE__ EQUAL 1)
				# Header file
                set (__HAVE_FILENAME__ TRUE)

                if ("${__NEXTARG__}" STREQUAL "GENERATED")
                    # File probably generated, don't test for existence
                else ()
                    if (NOT EXISTS ${_FILE})
                        if (IS_ABSOLUTE ${ARG})
                            xme_message (WARNING "Unable to find file '${_FILE}' for component '${__COMP_NAME_LOWER__}'!")
                        else ()
                            xme_message (WARNING "Unable to find file '${ARG}' in '${_XME_CURRENT_SOURCE_DIR}' for component '${__COMP_NAME_LOWER__}'!")
                        endif ()
                    endif ()
                endif ()

				# Add the file to the list of required files in any case (even if it is missing).
				# If the file is missing, a respective error message will be output by CMake
				# when the build system is generated.
				list (APPEND __HDR__ ${_FILE})

			elseif (__MODE__ EQUAL 2)
				# Source file
                set (__HAVE_FILENAME__ TRUE)

                if ("${__NEXTARG__}" STREQUAL "GENERATED")
                    # File probably generated, don't test for existence
                else ()
                    if (NOT EXISTS ${_FILE})
                        if (IS_ABSOLUTE ${ARG})
                            xme_message (WARNING "Unable to find file '${_FILE}' for component '${__COMP_NAME_LOWER__}'!")
                        else ()
                            xme_message (WARNING "Unable to find file '${ARG}' in '${_XME_CURRENT_SOURCE_DIR}' for component '${__COMP_NAME_LOWER__}'!")
                        endif ()
                    endif ()
                endif ()

				# Add the file to the list of required files in any case (even if it is missing).
				# If the file is missing, a respective error message will be output by CMake
				# when the build system is generated.
				list (APPEND __SRC__ ${_FILE})

			elseif (__MODE__ EQUAL 3)
				# Dependency
                set (__HAVE_FILENAME__ FALSE)
				list (APPEND __DEP__ ${ARG})
			elseif (__MODE__ EQUAL 4)
				# Package dependency
                set (__HAVE_FILENAME__ FALSE)
				list (APPEND __PKG__ ${ARG})
			elseif (__MODE__ EQUAL 5)
				# Include path variable name
                set (__HAVE_FILENAME__ FALSE)
				list (APPEND __INP__ ${__BYVALUE__} ${ARG})
				set (__BYVALUE__ "VALUE") # Reset to default
			elseif (__MODE__ EQUAL 6)
				# Library path variable name
                set (__HAVE_FILENAME__ FALSE)
				list (APPEND __LIP__ ${__BYVALUE__} ${ARG})
				set (__BYVALUE__ "VALUE") # Reset to default
            elseif (__MODE__ EQUAL 10)
                # Target name
                set (__HAVE_FILENAME__ FALSE)
                list (APPEND __TAR__ ${__BYVALUE__} ${ARG})
                set (__BYVALUE__ "VALUE") # Reset to default
            else ()
				# Link library
                set (__HAVE_FILENAME__ FALSE)
				list (APPEND __LNK__ "${__BYVALUE__}|${__CONFIGURATION__}" ${ARG})
				set (__CONFIGURATION__ "GENERAL") # Reset to default
				set (__BYVALUE__ "VALUE") # Reset to default
            endif ()
		endif ()
	endforeach (ARG)

    if (__EXPECT_CONFIGURATION__)
        xme_message(FATAL_ERROR "Expected configuration name, but found end of argument list!")
    endif ()

    if (__EXPECT_PROPERTY_VALUE__)
        xme_message(FATAL_ERROR "Expected property value, but found end of argument list!")
    endif ()

	if (__WARN_IF_NEW_MODE__)
		xme_message (WARNING "Definition of component '${__COMP_NAME_LOWER__}' in file '${CMAKE_CURRENT_LIST_FILE}' ended with a keyword! Maybe you are missing a parameter or is it set to an empty value?")
	endif (__WARN_IF_NEW_MODE__)

    xme_defined (__COMP_USED__ PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_USED")
    if (NOT __COMP_USED__)
        xme_set (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_USED" FALSE)
    endif ()

	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_HEADERS" "${__HDR__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_SOURCES" "${__SRC__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_SOURCES" "${__ASM_SRC__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_DEPENDS" "${__DEP__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_PACKAGES" "${__PKG__}" UNIQUE)
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_TARGETS" "${__TAR__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_INCLUDE_PATHS" "${__INP__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_LIBRARY_PATHS" "${__LIP__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_LINKS" "${__LNK__}")
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_PROPERTIES" "${__PROP__}")

	xme_append (PROPERTY_GLOBAL XME_DOCUMENTATION_FILES "${__HDR__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL XME_DOCUMENTATION_FILES "${__SRC__}" UNIQUE)
	xme_append (PROPERTY_GLOBAL XME_DOCUMENTATION_FILES "${__ASM_SRC__}" UNIQUE)
	
	# Add install rule for included files and recursively add all files (but not directories) in
	# This is done to install sources which are not added via xme_add_subdirectory(), e.g.
	# the generic platform specific code which is included by the corresponding platform.
	#
	
	# TODO: Make install prefix an optional argument to xme_add_component() (defaulting
	#       to XME_ROOT). This could be used to install components that are outside of
	#       XME_ROOT, and which are currently not installed. (Issue#2057)
	xme_install_rec("src" "${XME_ROOT}" "${__HDR__}" "${__SRC__}" "${__ASM_SRC__}")

    # Add build system related files (currently CMakeLists.txt file and optionally Options.cmake file(s) and component-specific CMake files)
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_BUILDSYSTEM" "${_XME_CURRENT_SOURCE_DIR}/CMakeLists.txt" UNIQUE)
    _xme_get_build_options_files(__OPTIONS__ ${_XME_CURRENT_SOURCE_DIR})
    xme_append (PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_BUILDSYSTEM" "${__OPTIONS__}" UNIQUE)

	# TODO: Use __COMP_NAME_LOWER__ instead of __COMP_NAME__?
	xme_get (_TMP_ PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_HEADERS")
	#xme_message (DEBUG "Updated '${__COMP_NAME__}' header files to: '${_TMP_}'")
	# TODO: Use __COMP_NAME_LOWER__ instead of __COMP_NAME__?
	xme_get (_TMP_ PROPERTY_GLOBAL "${__COMP_NAME_UPPER__}_SOURCES")
	#xme_message (DEBUG "Updated '${__COMP_NAME__}' source files to: '${_TMP_}'")

	# Accumulating sources across multiple directories also with
	# add_subdirectory() since we are using global properties to propagate the values.
	xme_get (__XME_COMPONENTS__ PROPERTY_GLOBAL XME_COMPONENTS)
	list (FIND __XME_COMPONENTS__ ${__COMP_NAME_LOWER__} __XME_COMPONENT_FOUND__)
	if (__XME_COMPONENT_FOUND__ LESS 0)
		# TODO: Use __COMP_NAME_LOWER__ instead?
		xme_message (DEBUG "Found XME component '${__COMP_NAME__}'")
		# TODO: Use __COMP_NAME_LOWER__ instead?
		list (APPEND __XME_COMPONENTS__ ${__COMP_NAME__})
		xme_set (PROPERTY_GLOBAL XME_COMPONENTS "${__XME_COMPONENTS__}")
	endif (__XME_COMPONENT_FOUND__ LESS 0)

endmacro (xme_add_component)

macro (xme_definition)
	# Append definitions to global list
	xme_get (_DEFINITIONS PROPERTY_GLOBAL XME_DEFINITIONS)
	foreach (ARG ${ARGV})
		list (FIND _DEFINITIONS ${ARG} _FOUND)
		if (${_FOUND} EQUAL -1)
			xme_append (PROPERTY_GLOBAL XME_DEFINITIONS "${ARG}")
		endif (${_FOUND} EQUAL -1)
	endforeach (ARG)

	# Ensure that add_definitions() is called with all definitions
	# specified so far. This is important because add_subdirectory() defines
	# a new scope.
	xme_get (_DEFINITIONS PROPERTY_GLOBAL XME_DEFINITIONS)

	foreach (_DEFINITION ${_DEFINITIONS})
		add_definitions(${_DEFINITION})
		xme_message (VERBOSE "Added definition '${_DEFINITION}'.")
	endforeach (_DEFINITION)
endmacro (xme_definition)

macro (xme_include_directory)
	xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
	set (__ARGS__ ${ARGV})
	if (${ARGC} LESS 1)
		set (__ARGS__ ${_XME_CURRENT_SOURCE_DIR})
	endif (${ARGC} LESS 1)

	# Append new include directories to global list of include directories
	# (as absolute path)
	xme_get (_INCLUDE_DIRECTORIES PROPERTY_GLOBAL XME_INCLUDE_DIRECTORIES)
	foreach (ARG ${__ARGS__})
		if (NOT IS_ABSOLUTE ${ARG})
			set (ARG "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
		endif (NOT IS_ABSOLUTE ${ARG})
		xme_simplify_path (ARG "${ARG}")

		list (FIND _INCLUDE_DIRECTORIES ${ARG} _FOUND)
		if (${_FOUND} EQUAL -1)
			xme_append (PROPERTY_GLOBAL XME_INCLUDE_DIRECTORIES "${ARG}" UNIQUE)
		endif (${_FOUND} EQUAL -1)
	endforeach (ARG)

	# Ensure that include_directories() is called with all include directories
	# defined so far. This is important because add_subdirectory() defines
	# a new scope.
	xme_get (_INCLUDE_DIRECTORIES PROPERTY_GLOBAL XME_INCLUDE_DIRECTORIES)

	foreach (_INCLUDE_DIRECTORY ${_INCLUDE_DIRECTORIES})
		include_directories(${_INCLUDE_DIRECTORY})
		xme_message (DEBUG "Added include directory '${_INCLUDE_DIRECTORY}'.")
	endforeach (_INCLUDE_DIRECTORY)
endmacro (xme_include_directory)

macro (xme_lib_directory)
	xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
	set (__ARGS__ ${ARGV})
	if (${ARGC} LESS 1)
		set (__ARGS__ ${_XME_CURRENT_SOURCE_DIR})
	endif (${ARGC} LESS 1)

	# Append new library directories to global list of library directories
	# (as absolute path)
	xme_get (_LIBRARY_DIRECTORIES PROPERTY_GLOBAL XME_LIBRARY_DIRECTORIES)
	foreach (ARG ${__ARGS__})
		if (NOT IS_ABSOLUTE ${ARG})
			set (ARG "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
		endif (NOT IS_ABSOLUTE ${ARG})
		xme_simplify_path (ARG "${ARG}")

		list (FIND _LIBRARY_DIRECTORIES ${ARG} _FOUND)
		if (${_FOUND} EQUAL -1)
			xme_append (PROPERTY_GLOBAL XME_LIBRARY_DIRECTORIES "${ARG}" UNIQUE)
		endif (${_FOUND} EQUAL -1)
	endforeach (ARG)

	# Ensure that link_directories() is called with all link directories
	# defined so far. This is important because add_subdirectory() defines
	# a new scope.
	xme_get (_LIBRARY_DIRECTORIES PROPERTY_GLOBAL XME_LIBRARY_DIRECTORIES)

	foreach (_LIBRARY_DIRECTORY ${_LIBRARY_DIRECTORIES})
		link_directories (${_LIBRARY_DIRECTORY})
		xme_message (DEBUG "Added library directory '${_LIBRARY_DIRECTORY}' at scope '${CMAKE_CURRENT_LIST_FILE}'")
	endforeach (_LIBRARY_DIRECTORY)
endmacro (xme_lib_directory)

# Macro:      xme_add_executable ( <target-name> [ ( <header.h> ... | <source.c> [ GENERATED ] ) ... ] [ ECLUDE_FROM_ALL ] )
#
# Purpose:    Specifies that an executable target is to be built from the given sources and header files.
#             The list of source and header files typically consists only of the main() program with the
#             main loop. All other required software components should be linked to the executable target
#             by using the xme_link_components() macro.
#
# Parameters: <target-name>              Name of the executable target to create. Depending on the
#                                        execution platform, an extension might be appended to the name
#                                        automatically.
#
#             [ ( <header.h> ... | <source.c> [GENERATED] ) ... ]
#                                        Name(s) of C header and source file(s) to use for building the
#                                        respective target. If a file name is followed by the GENERATED
#                                        keyword, the macro will not check the file for existence, which
#                                        would normally produce a warning in case the file does not exist.
#                                        This can be used for files generated during the build system
#                                        configuration process.
#
#             [ ECLUDE_FROM_ALL ]        When specified, the generated target is excluded from the "all"
#                                        target (i.e., it is not automatically built when the whole
#                                        project is built).
# 
macro (xme_add_executable TARGET)
	set (__FILES__ ${ARGV})
	list (REMOVE_AT __FILES__ 0)

	xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)

	# Parse which files are generated (marked with 'GENERATED' keyword)
	# and which should exist already.
	set (__OLDFILE__ "")
	set (__GENERATED_LIST__ "")
	set (__EXISTING_LIST__ "")
	set (__EXCLUDE_FROM_ALL__ "")
	foreach (__FILE__ ${__FILES__})
		if ("${__FILE__}" STREQUAL "EXCLUDE_FROM_ALL")
			set (__EXCLUDE_FROM_ALL__ "EXCLUDE_FROM_ALL")
			set (__OLDFILE__ "")
		elseif ("${__FILE__}" STREQUAL "GENERATED")
			if ("${__OLDFILE__}" STREQUAL "")
				xme_message(FATAL_ERROR "Invalid arguments passed to xme_add_executable(): missing file name before 'GENERATED' keyword! Maybe you are missing a parameter or is it set to an empty value?")
			else ("${__OLDFILE__}" STREQUAL "")

				# File probably generated
				if (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
					# Leave file name as-is
					list (APPEND __GENERATED_LIST__ "${__OLDFILE__}")
				else (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
					if (IS_ABSOLUTE "${__OLDFILE__}")
						# File name is already absolute, leave as-is
						list (APPEND __GENERATED_LIST__ "${__OLDFILE__}")
					else (IS_ABSOLUTE "${__OLDFILE__}")
						# File name is relative and source dir is set, prepend source dir
						list (APPEND __GENERATED_LIST__ "${_XME_CURRENT_SOURCE_DIR}/${__OLDFILE__}")
					endif (IS_ABSOLUTE "${__OLDFILE__}")
				endif (${_XME_CURRENT_SOURCE_DIR} EQUAL "")

			endif ("${__OLDFILE__}" STREQUAL "")
			set (__OLDFILE__ "")
		else ()
			if (NOT "${__OLDFILE__}" STREQUAL "")

				# File should exist
				if (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
					# Leave file name as-is
					list (APPEND __EXISTING_LIST__ "${__OLDFILE__}")
				else (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
					if (IS_ABSOLUTE "${__OLDFILE__}")
						# File name is already absolute, leave as-is
						list (APPEND __EXISTING_LIST__ "${__OLDFILE__}")
					else (IS_ABSOLUTE "${__OLDFILE__}")
						# File name is relative and source dir is set, prepend source dir
						list (APPEND __EXISTING_LIST__ "${_XME_CURRENT_SOURCE_DIR}/${__OLDFILE__}")
					endif (IS_ABSOLUTE "${__OLDFILE__}")
				endif (${_XME_CURRENT_SOURCE_DIR} EQUAL "")

			endif (NOT "${__OLDFILE__}" STREQUAL "")
			set (__OLDFILE__ ${__FILE__})
		endif ()
	endforeach (__FILE__)

	# Handle the last file
	if (NOT "${__OLDFILE__}" STREQUAL "")
		set (__FILES2__ "${__FILES__}")
		list (REVERSE __FILES2__)
		list (GET __FILES2__ 0 __LASTFILE__)
		if (NOT "${__LASTFILE__}" STREQUAL "GENERATED")

			# File should exist
			if (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
				# Leave file name as-is
				list (APPEND __EXISTING_LIST__ "${__OLDFILE__}")
			else (${_XME_CURRENT_SOURCE_DIR} EQUAL "")
				if (IS_ABSOLUTE "${__OLDFILE__}")
					# File name is already absolute, leave as-is
					list (APPEND __EXISTING_LIST__ "${__OLDFILE__}")
				else (IS_ABSOLUTE "${__OLDFILE__}")
					# File name is relative and source dir is set, prepend source dir
					list (APPEND __EXISTING_LIST__ "${_XME_CURRENT_SOURCE_DIR}/${__OLDFILE__}")
				endif (IS_ABSOLUTE "${__OLDFILE__}")
			endif (${_XME_CURRENT_SOURCE_DIR} EQUAL "")

		endif (NOT "${__LASTFILE__}" STREQUAL "GENERATED")
	endif (NOT "${__OLDFILE__}" STREQUAL "")

	# Debug output for generated file names
	foreach (__FILE__ ${__GENERATED_LIST__})
		xme_message (DEBUG "Expecting file '${__FILE__}' for target '${TARGET}' to be generated during the build process.")
	endforeach (__FILE__)

	# Check files that should exist for existence
	foreach (__FILE__ ${__EXISTING_LIST__})
		if (NOT EXISTS "${__FILE__}")
			xme_message (FATAL_ERROR "Unable to find file '${__FILE__}' for executable '${TARGET}'! Please use the 'GENERATED' keyword if the file is generated during the build process. See the documentation of xme_add_executable() for more information.")
		endif (NOT EXISTS "${__FILE__}")
	endforeach (__FILE__)

	# treating the executable target as a component and adding its files to the global SOURCES list
	string (TOUPPER ${TARGET} TARGET_UPPER)
	list (APPEND __GENERATED_AND_EXISTING_LIST__ ${__GENERATED_LIST__} ${__EXISTING_LIST__})

	foreach (__FILE__ ${__GENERATED_AND_EXISTING_LIST__})
		xme_is_source_file (${__FILE__} _IS_C_FILE)
		xme_is_asm_file (${__FILE__} _IS_ASM_FILE)
		# only add source, if it is a C or an ASM file
		if (_IS_C_FILE OR _IS_ASM_FILE)
			# adding the file to the global SOURCES list for this component
			xme_append (PROPERTY_GLOBAL "${TARGET_UPPER}_SOURCES" "${__FILE__}" UNIQUE)
		endif(_IS_C_FILE OR _IS_ASM_FILE)
	endforeach (__FILE__)

    # Retrieve the set of build options and application CMake files associated with this executable
    _xme_get_build_options_files(__OPTIONS__ ${CMAKE_CURRENT_LIST_DIR})
    _xme_get_application_cmake_files(__APPLICATION_CMAKE__ ${CMAKE_CURRENT_LIST_DIR})

	# Define the target.
	# Add all files to the list of required files in any case (even if they are marked as generated).
	# If a generated file is missing, a respective error message will be output by CMake when the
	# build system is generated.
	add_executable (${TARGET} ${__EXCLUDE_FROM_ALL__} ${__EXISTING_LIST__} ${__GENERATED_LIST__} ${__OPTIONS__} ${__APPLICATION_CMAKE__})
	xme_message (VERBOSE "Generated executable target '${TARGET}'")

    # Add "Build System" source group
    source_group ("Build System" FILES ${__OPTIONS__} ${__APPLICATION_CMAKE__} "${CMAKE_SOURCE_DIR}/CMakeLists.txt")

	# Add install rule for executable if it is not excluded from "all"
	if ("${__EXCLUDE_FROM_ALL__}" STREQUAL "")
		xme_get (__XME_TARGET_OS__ PROPERTY_GLOBAL XME_TARGET_OS)
		xme_get (__XME_TARGET_CPU__ PROPERTY_GLOBAL XME_TARGET_CPU)
		install(TARGETS ${TARGET}
			DESTINATION "bin/${__XME_TARGET_OS__}_${__XME_TARGET_CPU__}")

		# Add install rule for example project directories
		install(DIRECTORY ${_XME_CURRENT_SOURCE_DIR}
				DESTINATION	"src/examples"
				PATTERN "*/build*" EXCLUDE)
	endif ()

    # Create map file
    if (XME_ENABLE_MAP_FILES)
        xme_defined (_XME_LINKER_FLAGS_MAP_FILE_DEFINED PROPERTY_GLOBAL XME_LINKER_FLAGS_MAP_FILE)
        if (${_XME_LINKER_FLAGS_MAP_FILE_DEFINED})
            xme_get (_XME_LINKER_FLAGS_MAP_FILE PROPERTY_GLOBAL XME_LINKER_FLAGS_MAP_FILE)
            set_target_properties (${TARGET} PROPERTIES LINK_FLAGS "${_XME_LINKER_FLAGS_MAP_FILE}${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.map")
            set (_CLEAN_UP ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.map)
        endif (${_XME_LINKER_FLAGS_MAP_FILE_DEFINED})
    endif ()

	# Create firmware image and disassembly
	xme_defined (_XME_IMAGE_FORMAT_DEFINED PROPERTY_GLOBAL XME_IMAGE_FORMAT)
	if (${_XME_IMAGE_FORMAT_DEFINED})
		xme_get (_XME_IMAGE_FORMAT PROPERTY_GLOBAL XME_IMAGE_FORMAT)
		xme_get (_XME_IMAGE_EXTENSION PROPERTY_GLOBAL XME_IMAGE_EXTENSION)
		set (_IMAGE_FILE ${TARGET}.${_XME_IMAGE_EXTENSION})
		set (_DUMP_FILE ${TARGET}_dump.${_XME_IMAGE_EXTENSION})

		# Use tempory file ${TARGET}.tmp in order to be able to attach
		# objcopy run to current executable target
		add_custom_command(TARGET ${TARGET} POST_BUILD
			COMMAND ${CMAKE_OBJCOPY} --output-format=${_XME_IMAGE_FORMAT} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_IMAGE_FILE}
			COMMENT "Objcopying ${TARGET} to ${_IMAGE_FILE}")
		list (APPEND _CLEAN_UP ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${_IMAGE_FILE})

		# If the user requested an image file, also generate a disassembly
		# since chances are good that we are building for an embedded platform
		add_custom_command(TARGET ${TARGET} POST_BUILD
			COMMAND ${CMAKE_OBJDUMP} --all-headers --source ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET} > ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.txt
			COMMENT "Objdumping ${TARGET} to ${TARGET}.txt (-> headers, disassembly)")
		list (APPEND _CLEAN_UP ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${TARGET}.txt)
	endif (${_XME_IMAGE_FORMAT_DEFINED})

	xme_callback_execute("XME_ADD_EXECUTABLE_POSTHOOK" ${TARGET})

	# Ensure that additional files are considered during clean-up
	get_directory_property(_OLD_CLEAN_UP ADDITIONAL_MAKE_CLEAN_FILES)
	list (APPEND _OLD_CLEAN_UP ${_CLEAN_UP})
	set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_OLD_CLEAN_UP}")

endmacro (xme_add_executable)

# Macro:      xme_link_components ( <target-name> [ <component-name> ... ] )
#
# Purpose:    Specifies that the given executable or library target should link
#             against the named software component(s) during the build.
#             This command also takes care of resolving all recursive
#             dependencies between the components to be linked (as specified
#             by the respective call(s) to xme_add_component()) and schedules
#             them for building if necessary.
#
# Parameters: <target-name>              Name of the executable or library
#                                        target that should link against the
#                                        named software components.
#
#             [ <component-name> ... ]   Name(s) of software component(s)
#                                        defined using xme_add_component()
#                                        to link the executable target against.
# 
macro (xme_link_components TARGET)
	_xme_link_components_rec ("" ${ARGV})
endmacro (xme_link_components TARGET)

function (_xme_link_components_rec LINKED_COMPONENTS TARGET)
	set (__DEPENDS__ ${ARGN})

	xme_get (_XME_COMPONENTS PROPERTY_GLOBAL XME_COMPONENTS)
	
	# __DEPENDS__ contains all direct dependencies of the TARGET.
	# Link the TARGET to these dependencies and then recursively
	# call xme_link_components() to add the dependencies of the
	# dependent libraries.

# TODO - Issue #2389
#    # Remove those items from __DEPENDS__ that are already contained in LINKED_COMPONENTS
#    #xme_message(DEBUG "_xme_link_components_rec(), OLD: '${__DEPENDS__}', REMOVE '${LINKED_COMPONENTS}'")
#    foreach (__COMP__ ${LINKED_COMPONENTS})
#        list (REMOVE_ITEM __DEPENDS__ ${__COMP__})
#    endforeach (__COMP__)
#    #xme_message(DEBUG "_xme_link_components_rec(), NEW: '${__DEPENDS__}'")

	foreach (__COMP__ ${__DEPENDS__})
		set (__COMP_LOWER__ ${__COMP__})
		string (TOUPPER ${__COMP__} __COMP_UPPER__)

		list (FIND _XME_COMPONENTS ${__COMP_LOWER__} __INDEX__)
		if (${__INDEX__} LESS 0)
			xme_message (FATAL_ERROR "Target '${TARGET}' depends on unavailable component '${__COMP_LOWER__}'!")
		endif (${__INDEX__} LESS 0)

        if (TARGET ${__COMP_LOWER__})
            xme_message (DEBUG "Skipping build of component '${__COMP_LOWER__}', since it has already been configured for build!")
        else ()

			# If a component does not include any source files (e.g., only header files),
			# we add it to the project, but obviously we do not link against it, because
			# no real library can be generated and the component is more of a structural
			# sort. We call such a library a 'virtual' component.

			xme_get (_HEADERS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_HEADERS)
			xme_get (_SOURCES_ PROPERTY_GLOBAL ${__COMP_UPPER__}_SOURCES)
			xme_get (_PACKAGES_ PROPERTY_GLOBAL ${__COMP_UPPER__}_PACKAGES)
            xme_get (_TARGETS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_TARGETS)
			xme_get (_INCLUDE_PATHS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_INCLUDE_PATHS)
			xme_get (_LIBRARY_PATHS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_LIBRARY_PATHS)
            xme_get (_BUILDSYSTEM_ PROPERTY_GLOBAL ${__COMP_UPPER__}_BUILDSYSTEM)
            xme_get (_PROPERTIES_ PROPERTY_GLOBAL ${__COMP_UPPER__}_PROPERTIES)

            _xme_component_callback_execute(${__COMP_LOWER__} BEFORE_USE ${__COMP_LOWER__})

			# Find required packages
			foreach (__CURPKG__ ${_PACKAGES_})
				xme_message (VERBOSE "Importing package '${__CURPKG__}', which is required by component '${__COMP_LOWER__}'")
				find_package (${__CURPKG__} REQUIRED)
			endforeach (__CURPKG__)

            # Add include paths
            set (__IS_INFO__ TRUE)
            set (__BYVALUE__)
            foreach (__PATH__ ${_INCLUDE_PATHS_})
                if (__IS_INFO__)
                    # ${__PATH__} is either "VALUE", "VALUE_OF" or "TEMPLATE"
                    set (__BYVALUE__ ${__PATH__})
                    set (__IS_INFO__ FALSE)
                else (__IS_INFO__)
                    if (${__BYVALUE__} STREQUAL "VALUE")
                        # ${__PATH__} is the name of the include path
                        xme_message (VERBOSE "Adding include directory '${__PATH__}' to component '${__COMP_LOWER__}'")
                        xme_include_directory (${__PATH__})
                    elseif (${__BYVALUE__} STREQUAL "VALUE_OF")
                        # ${__PATH__} is the name of the variable holding the include path
                        if ()
                            xme_message (WARNING "Lazily evaluated variable '${__PATH__}', which is required for component '${__COMP_LOWER__}', is not defined!")
                        else ()
                            xme_message (VERBOSE "Adding include directory '${${__PATH__}}' to component '${__COMP_LOWER__}'")
                            xme_include_directory (${${__PATH__}})
                        endif ()
                    elseif (${__BYVALUE__} STREQUAL "TEMPLATE")
                        # ${__PATH__} is a template possibly containing the names of variables holding the include path
                        _xme_resolve_template_variables(${__COMP_LOWER__} ${__PATH__} __RESULT__)
                        if (__RESULT__)
                            xme_message (VERBOSE "Adding include directory '${__RESULT__}' to component '${__COMP_LOWER__}'")
                            xme_include_directory (${__RESULT__})
                        endif ()
                    else ()
                        xme_message(WARNING "Invalid __BYVALUE__ value '${__BYVALUE__}'!")
                    endif ()
                    set (__IS_INFO__ TRUE)
                endif (__IS_INFO__)
            endforeach (__PATH__)

            # Add library paths
            set (__IS_INFO__ TRUE)
            set (__BYVALUE__)
            foreach (__PATH__ ${_LIBRARY_PATHS_})
                if (__IS_INFO__)
                    # ${__PATH__} is either "VALUE", "VALUE_OF" or "TEMPLATE"
                    set (__BYVALUE__ ${__PATH__})
                    set (__IS_INFO__ FALSE)
                else (__IS_INFO__)
                    if (${__BYVALUE__} STREQUAL "VALUE")
                        # ${__PATH__} is the name of the library path
                        xme_message (VERBOSE "Adding library directory '${__PATH__}' to component '${__COMP_LOWER__}'")
                        xme_lib_directory (${__PATH__})
                    elseif (${__BYVALUE__} STREQUAL "VALUE_OF")
                        # ${__PATH__} is the name of the variable holding the library path
                        if (NOT ${__PATH__})
                            xme_message (WARNING "Lazily evaluated variable '${__PATH__}', which is required for component '${__COMP_LOWER__}', is not defined!")
                        else ()
                            xme_message (VERBOSE "Adding library directory '${${__PATH__}}' to component '${__COMP_LOWER__}'")
                            xme_lib_directory (${${__PATH__}})
                        endif ()
                    elseif (${__BYVALUE__} STREQUAL "TEMPLATE")
                        # ${__PATH__} is a template possibly containing the names of variables holding the library path
                        _xme_resolve_template_variables(${__COMP_LOWER__} ${__PATH__} __RESULT__)
                        if (__RESULT__)
                            xme_message (VERBOSE "Adding library directory '${__RESULT__}' to component '${__COMP_LOWER__}'")
                            xme_lib_directory (${__RESULT__})
                        endif ()
                    else ()
                        xme_message(WARNING "Invalid __BYVALUE__ value '${__BYVALUE__}'!")
                    endif ()
                    set (__IS_INFO__ TRUE)
                endif (__IS_INFO__)
            endforeach (__PATH__)

            _xme_component_callback_execute(${__COMP_LOWER__} BEFORE_ADD_TARGET ${__COMP_LOWER__})

			list (LENGTH _SOURCES_ __NUM_SOURCES__)
			if (${__NUM_SOURCES__} LESS 1)
				# Neither sources nor dependencies, this component is virtual
				xme_message (VERBOSE "Generated target for virtual XME component '${__COMP_LOWER__}'")
                xme_set (PROPERTY_GLOBAL "XME_COMPONENT_VIRTUAL_${__COMP_UPPER__}" TRUE)
			else (${__NUM_SOURCES__} LESS 1)
				# Either sources or dependencies, this component is 'real'
				xme_message (VERBOSE "Generated target for XME component '${__COMP_LOWER__}'")
                xme_set (PROPERTY_GLOBAL "XME_COMPONENT_VIRTUAL_${__COMP_UPPER__}" FALSE)
			endif (${__NUM_SOURCES__} LESS 1)

            add_library(
                ${__COMP_LOWER__} STATIC
                ${_HEADERS_}
                ${_SOURCES_}
                ${_BUILDSYSTEM_}
            )
            
            if (${__NUM_SOURCES__} LESS 1)
                # For virtual targets, CMake cannot automatically deduce the linker language.
                # In this case, we assume C linker language here, which should be actually
                # indifferent, because there is nothing to link anyway.
                set_target_properties (${__COMP_LOWER__} PROPERTIES "LINKER_LANGUAGE" "C")
            endif ()
            
            # Apply custom target properties
            foreach (_PROPERTY_ ${_PROPERTIES_})
                string (REGEX MATCH "^(.+)\\|(.+)$" _DUMMY_ ${_PROPERTY_})
                set (_KEY_ ${CMAKE_MATCH_1})
                set (_VALUE_ ${CMAKE_MATCH_2})
                xme_message (VERBOSE "Setting target property '${_KEY_}' for XME component '${__COMP_LOWER__}' to '${_VALUE_}'")
                set_target_properties (${__COMP_LOWER__} PROPERTIES ${_KEY_} ${_VALUE_})
            endforeach (_PROPERTY_)
            
            # Add dependencies against other targets
            set (__IS_INFO__ TRUE)
            set (__BYVALUE__)
            foreach (__TARGET__ ${_TARGETS_})
                if (__IS_INFO__)
                    # ${__TARGET__} is either "VALUE", "VALUE_OF" or "TEMPLATE"
                    set (__BYVALUE__ ${__TARGET__})
                    set (__IS_INFO__ FALSE)
                else (__IS_INFO__)
                    if (${__BYVALUE__} STREQUAL "VALUE")
                        # ${__TARGET__} is the name of the target
                        xme_message (VERBOSE "Adding dependency against '${__TARGET__}' to component '${__COMP_LOWER__}'")
                        add_dependencies (${__COMP_LOWER__} ${__TARGET__})
                    elseif (${__BYVALUE__} STREQUAL "VALUE_OF")
                        # ${__TARGET__} is the name of the variable holding the target name
                        if (NOT ${__TARGET__})
                            xme_message (WARNING "Lazily evaluated variable '${__TARGET__}', which is required for component '${__COMP_LOWER__}', is not defined!")
                        else ()
                            xme_message (VERBOSE "Adding dependency against '${${__TARGET__}}' to component '${__COMP_LOWER__}'")
                            add_dependencies (${__COMP_LOWER__} ${${__TARGET__}})
                        endif ()
                    elseif (${__BYVALUE__} STREQUAL "TEMPLATE")
                        # ${__PATH__} is a template possibly containing the names of variables holding the target name
                        _xme_resolve_template_variables(${__COMP_LOWER__} ${__PATH__} __RESULT__)
                        if (__RESULT__)
                            xme_message (VERBOSE "Adding dependency against '${__RESULT__}' to component '${__COMP_LOWER__}'")
                            add_dependencies (${__COMP_LOWER__} ${__RESULT__})
                        endif ()
                    else ()
                        xme_message(WARNING "Invalid __BYVALUE__ value '${__BYVALUE__}'!")
                    endif ()
                    set (__IS_INFO__ TRUE)
                endif (__IS_INFO__)
            endforeach (__TARGET__)
            
            # Add "Build System" source group
            source_group ("Build System" FILES ${_BUILDSYSTEM_})

            xme_set (PROPERTY_GLOBAL "${__COMP_UPPER__}_USED" TRUE)

            if (XME_UNITTEST)
                _xme_component_callback_execute(${__COMP_LOWER__} BEFORE_ADD_UNITTESTS ${__COMP_LOWER__})
                
                xme_get (_UNITTEST_TYPE_LIST PROPERTY_GLOBAL UNITTEST_TYPE_LIST)
                #message(WARNING "Got a list of following types: ${_UNITTEST_TYPE_LIST}")
                foreach(unittest_type ${_UNITTEST_TYPE_LIST})
                    set (TESTNAME "${__COMP_LOWER__}_${unittest_type}Test")
                    xme_get (_UNITTEST_HEADERS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_HEADERS)
                    xme_get (_UNITTEST_SOURCES_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_SOURCES)
                    xme_get (_UNITTEST_LINKS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_LINKS)
                    xme_get (_UNITTEST_INTERNAL_LINKS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_INTERNAL_LINKS)
                    xme_get (_UNITTEST_MOCKS_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_MOCKS)
                    xme_get (_UNITTEST_TIMEOUT_ PROPERTY_GLOBAL ${__COMP_UPPER__}_${unittest_type}_UNITTEST_TIMEOUT)
                    
                    # Notice: We currently rely on the linker to prefer symbols defined
					#         in the current translation unit over symbols from libraries.
					#         As long as this works, we do not need to evaluate the
					#         _UNITTEST_MOCKS_ variable here.
                    
                    list (LENGTH _UNITTEST_SOURCES_ __NUM_UNITTEST_SOURCES__)
                    if (${__NUM_UNITTEST_SOURCES__} LESS 1)
                        xme_message (DEBUG "No unit test of type '${unittest_type}' defined for '${__COMP_LOWER__}'")
                    else (${__NUM_UNITTEST_SOURCES__} LESS 1)
                        _xme_component_callback_execute(${__COMP_LOWER__} BEFORE_ADD_TEST ${__COMP_LOWER__} "${TESTNAME}")
                        
                        xme_add_executable ("${TESTNAME}" ${_UNITTEST_SOURCES_} ${_UNITTEST_HEADERS_})
                        
                        _xme_component_callback_execute(${__COMP_LOWER__} BEFORE_LINK_TEST ${__COMP_LOWER__} "${TESTNAME}")
                        
                        xme_link_components ("${TESTNAME}" ${_UNITTEST_LINKS_})
                        
                        # Ensure that a unit test does not attempt to mock the component it is about to test
                        xme_contains (VARIABLE _UNITTEST_MOCKS_ ${__COMP_LOWER__} __CONTAINS__)
                        if (__CONTAINS__)
                            message (WARNING "Unit test '${TESTNAME}' attempts to mock the component that is about to be tested! This is not supported! Ignoring mock declaration.")
                        endif ()
                        
                        xme_get(__VIRTUAL__ PROPERTY_GLOBAL "XME_COMPONENT_VIRTUAL_${__COMP_UPPER__}")
                        if (NOT __VIRTUAL__)
                            # Link against the component itself if it is not virtual
                            target_link_libraries (${TESTNAME} ${__COMP_LOWER__})
                            xme_message (VERBOSE "Linked target '${TESTNAME}' against XME component '${__COMP_LOWER__}'")
                        else ()
                            # Otherwise, recursively link against the dependencies of the virtual component
                            xme_get (_DEPENDENCIES PROPERTY_GLOBAL "${__COMP_UPPER__}_DEPENDS")
                            if (_DEPENDENCIES)
                                _xme_link_components_rec ("${_UNITTEST_LINKS_}" "${TESTNAME}" ${_DEPENDENCIES})
                            endif ()
                        endif ()
                        
                        # Link against third-party libraries of the component itself
                        _xme_link_thirdparty_dependencies (${TESTNAME} ${__COMP_LOWER__})
                        
                        foreach (_UNITTEST_INTERNAL_LINK_ ${_UNITTEST_INTERNAL_LINKS_})
                            target_link_libraries (${TESTNAME} ${_UNITTEST_INTERNAL_LINK_})
                            xme_message (VERBOSE "Linked target '${TESTNAME}' against library '${_UNITTEST_INTERNAL_LINK_}'")
                        endforeach ()

                        # Add the testsuite as a dependency in case it should be configured.
                        # We specify the full file name to the test result file here to force
                        # gtest to use exactly this file name. Otherwise, it would pick a
                        # unique file name in the output directory, which would break our
                        # approach for detecting testsuites that crash unintentionally.
                        add_dependencies (${TESTNAME} googletest)
                        add_test (${TESTNAME} ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${TESTNAME} "--gtest_output=xml:${PROJECT_BINARY_DIR}/Testing/${TESTNAME}.xml")
                        set_tests_properties(${TESTNAME} PROPERTIES TIMEOUT ${_UNITTEST_TIMEOUT_})
                        
                        _xme_component_callback_execute(${__COMP_LOWER__} AFTER_ADD_TEST ${__COMP_LOWER__} "${TESTNAME}")
                    endif (${__NUM_UNITTEST_SOURCES__} LESS 1)
                endforeach(unittest_type)
                
                _xme_component_callback_execute(${__COMP_LOWER__} AFTER_ADD_UNITTESTS ${__COMP_LOWER__})
            endif (XME_UNITTEST)

			# If the user requested an image file, also dump library headers and disassembly
			# since chances are good that we are building for an embedded platform
			xme_defined (_XME_IMAGE_FORMAT_DEFINED PROPERTY_GLOBAL XME_IMAGE_FORMAT)
			if (${_XME_IMAGE_FORMAT_DEFINED})
				get_target_property(_LIB_FILE ${__COMP_LOWER__} LOCATION)
				add_custom_command(TARGET ${__COMP_LOWER__} POST_BUILD
					COMMAND ${CMAKE_OBJDUMP} --all-headers --source ${_LIB_FILE} > ${_LIB_FILE}.txt
					COMMENT "Objdumping ${_LIB_FILE} to ${_LIB_FILE}.txt (-> headers, disassembly)")
				list (APPEND _CLEAN_UP ${_LIB_FILE}.txt)
			endif (${_XME_IMAGE_FORMAT_DEFINED})

            _xme_component_callback_execute(${__COMP_LOWER__} AFTER_USE ${__COMP_LOWER__})

            xme_callback_execute("XME_USE_COMPONENT_POSTHOOK" ${__COMP_LOWER__})
        endif ()

        xme_get(__VIRTUAL__ PROPERTY_GLOBAL "XME_COMPONENT_VIRTUAL_${__COMP_UPPER__}")
        if (__VIRTUAL__)
            xme_message (VERBOSE "Linked target '${TARGET}' against virtual XME component '${__COMP_LOWER__}'")
        else ()
            xme_message (VERBOSE "Linked target '${TARGET}' against XME component '${__COMP_LOWER__}'")

            # Link the target against all directly specified dependencies
            target_link_libraries (${TARGET} ${__COMP_LOWER__})
        endif ()
        
        # Link against third-party libraries
        _xme_link_thirdparty_dependencies (${TARGET} ${__COMP__})

        # Recursively add the dependencies of the respective dependency
        xme_get (_DEPENDS_ PROPERTY_GLOBAL "${__COMP_UPPER__}_DEPENDS")

		# the component TARGET needs to be linked to __COMP__
		# save this 'dependency' in a global property, as we need it for
		# the xme_get_sources_for_component function
		string (TOUPPER ${TARGET} TARGET_UPPER)
		xme_append (PROPERTY_GLOBAL "${TARGET_UPPER}_LINK_COMPONENT" "${__COMP__}" UNIQUE)
		
        list (FIND LINKED_COMPONENTS ${__COMP_LOWER__} __COMPONENT_FOUND__)
        if (${__COMPONENT_FOUND__} LESS 0)
            list (APPEND LINKED_COMPONENTS ${__COMP_LOWER__})
            _xme_link_components_rec ("${LINKED_COMPONENTS}" ${__COMP__} ${_DEPENDS_})
        endif ()
        set (LINKED_COMPONENTS "${LINKED_COMPONENTS}" PARENT_SCOPE)
	endforeach (__COMP__)

	# Ensure that additional files are considered during clean-up
	get_directory_property(_OLD_CLEAN_UP ADDITIONAL_MAKE_CLEAN_FILES)
	list (APPEND _OLD_CLEAN_UP ${_CLEAN_UP})
	set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${_OLD_CLEAN_UP}")
endfunction (_xme_link_components_rec)

function (_xme_link_thirdparty_dependencies TARGET COMP)
    string (TOUPPER ${COMP} __COMP_UPPER__)

    xme_get (__LINKS__ PROPERTY_GLOBAL "${__COMP_UPPER__}_LINKS")

    set (__IS_INFO__ TRUE)
    set (__BYVALUE__)
    foreach (__LNK__ ${__LINKS__})
        if (__IS_INFO__)
            # ${__LNK__} is of the form "<by-value>|<configuration>", where
            #  - <by-value> ("VALUE", "VALUE_OF" or "TEMPLATE") indicates how to treat the subsequent value
            #  - <configuration> is the name of the configuration (DEBUG, OPTIMIZED or GENERAL)
            #    where the linker library is to be applied
            string (REGEX REPLACE "\\|.*$" "" __BYVALUE__ ${__LNK__})
            string (REGEX REPLACE "^[^\\|]+\\|" "" __CONFIGURATION__ ${__LNK__})

            if (${__BYVALUE__} STREQUAL "VALUE" OR ${__BYVALUE__} STREQUAL "VALUE_OF" OR ${__BYVALUE__} STREQUAL "TEMPLATE")
            else ()
                message (FATAL_ERROR "Invalid value '${__BYVALUE__}' for __BYVALUE__ passed in linker libraries property!")
            endif ()

            if (__CONFIGURATION__ STREQUAL "DEBUG" OR __CONFIGURATION__ STREQUAL "OPTIMIZED" OR __CONFIGURATION__ STREQUAL "GENERAL")
            else (__CONFIGURATION__ STREQUAL "DEBUG" OR __CONFIGURATION__ STREQUAL "OPTIMIZED" OR __CONFIGURATION__ STREQUAL "GENERAL")
                message (FATAL_ERROR "Invalid value for __CONFIGURATION__ passed in linker libraries property!")
            endif (__CONFIGURATION__ STREQUAL "DEBUG" OR __CONFIGURATION__ STREQUAL "OPTIMIZED" OR __CONFIGURATION__ STREQUAL "GENERAL")

            # Configuration keyword must be lowercase (i.e., "debug", "optimized" or "general")
            string (TOLOWER ${__CONFIGURATION__} __CONFIGURATION__)

            set (__IS_INFO__ FALSE)
        else (__IS_INFO__)
            if (${__BYVALUE__} STREQUAL "VALUE")
                # ${__LNK__} is the name of the linker library
                xme_message (VERBOSE "Linked target '${TARGET}' in configuration '${__CONFIGURATION__}' against third-party library '${__LNK__}'")
                target_link_libraries (${TARGET} ${__CONFIGURATION__} ${__LNK__})
            elseif (${__BYVALUE__} STREQUAL "VALUE_OF")
                # ${__LNK__} is the name of the variable holding a linker library name
                if (NOT ${__LNK__})
                    message (WARNING "Lazily evaluated variable '${__LNK__}', which is required for component '${__COMP_LOWER__}', is not defined!")
                else ()
                    xme_message (VERBOSE "Linked target '${TARGET}' in configuration '${__CONFIGURATION__}' against third-party library '${${__LNK__}}'")
                    target_link_libraries (${TARGET} ${__CONFIGURATION__} ${${__LNK__}})
                endif ()
            elseif (${__BYVALUE__} STREQUAL "TEMPLATE")
                # ${__LNK__} is a template possibly containing the names of variables holding a linker library name
                _xme_resolve_template_variables(${COMP} ${__LNK__} __RESULT__)
                if (__RESULT__)
                    xme_message (VERBOSE "Linked target '${TARGET}' in configuration '${__CONFIGURATION__}' against third-party library '${__RESULT__}'")
                    target_link_libraries (${TARGET} ${__CONFIGURATION__} ${__RESULT__})
                endif ()
            else ()
                xme_message(WARNING "Invalid __BYVALUE__ value '${__BYVALUE__}'!")
            endif ()
            set (__IS_INFO__ TRUE)
        endif (__IS_INFO__)
    endforeach (__LNK__)
endfunction (_xme_link_thirdparty_dependencies)

# Macro:      xme_build_documentation ( <project> TARGET <target> [ OUTPUT_DIR <output-dir> ] [ OUTPUT_NAME <output-name> ] [ INSTALL_DIR <install-dir> ] [ MODE <mode> ] [AUTO] [CLEAN] [RESTRICT] [ COMPONENTS [ WITH_DEPENDENCIES ] <component1> [ <component2> ... ] ] [ FILES <file1> [ <file2> ... ] ] )
#
# Purpose:    Schedule Doxygen documentation to be built for an amount of source files.
#
# Parameters: <project>                  Human-readable name of the documentation project.
#
#             TARGET <target>            Name of the build target for documentation creation.
#                                        A custom target with that name will be created.
#
#             OUTPUT_DIR <output-dir>    Output directory for generated documentation. In case
#                                        HTML Help Workshop is installed under Windows, the
#                                        resulting *.chm file will be placed in this directory.
#                                        Omit or set to an empty value to place the files in a
#                                        subdirectory of the current build directory.
#
#             OUTPUT_NAME <output-name>  Name (including extension) of the resulting *.chm file
#                                        if HTML Help Workshop is used. Omit or leave empty to
#                                        use the default "index.chm".
#
#             INSTALL_DIR <install-dir>  Installation directory for generated documentation.
#                                        Omit or leave empty to disable installation.
#
#             MODE <mode>                Documentation mode. Supported values for <mode> are:
#
#                                         - DEVELOPER (default if MODE is omitted): Generated
#                                             documentation is as a developer reference.
#                                             All input files are processed.
#
#                                         - PUBLIC: Generated documentation is designed for
#                                             developers that use CHROMOSOME functionality,
#                                             but do not need an extensive documentation of
#                                             the internals. Only header files are considered.
#
#             AUTO                       If specified, the given target will be scheduled to be
#                                        built automatically when the project is built. Otherwise,
#                                        the documentation target build must be invoked manually
#                                        on demand.
#
#             CLEAN                      If specified, the output directory will be completely
#                                        removed before each Doxygen run.
#
#             RESTRICT                   If this parameter is specified, all other files that would
#                                        otherwise automatically be added to the documentation are
#                                        excluded from the documentation. Most meaningful in
#                                        combination with COMPONENTS argument to select exactly a
#                                        given set of components for generating documentation.
#
#             COMPONENTS [ WITH_DEPENDENCIES ] <component1> [ ... ]
#                                        List of components for documentation should be generated.
#                                        All source and header files active on the current platform
#                                        will automatically added to the list of Doxygen input files.
#                                        If the WITH_DEPENDENCIES keyword directly follows the
#                                        COMPONENTS keyword, then all source and header files of all
#                                        direct and indirect dependencies of the respective components
#                                        will also become part of the documentation.
#                                        You may repeat the COMPONENTS statement multiple times,
#                                        with and without WITH_DEPENDENCIES keyword, in order to
#                                        specify that dependencies should only be considered for some
#                                        components.
#
#             FILES <file1> [ ... ]      List of additional source files to use as input for
#                                        documentation. All source and header files of used
#                                        or linked XME components will automatically be added
#                                        and do not need to be specified explicitly.
# 
macro (xme_build_documentation PROJECT_NAME TARGET_KEYWORD TARGET_NAME) # optional: OUTPUT_DIR_KEYWORD OUTPUT_DIR OUTPUT_NAME_KEYWORD OUTPUT_NAME INSTALL_DIR_KEYWORD INSTALL_DIR FILES_KEYWORD AUTO CLEAN FILES ...

	xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
	set (__SYNTAX_OK__ TRUE)

	if ("" STREQUAL PROJECT_NAME)
		xme_message (ERROR "Error: Empty project name argument!")
		set (__SYNTAX_OK__ FALSE)
	elseif (NOT "xTARGET" STREQUAL "x${TARGET_KEYWORD}") # TARGET is a special keyword to if(), hence needs to be escaped
		xme_message (ERROR "Error: Missing TARGET keyword!")
		set (__SYNTAX_OK__ FALSE)
	elseif (TARGET_NAME STREQUAL "")
		xme_message (ERROR "Error: Empty target name argument!")
		set (__SYNTAX_OK__ FALSE)
	endif ("" STREQUAL PROJECT_NAME)

	# Parse remaining options
	set (__OUTPUT_DIR__ "") # OUTPUT_DIR <output-dir>
	set (__OUTPUT_NAME__ "") # OUTPUT_NAME <output-name>
	set (__INSTALL_DIR__ "") # INSTALL_DIR <install-dir>
	set (__OUTPUT_MODE__ "DEVELOPER") # MODE <mode>
	set (__AUTO__ FALSE) # AUTO
	set (__CLEAN__ FALSE) # CLEAN
	set (__RESTRICT__ FALSE) # RESTRICT
    set (__COMPONENTS_NODEPS__) # COMPONENTS ...
	set (__COMPONENTS_WITHDEPS__) # COMPONENTS WITH_DEPENDENCIES ...
	set (__FILES__) # FILES ...

	set (__MODE__ 0)
	foreach (ARG ${ARGN})
		if ("OUTPUT_DIR" STREQUAL ARG)
			set (__MODE__ 1)
		elseif ("OUTPUT_NAME" STREQUAL ARG)
			set (__MODE__ 2)
		elseif ("INSTALL_DIR" STREQUAL ARG)
			set (__MODE__ 3)
		elseif ("MODE" STREQUAL ARG)
			set (__MODE__ 4)
		elseif ("AUTO" STREQUAL ARG)
			set (__AUTO__ TRUE)
			set (__MODE__ 0)
		elseif ("CLEAN" STREQUAL ARG)
			set (__CLEAN__ TRUE)
			set (__MODE__ 0)
		elseif ("RESTRICT" STREQUAL ARG)
			set (__RESTRICT__ TRUE)
			set (__MODE__ 0)
		elseif ("COMPONENTS" STREQUAL ARG)
			set (__MODE__ 5)
		elseif ("WITH_DEPENDENCIES" STREQUAL ARG)
			if (__MODE__ EQUAL 5)
				set (__MODE__ 6)
			else ()
				xme_message (ERROR "Error: Keyword \"WITH_DEPENDENCIES\" does not follow \"COMPONENTS\" keyword!")
			endif ()
		elseif ("FILES" STREQUAL ARG)
			set (__MODE__ 7)
		else ()
			if (__MODE__ EQUAL 0)
				xme_message (ERROR "Error: Unexpected arguments (probably missing keyword)!")
				set (__SYNTAX_OK__ FALSE)
				break()
			elseif (__MODE__ EQUAL 1)
				set (__OUTPUT_DIR__ ${ARG})
			elseif (__MODE__ EQUAL 2)
				set (__OUTPUT_NAME__ ${ARG})
			elseif (__MODE__ EQUAL 3)
				set (__INSTALL_DIR__ ${ARG})
			elseif (__MODE__ EQUAL 4)
				set (__OUTPUT_MODE__ ${ARG})
				if (NOT __OUTPUT_MODE__ STREQUAL "DEVELOPER" AND NOT __OUTPUT_MODE__ STREQUAL "PUBLIC")
					xme_message (ERROR "Error: Invalid MODE parameter \"${__OUTPUT_MODE__}\"!")
					set (__SYNTAX_OK__ FALSE)
				endif ()
			elseif (__MODE__ EQUAL 5)
				list (APPEND __COMPONENTS_NODEPS__ ${ARG})
			elseif (__MODE__ EQUAL 6)
				list (APPEND __COMPONENTS_WITHDEPS__ ${ARG})
			elseif (__MODE__ EQUAL 7)
				foreach (__FILE__ ${ARG})
					list (APPEND __FILES__ "${_XME_CURRENT_SOURCE_DIR}/${ARG}")
				endforeach (__FILE__)
			endif ()
		endif ()
	endforeach (ARG)

    # Check for errors
    if (NOT __SYNTAX_OK__)
        xme_message (FATAL_ERROR "Usage: xme_build_documentation ( <project> TARGET <target> [ OUTPUT_DIR <output-dir> ] [ OUTPUT_NAME <output-name> ] [ INSTALL_DIR <install-dir> ] [ MODE <mode> ] [AUTO] [CLEAN] [RESTRICT] [ COMPONENTS <component1> [ <component2> ... ] ] [ FILES <file1> [ <file2> ... ] ] )")
    endif (NOT __SYNTAX_OK__)

    if (NOT __RESTRICT__)
        xme_get (__DOCUMENTATION_FILES__ PROPERTY_GLOBAL XME_DOCUMENTATION_FILES)
        list (APPEND __FILES__ ${__DOCUMENTATION_FILES__})
    endif ()

    # Append files of mentioned components (with and without dependencies)
    foreach (__COMPONENT__ ${__COMPONENTS_NODEPS__} ${__COMPONENTS_WITHDEPS__})
        string (TOUPPER ${__COMPONENT__} __COMP_UPPER__)

        xme_get (__HEADERS__ PROPERTY_GLOBAL "${__COMP_UPPER__}_HEADERS")
        xme_append (VARIABLE __FILES__ "${__HEADERS__}" UNIQUE)

        xme_get (__SOURCES__ PROPERTY_GLOBAL "${__COMP_UPPER__}_SOURCES")
        xme_append (VARIABLE __FILES__ "${__SOURCES__}" UNIQUE)
    endforeach (__COMPONENT__)

    # Append files of directly or indirectly dependent components
    set (__DEPENDENCY_LIST__ )
    foreach (__COMPONENT__ ${__COMPONENTS_WITHDEPS__})
        string (TOUPPER ${__COMPONENT__} __COMP_UPPER__)

        xme_get (__DEPENDS__ PROPERTY_GLOBAL "${__COMP_UPPER__}_DEPENDS")
        xme_append (VARIABLE __DEPENDENCY_LIST__ "${__DEPENDS__}" UNIQUE)
    endforeach (__COMPONENT__)
    set (__INDEX__ 0)
    list (LENGTH __DEPENDENCY_LIST__ __LENGTH__)
    while (__INDEX__ LESS __LENGTH__)
        list (GET __DEPENDENCY_LIST__ ${__INDEX__} __COMPONENT__)
        string (TOUPPER ${__COMPONENT__} __COMP_UPPER__)

        xme_get (__HEADERS__ PROPERTY_GLOBAL "${__COMP_UPPER__}_HEADERS")
        xme_append (VARIABLE __FILES__ "${__HEADERS__}" UNIQUE)

        xme_get (__SOURCES__ PROPERTY_GLOBAL "${__COMP_UPPER__}_SOURCES")
        xme_append (VARIABLE __FILES__ "${__SOURCES__}" UNIQUE)

        xme_get (__DEPENDS__ PROPERTY_GLOBAL "${__COMP_UPPER__}_DEPENDS")
        xme_append (VARIABLE __DEPENDENCY_LIST__ "${__DEPENDS__}" UNIQUE)
        list (LENGTH __DEPENDENCY_LIST__ __LENGTH__)

        math (EXPR __INDEX__ "${__INDEX__} + 1")
    endwhile ()

	list (LENGTH __FILES__ __LENGTH__)

	if (__LENGTH__ GREATER 0)
		set (__INTERNAL_DOCS_ARG__ "")

		# Filter files according to processing mode
		if (__OUTPUT_MODE__ STREQUAL "DEVELOPER")
			set (__FILES_FILTERED__ ${__FILES__})
			set (__INTERNAL_DOCS_ARG__ "INTERNAL_DOCS")
		else ()
			if (__OUTPUT_MODE__ STREQUAL "PUBLIC")
				foreach (__FILE__ ${__FILES__})
					xme_is_header_file (${__FILE__} __FLAG__)
					if (__FLAG__)
						list (APPEND __FILES_FILTERED__ ${__FILE__})
					endif ()
				endforeach (__FILE__)
			else ()
				xme_message (FATAL_ERROR "Unsupported mode \"${__OUTPUT_MODE__}\" for documentation \"${PROJECT_NAME}\"!")
			endif ()
			xme_message (NOTE "Generating documentation \"${PROJECT_NAME}\" in mode \"${__OUTPUT_MODE__}\"")
		endif ()

		set (__AUTO_ARG__ "")
		if (__AUTO__)
			set (__AUTO_ARG__ "AUTO")
		endif (__AUTO__)

		set (__CLEAN_ARG__ "")
		if (__CLEAN__)
			set (__CLEAN_ARG__ "CLEAN")
		endif (__CLEAN__)

		xme_simplify_path (XME_ROOT_SIMPLE "${XME_ROOT}")
		doxygen_generate_documentation(
			"${PROJECT_NAME}"
			TARGET "${TARGET_NAME}"
			BASE_DIR "${XME_ROOT_SIMPLE}"
			OUTPUT_DIR "${__OUTPUT_DIR__}"
			OUTPUT_NAME "${__OUTPUT_NAME__}"
			INSTALL_DIR "${__INSTALL_DIR__}"
			${__INTERNAL_DOCS_ARG__}
			${__AUTO_ARG__}
			${__CLEAN_ARG__}
			FILES ${__FILES_FILTERED__}
		)

		xme_message (VERBOSE "Generated target '${TARGET_NAME}' for building '${PROJECT_NAME}' with ${__LENGTH__} input file(s)")
	endif (__LENGTH__ GREATER 0)

	# Generate dependency graph if Graphviz is available
	if (GRAPHVIZ_FOUND)
		add_custom_command(
			TARGET "${TARGET_NAME}"
			POST_BUILD
			COMMAND "${CMAKE_COMMAND}" "--graphviz=${PROJECT_NAME}.dot" ${CMAKE_SOURCE_DIR}
			COMMAND "${GRAPHVIZ_BINARY}" -Tpdf "${PROJECT_NAME}.dot" > "${PROJECT_NAME}.pdf"
			COMMAND "${GRAPHVIZ_BINARY}" -Tpng "${PROJECT_NAME}.dot" > "${PROJECT_NAME}.png"
			WORKING_DIRECTORY .
			COMMENT "Building dependency graph..."
		)
	endif (GRAPHVIZ_FOUND)

endmacro (xme_build_documentation)

# Macro:      xme_generic_port_path ( <output-varname> <port-name> )
#
# Purpose:    Returns a relative path from the current directory (XME_CURRENT_SOURCE_DIR) to
#             the directory of the XME port for architecture/platform port-name.
#             This is useful to reference generic implementations (e.g., in C language)
#             from architecture specific ports (e.g., ARMv7-M).
#             As a side effect, the computed directory will be added as an include
#             directory since it might contain header files.
#
# Parameters: <output-varname>           Name of the variable to receive the relative path name.
#
#             <port-name>                Name of the architecture/platform to look up.
# 
macro (xme_generic_port_path OUTPUT_PATH PORT)
	if (${ARGC} GREATER 3)
		xme_message (WARNING "Too many arguments to macro xme_generic_port_path()!")
	endif ()

	if (${ARGC} EQUAL 3)
		set(_NUM_DIRS_TO_REMOVE ${ARGV2})
		if (${_NUM_DIRS_TO_REMOVE} LESS 0)
			set(_NUM_DIRS_TO_REMOVE 1)
		endif (${_NUM_DIRS_TO_REMOVE} LESS 0)
	else (${ARGC} EQUAL 3)
		set(_NUM_DIRS_TO_REMOVE 1)
	endif (${ARGC} EQUAL 3)

	# Calculate current relative path (with respect to port root),
	# i.e. the suffix after ${XME_CURRENT_SOURCE_DIR}/ports/<os|plat>.
	xme_get (_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)

	# Remove ${XME_SRC_DIR}/ports/hardware/ or ${XME_SRC_DIR}/ports/software/ (whichever is used)
	string (REGEX REPLACE "^${XME_SRC_DIR}/ports/(hardware|software)/" "" _DIR ${_DIR})

	# Remove arch (or plat)
	string (FIND ${_DIR} "/" _POS)
	math (EXPR _POS "${_POS} + 1")
	string (SUBSTRING ${_DIR} ${_POS} -1 _DIR)

	# Remove respective architecture / platform directories
	# and compute respective part of the relative path which
	# is contributed by these directories.
	set (_I 0)
	set (_PREFIX_CNT 0)
	while (NOT (${_I} EQUAL ${_NUM_DIRS_TO_REMOVE}))
		string (FIND ${_DIR} "/" _POS)
		math (EXPR _POS "${_POS} + 1")
		string (SUBSTRING ${_DIR} ${_POS} -1 _DIR)
		math (EXPR _I "${_I} + 1")
		math (EXPR _PREFIX_CNT "${_PREFIX_CNT} + 1")
	endwhile (NOT (${_I} EQUAL ${_NUM_DIRS_TO_REMOVE}))
	
	set (_REL_PATH ${_DIR})
	# Finalize relative path from current directory by walking
	# up to ${XME_CURRENT_SOURCE_DIR}/ports/<arch|plat>.
	while (NOT ${_POS} EQUAL -1)
		string (FIND ${_DIR} "/" _POS)
		math (EXPR _PREFIX_CNT "${_PREFIX_CNT} + 1")
		string (SUBSTRING ${_DIR} 0 ${_POS} _TMP)
		if (NOT ${_POS} EQUAL -1)
			math (EXPR _POS "${_POS} + 1")
			string (SUBSTRING ${_DIR} ${_POS} -1 _DIR)
		endif (NOT ${_POS} EQUAL -1)	
	endwhile (NOT ${_POS} EQUAL -1)

	xme_get (_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)
	while (${_PREFIX_CNT} GREATER 0)
		string (FIND ${_DIR} "/" _POS REVERSE)
		string (SUBSTRING "${_DIR}" 0 ${_POS} _DIR)
		math (EXPR _PREFIX_CNT "${_PREFIX_CNT} -1")
	endwhile (${_PREFIX_CNT} GREATER 0)

	# Add generic port directory to list of include directories
	xme_include_directory("${_DIR}/${PORT}")

	# Finalize path by appending desired port and subdirectory within
	# current port
	set (${OUTPUT_PATH} "${_DIR}/${PORT}/${_REL_PATH}")
endmacro (xme_generic_port_path)

# Macro:      xme_callback_register ( <callback-name> <callback-function> )
#
# Purpose:    This function can be used to register a callback which is later executed
#             by calling the macro xme_callback_execute( ... ). A list of all registered
#             callbacks is kept which is executed in the order of registration.
#
# Callbacks:
#             Name:        XME_ADD_EXECUTABLE_POSTHOOK 
#             Parameters:  TARGET - Name of target 
#             Description: This callback is executed at the end of the xme_add_executable
#                          macro. It can be used to configure something that depends on the
#                          name of the target e.g. settings for an IDE.
#
#             Name:        XME_USE_COMPONENT_POSTHOOK
#             Parameters:  COMPONENT - Name of component (equals name of target)
#             Description: This callback is executed after an XME component has been
#                          scheduled for build. This happens if it is a direct or indirect
#                          dependency of an executable target. The parameter specifies the
#                          name of the component scheduled for build, which is the same than
#                          the name of the respective target.
#
# Parameters: <callback-name>            The name of the callback list. This name has to be
#                                        given as parameter to xme_callback_execute to identify
#                                        all callbacks for execution.
#
#             <callback-function>        Name of the macro which is called when the callback
#                                        is executed.
# 
macro (xme_callback_register CALLBACK_NAME CALLBACK_FUNCTION) 
	if (NOT ${ARGC} EQUAL 2)
		xme_message (WARNING "Wrong number of arguments to macro xme_callback_register()!")
	endif (NOT ${ARGC} EQUAL 2)

	if (NOT COMMAND ${CALLBACK_FUNCTION})
		xme_message (FATAL_ERROR "Callback function not defined in xme_callback_register()!")
	endif (NOT COMMAND ${CALLBACK_FUNCTION})

	xme_append (PROPERTY_GLOBAL "CALLBACK_LIST_${CALLBACK_NAME}" "${CALLBACK_FUNCTION}")
endmacro (xme_callback_register)

# Macro:      xme_callback_execute ( <callback-name> <Parameter1> ... <ParameterN> )
#
# Purpose:    This function executes all callbacks previously registered with 
#             xme_callback_register. All callbacks associated with <callback-name> 
#             are executed in the order of registration. A list of callbacks is
#             available in the description of the macro xme_callback_register. All
#             parameters are passed to the callback macros.
#
# Parameters: <callback-name>            The name of the callback list. This name has to be
#                                        equal to the name used in xme_callback_register.
#
#             <Parameter>                A variable list of parameters which is passed to
#                                        the callback macros.
#
macro (xme_callback_execute CALLBACK_NAME)
	if (${ARGC} LESS 1)
		xme_message (WARNING "Too few arguments to macro xme_callback_execute()!")
	endif (${ARGC} LESS 1)

	xme_get (_LIST PROPERTY_GLOBAL "CALLBACK_LIST_${CALLBACK_NAME}")

	# Builds string for parameters
	set (_PARAMETER_LIST)
	foreach (_PARAMETER_ITEM ${ARGN})
		set (_PARAMETER_LIST "${_PARAMETER_LIST} ${_PARAMETER_ITEM}")
	endforeach (_PARAMETER_ITEM ${ARGN})

	set (_FILENAME "${CMAKE_BINARY_DIR}/callback.cmake")

	# Execute callbacks
	foreach (_CALLBACK_ITEM ${_LIST})
		set (_CALLBACK_STRING "${_CALLBACK_ITEM} (${_PARAMETER_LIST})")
		file (WRITE "${_FILENAME}" ${_CALLBACK_STRING})
		include (${_FILENAME})
	endforeach (_CALLBACK_ITEM)

    # Notice (Issue #2938): We do not delete the generated file, because
    # otherwise CMake will re-run due to a missing build dependency when
    # we call 'make' subsequently. Since CMake processes the file immediately
    # upon include() and the file if overwritten every time before it is
    # loaded, this is not a problem.
    #execute_process (COMMAND ${CMAKE_COMMAND} -E remove "${_FILENAME}")
endmacro (xme_callback_execute)



# Macro:      xme_execution_test ( <target> EXECUTABLE <name>
#                                  [ STDIN <stdin-file> | EXPECT_SCRIPT <expect-script> ]
#                                  [ EXPECT_STDOUT <stout-file> ]
#                                  [ EXPECT_STDERR <stderr-file> ]
#                                  [ EXPECT_EXIT_CODE <exit-code> ]
#                                  [ TIMEOUT <timeout> ] )
#
# Purpose:    This macro executes the given executable and checks whether the
#             output is as expected.
#
# Parameters: <target>       Name of the custom target that represents this execution test.
#
#             <name>         Name of the executable to run. Must be a target name.
#                            A dependency against that target is automatically added.
#
#             <stdin-file>   If specified, names a file whose content is to be piped to the
#                            process. Cannot be combined with EXPECT.
#
#             <expect-script>
#                            If specified, names an expect script file (see
#                            http://en.wikipedia.org/wiki/Expect) that is to be used to
#                            interact with the executable. Cannot be combined with STDIN.
#
#             <stout-file>   If specified, names a file with the expected standard output of
#                            the process.
#
#             <stderr-file>  If specified, names a file with the expected error output of
#                            the process.
#
#             <exit-code>    If specified, indicates the expected exit code of the process.
#                            Non-zero exit codes are considered as errors by default except
#                            if this argument specifies that the non-zero exit code is
#                            expected.
#
#             <timeout>      If specified, the process will be terminated after the given
#                            number of seconds (may be a fractal number) if it is still
#                            running.
#
macro (xme_execution_test TARGET_NAME EXECUTABLE_KEYWORD EXECUTABLE_NAME)
	if (${ARGC} LESS 3)
		xme_message (WARNING "Too few arguments to macro xme_execution_test()!")
	endif (${ARGC} LESS 3)
	
	if (NOT ${EXECUTABLE_KEYWORD} STREQUAL "EXECUTABLE")
		xme_message (FATAL_ERROR "Invalid parameters passed to xme_execution_test()!")
	endif ()

	set (__ARGS__ ${ARGV})
	list (GET __ARGS__ 0 __COMP_NAME__)
	list (REMOVE_AT __ARGS__ 0 1 2)
	

	set (__MODE__ 0)
	set (__WARN_IF_NEW_MODE__ FALSE)
	set (__STDIN__)
	set (__EXPECT__)
	set (__STDOUT__)
	set (__STDERR__)
	set (__EXIT_CODE__)
	set (__TIMEOUT__)
	foreach (ARG ${__ARGS__})
		if (ARG STREQUAL "STDIN")
			set (__MODE__ 1)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "EXPECT_SCRIPT")
			set (__MODE__ 2)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
		elseif (ARG STREQUAL "EXPECT_STDOUT")
			set (__MODE__ 3)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "EXPECT_STDERR")
			set (__MODE__ 4)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "EXPECT_EXIT_CODE")
			set (__MODE__ 5)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		elseif (ARG STREQUAL "TIMEOUT")
			set (__MODE__ 6)
			if (__WARN_IF_NEW_MODE__)
				xme_message (WARNING "Found section with no input statements before '${ARG}' in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif (__WARN_IF_NEW_MODE__)
			set (__WARN_IF_NEW_MODE__ TRUE)
		else ()
			set (__WARN_IF_NEW_MODE__ FALSE)

			if (__MODE__ EQUAL 0)
				xme_message (FATAL_ERROR "Found argument '${ARG}' with no preceeding keyword in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! Maybe you are missing a parameter or is it set to an empty value?")
			endif ()
			
			# All items besides mode 5 and 6 are file arguments
			if (__MODE__ EQUAL 5 OR __MODE__ EQUAL 6)
				set (__ITEM__ ${ARG})
			elseif (IS_ABSOLUTE ${ARG})
				set (__ITEM__ ${ARG})
			else (IS_ABSOLUTE ${ARG})
				set (__ITEM__ ${_XME_CURRENT_SOURCE_DIR}/${ARG})
			endif ()

			if (__MODE__ EQUAL 1)
				if (__STDIN__)
					xme_message(WARNING "Duplicate STDIN parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__STDIN__ "${__ITEM__}")
				set (__MODE__ 0)
			elseif (__MODE__ EQUAL 2)
				if (__EXPECT__)
					xme_message(WARNING "Duplicate EXPECT_SCRIPT parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__EXPECT__ "${__ITEM__}")
				set (__MODE__ 0)
			elseif (__MODE__ EQUAL 3)
				if (__STDOUT__)
					xme_message(WARNING "Duplicate EXPECT_STDOUT parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__STDOUT__ "${__ITEM__}")
				set (__MODE__ 0)
			elseif (__MODE__ EQUAL 4)
				if (__STDERR__)
					xme_message(WARNING "Duplicate EXPECT_STDERR parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__STDERR__ "${__ITEM__}")
				set (__MODE__ 0)
			elseif (__MODE__ EQUAL 5)
				if (__EXIT_CODE__)
					xme_message(WARNING "Duplicate EXPECT_EXIT_CODE parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__EXIT_CODE__ "${__ITEM__}")
				set (__MODE__ 0)
			elseif (__MODE__ EQUAL 6)
				if (__TIMEOUT__)
					xme_message(WARNING "Duplicate TIMEOUT parameter in definition of execution test '${TARGET_NAME}' in file '${CMAKE_CURRENT_LIST_FILE}'! The last value set will be used.")
				endif ()
				set (__TIMEOUT__ "${__ITEM__}")
				set (__MODE__ 0)
			else (__MODE__ EQUAL 0)
				xme_message (FATAL_ERROR "Invalid parameters passed to xme_execution_test()!")
			endif ()
		endif ()
	endforeach (ARG)

	if (__STDIN__ AND __EXPECT__)
		xme_message (FATAL_ERROR "Invalid arguments passed to macro xme_execution_test(): pass either STDIN or EXPECT_SCRIPT, but not both!")
	endif ()

	set (EXECUTE_PARAMS)

	if (NOT __EXIT_CODE__)
		set (__EXIT_CODE__ 0) # Expect zero exit code by default
	endif ()
	if (__TIMEOUT__)
		list (APPEND EXECUTE_PARAMS "TIMEOUT" "${__TIMEOUT__}")
	endif ()
	if (__STDIN__)
		list (APPEND EXECUTE_PARAMS "INPUT_FILE" "${__STDIN__}")
	elseif (__EXPECT__)
		find_package (Expect REQUIRED)

		# Generate expect script
		set (__EXPECT_SCRIPT__ "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}.expect")
		file (WRITE  ${__EXPECT_SCRIPT__} "set program [lrange \$argv 0 end]\n")
		file (APPEND ${__EXPECT_SCRIPT__} "\n")
		file (APPEND ${__EXPECT_SCRIPT__} "catch \"spawn -noecho \$program\"\n")
		file (APPEND ${__EXPECT_SCRIPT__} "source \"${__EXPECT__}\"\n")
	endif ()

	set (REAL_STDOUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}-stdout.txt")
	set (REAL_STDERR_FILE "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}-stderr.txt")
	
	set (CONVERTED_REAL_STDOUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}-stdout-lf.txt")
	set (CONVERTED_REAL_STDERR_FILE "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}-stderr-lf.txt")

	list (APPEND EXECUTE_PARAMS "OUTPUT_FILE" "${REAL_STDOUT_FILE}")
	list (APPEND EXECUTE_PARAMS "ERROR_FILE" "${REAL_STDERR_FILE}")

	set (__SCRIPT__ "${CMAKE_CURRENT_BINARY_DIR}/execution-test-${TARGET_NAME}.cmake")

	file (WRITE  ${__SCRIPT__} "# Automatically generated file, do not edit!\n")
	file (APPEND ${__SCRIPT__} "set (PATHS \"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}\")\n")
	file (APPEND ${__SCRIPT__} "foreach (CONFIG ${CMAKE_CONFIGURATION_TYPES})\n")
	file (APPEND ${__SCRIPT__} "    list (APPEND PATHS \"${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/\${CONFIG}\")\n")
	file (APPEND ${__SCRIPT__} "endforeach (CONFIG)\n")
	file (APPEND ${__SCRIPT__} "find_program (CLIENT NAMES \"${EXECUTABLE_NAME}\" PATHS \${PATHS})\n")
	file (APPEND ${__SCRIPT__} "if (NOT CLIENT)\n")
	file (APPEND ${__SCRIPT__} "    message (FATAL_ERROR \"Unable to find executable \\\"${EXECUTABLE_NAME}\\\" for execution test \\\"${TARGET_NAME}\\\" (looked in the following paths: \${PATHS})!\")\n")
	file (APPEND ${__SCRIPT__} "endif()\n")
	file (APPEND ${__SCRIPT__} "message (STATUS \"Running executable \\\"\${CLIENT}\\\" (this might take a while) ...\")\n")
	file (APPEND ${__SCRIPT__} "set (ENV{nodosfilewarning} 1) # Prevent Cygwin from complaining about MS-DOS-style paths\n")
	if (__EXPECT__)
		file (APPEND ${__SCRIPT__} "execute_process (COMMAND \"${EXPECT_BINARY}\" \"${__EXPECT_SCRIPT__}\" \"\${CLIENT}\" RESULT_VARIABLE RESULT ${EXECUTE_PARAMS})\n")
	else ()
		file (APPEND ${__SCRIPT__} "execute_process (COMMAND \"\${CLIENT}\" RESULT_VARIABLE RESULT ${EXECUTE_PARAMS})\n")
	endif ()
	file (APPEND ${__SCRIPT__} "if (NOT \${RESULT} EQUAL ${__EXIT_CODE__})\n")
	file (APPEND ${__SCRIPT__} "    message (SEND_ERROR \"Executable returned with \${RESULT} exit status, but expected ${__EXIT_CODE__}!\")\n")
	file (APPEND ${__SCRIPT__} "endif ()\n")
	file (APPEND ${__SCRIPT__} "message (STATUS \"Executable returned with \${RESULT} exit status\")\n")
	file (APPEND ${__SCRIPT__} "find_program (DIFF NAMES \"diff\" PATHS \"/\" \"/usr\" \"/usr/local\" \"C:/cygwin\" PATH_SUFFIXES \"bin\" \"sbin\" DOC \"diff program\")\n")
	file (APPEND ${__SCRIPT__} "find_program (SED NAMES \"sed\" PATHS \"/\" \"/usr\" \"/usr/local\" \"C:/cygwin\" PATH_SUFFIXES \"bin\" \"sbin\" DOC \"sed program\")\n")
	if (__STDOUT__)
		file (APPEND ${__SCRIPT__} "if (SED)\n")
		file (APPEND ${__SCRIPT__} "    execute_process (COMMAND \"\${SED}\" -r \"s/\\\\x1B\\\\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g\" \"${REAL_STDOUT_FILE}\" OUTPUT_VARIABLE OUTPUT) # Get rid of escape sequences (e.g., color codes)\n")
		file (APPEND ${__SCRIPT__} "    file (WRITE \"${REAL_STDOUT_FILE}\" \"\${OUTPUT}\")\n")
		file (APPEND ${__SCRIPT__} "endif ()\n")
		file (APPEND ${__SCRIPT__} "configure_file (\"${REAL_STDOUT_FILE}\" \"${CONVERTED_REAL_STDOUT_FILE}\" @ONLY NEWLINE_STYLE LF)\n")
		file (APPEND ${__SCRIPT__} "execute_process (COMMAND \"${CMAKE_COMMAND}\" -E compare_files \"${CONVERTED_REAL_STDOUT_FILE}\" \"${__STDOUT__}\" ERROR_VARIABLE OUTPUT)\n")
		file (APPEND ${__SCRIPT__} "if (OUTPUT)\n")
		file (APPEND ${__SCRIPT__} "    message (STATUS \"Real standard output differs from expected standard output!\")\n")
		file (APPEND ${__SCRIPT__} "    if (DIFF)\n")
		file (APPEND ${__SCRIPT__} "        message (STATUS \"Here is a list of differences:\")\n")
		file (APPEND ${__SCRIPT__} "        execute_process (COMMAND \"\${DIFF}\" -u \"${__STDOUT__}\" \"${CONVERTED_REAL_STDOUT_FILE}\")\n")
		file (APPEND ${__SCRIPT__} "    endif ()\n")
		file (APPEND ${__SCRIPT__} "    message (SEND_ERROR \"Error: real standard output differs from expected standard output!\")\n")
		file (APPEND ${__SCRIPT__} "else ()\n")
		file (APPEND ${__SCRIPT__} "    message (STATUS \"Standard output verified.\")\n")
		file (APPEND ${__SCRIPT__} "endif ()\n")
	endif()
	if (__STDERR__)
		file (APPEND ${__SCRIPT__} "if (SED)\n")
		file (APPEND ${__SCRIPT__} "    execute_process (COMMAND \"\${SED}\" -r \"s/\\\\x1B\\\\[([0-9]{1,2}(;[0-9]{1,2})?)?[m|K]//g\" \"${REAL_STDERR_FILE}\" OUTPUT_VARIABLE OUTPUT) # Get rid of escape sequences (e.g., color codes)\n")
		file (APPEND ${__SCRIPT__} "    file (WRITE \"${REAL_STDERR_FILE}\" \"\${OUTPUT}\")\n")
		file (APPEND ${__SCRIPT__} "endif ()\n")
		file (APPEND ${__SCRIPT__} "configure_file (\"${REAL_STDERR_FILE}\" \"${CONVERTED_REAL_STDERR_FILE}\" @ONLY NEWLINE_STYLE LF)\n")
		file (APPEND ${__SCRIPT__} "execute_process (COMMAND \"${CMAKE_COMMAND}\" -E compare_files \"${CONVERTED_REAL_STDERR_FILE}\" \"${__STDERR__}\" ERROR_VARIABLE OUTPUT)\n")
		file (APPEND ${__SCRIPT__} "if (OUTPUT)\n")
		file (APPEND ${__SCRIPT__} "    message (STATUS \"Real error output differs from expected error output!\")\n")
		file (APPEND ${__SCRIPT__} "    if (DIFF)\n")
		file (APPEND ${__SCRIPT__} "        message (STATUS \"Here is a list of differences:\")\n")
		file (APPEND ${__SCRIPT__} "        execute_process (COMMAND \"\${DIFF}\" -u \"${__STDOUT__}\" \"${CONVERTED_REAL_STDERR_FILE}\")\n")
		file (APPEND ${__SCRIPT__} "    endif ()\n")
		file (APPEND ${__SCRIPT__} "    message (SEND_ERROR \"Error: real error output differs from expected error output!\")\n")
		file (APPEND ${__SCRIPT__} "else ()\n")
		file (APPEND ${__SCRIPT__} "    message (STATUS \"Error output verified.\")\n")
		file (APPEND ${__SCRIPT__} "endif ()\n")
	endif()

	# Add a custom target to automatically validate the result from this example project
	add_custom_target(
		${TARGET_NAME}
		"${CMAKE_COMMAND}" -P "${__SCRIPT__}"
		DEPENDS "${EXECUTABLE_NAME}"
		COMMENT "Running execution test \"${TARGET_NAME}\"..."
		VERBATIM
	)
endmacro (xme_execution_test)



# Function    xme_compilation_tests ( <group-name> [ TEST <test-name1> CODE <test-code1> EXPECT <expect-pass1> ] [ TEST <testname-2> ... ] )
#
# Purpose:    Generates an array of targets for compilation testing snippets of C code if compilation tests are enabled.
#             The target names will be of the form "compilationTest_<group-name>_<test-nameN>_expect<Pass|Fail>".
#             Whether compilation tests are enabled or not is determined depending on whether the CMake cache entry
#             XME_ENABLE_COMPILATIONTESTS is set. See the documentation of that define in FindXME.cmake for more information.
#
# Parameters: <group-name>        Name of this group of compilation tests. Used to build the target names.
#
#             TEST <test-nameN>   Name of this compilation test.
#
#             CODE <test-codeN>   C code to compile in the test. If this code contains semicola,
#                                 make sure they are escaped properly or CMake will treat them as
#                                 argument separators.
#
#             EXPECT <expect-passN>
#                                 Set to a value considered true by CMake (e.g., TRUE, ON, 1) if
#                                 the code snippet should pass compilation or a value considered
#                                 false by CMake (e.g., FALSE, OFF, 0) if it should fail.
#                                 The value "PASS" can be used as a synonym for "TRUE" and the
#                                 value "FAIL" can be used as a synonym for "FALSE".
#
function (xme_compilation_tests GROUP_NAME)
    # Check whether compilation tests are actually enabled
    if (NOT XME_ENABLE_COMPILATIONTESTS)
        return ()
    endif ()

    # Just remember the compilation tests in some global properties.
    # They will be converted into targets later in the configuration process.
    xme_append(PROPERTY_GLOBAL "XME_COMPILATION_TEST_GROUPS" ${GROUP_NAME} UNIQUE)
    xme_append(PROPERTY_GLOBAL "XME_COMPILATION_TEST_ARGS_${GROUP_NAME}" "${ARGN}")
endfunction (xme_compilation_tests)

function (_xme_create_compilation_tests)
    xme_get (CTGROUPS PROPERTY_GLOBAL "XME_COMPILATION_TEST_GROUPS")
    foreach (CTGROUP ${CTGROUPS})
        xme_message (VERBOSE "Generating compilation tests for group \"${CTGROUP}\":")
        _xme_create_compilation_test (${CTGROUP})
    endforeach (CTGROUP)
endfunction (_xme_create_compilation_tests)

function (_xme_create_compilation_test GROUP_NAME)
    set (XME_COMPILATIONTEST_TESTCASE_DEFINES)
    set (XME_COMPILATIONTEST_TESTCASE_NAMES)
    set (XME_COMPILATIONTEST_TESTCASE_IMPLEMENTATIONS)

    set (COMPILATIONTEST_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/CompilationTests.${GROUP_NAME}.dir/compilationTests.c")

    xme_get (CTARGS PROPERTY_GLOBAL "XME_COMPILATION_TEST_ARGS_${GROUP_NAME}")
    list (LENGTH CTARGS CTARGC)

    set (TEST_COUNT 0)
    set (ARG_COUNT 0)
    while (${ARG_COUNT} LESS ${CTARGC})
        set (TEST_NAME "")

        list (GET CTARGS ${ARG_COUNT} KEYWORD_TEST)
        if (NOT "${KEYWORD_TEST}" STREQUAL "TEST")
            xme_message (FATAL_ERROR "Expected keyword 'TEST', but got token '${KEYWORD_TEST}'!")
        endif ()

        math (EXPR ARG_COUNT "${ARG_COUNT}+1")
        list (GET CTARGS ${ARG_COUNT} TEST_NAME)

        math (EXPR ARG_COUNT "${ARG_COUNT}+1")
        list (GET CTARGS ${ARG_COUNT} KEYWORD_CODE)
        if (NOT "${KEYWORD_CODE}" STREQUAL "CODE")
            xme_message (FATAL_ERROR "Expected keyword 'CODE', but got token '${KEYWORD_CODE}'!")
        endif ()

        math (EXPR ARG_COUNT "${ARG_COUNT}+1")
        if (NOT ${ARG_COUNT} LESS ${CTARGC})
            xme_message (FATAL_ERROR "Expected more tokens after 'CODE', but got end of argument list!")
        endif ()
        set (TEST_CODE )
        list (GET CTARGS ${ARG_COUNT} TOKEN)
        while (NOT ${TOKEN} STREQUAL "EXPECT" AND ${ARG_COUNT} LESS ${CTARGC})
            list (GET CTARGS ${ARG_COUNT} TOKEN)
            if (NOT "EXPECT" STREQUAL "${TOKEN}")
                list (APPEND TEST_CODE ${TOKEN})
                math (EXPR ARG_COUNT "${ARG_COUNT}+1")
            endif ()
        endwhile ()
        if (NOT ${ARG_COUNT} LESS ${CTARGC})
            xme_message (FATAL_ERROR "Expected keyword 'EXPECT', but got end of argument list!")
        endif ()

        list (GET CTARGS ${ARG_COUNT} KEYWORD_EXPECT)
        if (NOT "${KEYWORD_EXPECT}" STREQUAL "EXPECT")
            xme_message (FATAL_ERROR "Expected keyword 'EXPECT', but got token '${KEYWORD_EXPECT}'!")
        endif ()

        math (EXPR ARG_COUNT "${ARG_COUNT}+1")
        list (GET CTARGS ${ARG_COUNT} TEST_EXPECT)

        if ("${TEST_EXPECT}" STREQUAL "PASS")
            set (TEST_EXPECT_PASS TRUE)
        elseif ("${TEST_EXPECT}" STREQUAL "FAIL")
            set (TEST_EXPECT_PASS FALSE)
        elseif(TEST_EXPECT)
            set (TEST_EXPECT_PASS TRUE)
        else ()
            set (TEST_EXPECT_PASS FALSE)
        endif ()
        
        math (EXPR ARG_COUNT "${ARG_COUNT}+1")

        # Ensure TEST_NAME is a valid C identifier
        string (REGEX REPLACE "[^A-Za-z0-9]" "_" TEST_NAME ${TEST_NAME})

        math (EXPR TEST_COUNT "${TEST_COUNT}+1")

        set(
            XME_COMPILATIONTEST_TESTCASE_DEFINES
            "${XME_COMPILATIONTEST_TESTCASE_DEFINES}#define TEST_${TEST_NAME} ${TEST_COUNT}\n"
        )

        set(
            XME_COMPILATIONTEST_TESTCASE_NAMES
            "${XME_COMPILATIONTEST_TESTCASE_NAMES}    \"${TEST_NAME}\",\n"
        )

        set(
            XME_COMPILATIONTEST_TESTCASE_IMPLEMENTATIONS
            "${XME_COMPILATIONTEST_TESTCASE_IMPLEMENTATIONS}#elif XME_COMPILATIONTEST_RUN == TEST_${TEST_NAME}\n    ${TEST_CODE};\n"
        )

        if (TEST_EXPECT_PASS)
            set (TARGET_SUFFIX "_expectPass")
            set (TEST_EXPECT "PASS")
        else ()
            set (TARGET_SUFFIX "_expectFail")
            set (TEST_EXPECT "FAIL")
        endif ()

        set (TARGET_NAME "compilationTest_${GROUP_NAME}_${TEST_NAME}${TARGET_SUFFIX}")

        xme_message (VERBOSE "Generating compilation test \"${TARGET_NAME}\"")
        xme_add_executable(
            ${TARGET_NAME}
            ${COMPILATIONTEST_SOURCE} GENERATED
            EXCLUDE_FROM_ALL
        )

        set_target_properties(
            ${TARGET_NAME}
            PROPERTIES
                COMPILE_DEFINITIONS
                "XME_COMPILATIONTEST_RUN=TEST_${TEST_NAME};XME_COMPILATIONTEST_EXPECT=${TEST_EXPECT}"
        )
    endwhile ()

    # Configure the test file
    configure_file(
        "${XME_COMPILATIONTEST_TEMPLATE}"
        "${COMPILATIONTEST_SOURCE}"
        @ONLY
    )
endfunction (_xme_create_compilation_test)

find_file(
    XME_COMPILATIONTEST_TEMPLATE
    NAMES "compilationTests.c.in"
    PATHS ${CMAKE_CURRENT_LIST_DIR}
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)



# Function:   xme_code_example ( <component> EXAMPLE <example> [ FILES ] <file1> [ <file2> ... ] [ BUILD [ <buildfile1> [ <buildfile2> ] <depends1> [ <depends2> ... ] ] )
#
# Purpose:    Define a code example that can be referred to from within the
#             Doxygen documentation. For this purpose, the path of the
#             CMakeLists.txt file where this function is used is automatically
#             added to the EXAMPLE_PATH Doxygen setting.
#             Optionally, the code example can be scheduled for building in
#             order to test it. This is meaningful if the code example does
#             not only contain a code snippet, but a full program. The code
#             example is scheduled for building when both the BUILD argument
#             to this function is present as well as the global CMake variable
#             XME_ENABLE_CODE_EXAMPLE_BUILDS is set to a true value.
#             If a build is scheduled, the target name is built from the name
#             of the component and the example, separated by an underscore
#             character and the suffix "CodeExample" appended (for easy
#             filtering in the generated Makefile), for example:
#             xme_adv_myComponent_feature1CodeExample
#
# Parameters: <component>         Name of the component this code example
#                                 belongs to.
#
#             EXAMPLE <example>   Name of this example, for example the name of
#                                 the function whose usage is illustrated in
#                                 the code example.
#
#             [ FILES ] <file1> [ <file2> ... ]
#                                 Specifies one or more files to consider as
#                                 code examples. Those files will be
#                                 automatically added to the project in case
#                                 the example code is scheduled for build.
#
#             [ BUILD [ <buildfile1> [ <buildfile2> ] <depends1> [ <depends2> ... ] ]
#                                 When the "BUILD" keyword is specified, a
#                                 target is created that can be used to build
#                                 the code example. The arguments following the
#                                 "BUILD" keyword, if any, specify additional
#                                 source or header files (depending on their
#                                 file extension) or dependencies against other
#                                 XME components.
#                                 An executable target with the name as
#                                 specified above will be added to the project,
#                                 provided the XME_ENABLE_CODE_EXAMPLE_BUILDS
#                                 CMake variable is set to a true value.
#
function (xme_code_example COMPONENT EXAMPLE_KEYWORD EXAMPLE)
    if (${ARGC} LESS 4)
        xme_message (WARNING "Too few arguments to function xme_code_example()!")
    endif (${ARGC} LESS 4)
    
    if (NOT ${EXAMPLE_KEYWORD} STREQUAL "EXAMPLE")
        xme_message (FATAL_ERROR "Invalid parameters passed to xme_code_example()!")
    endif ()

    set (TARGET_NAME "${COMPONENT}_${EXAMPLE}CodeExample")

    # Just remember the example projects in some global properties.
    # They will be converted into targets later in the configuration process.
    xme_append (PROPERTY_GLOBAL "XME_CODE_EXAMPLE_TARGETS" ${TARGET_NAME} UNIQUE)
    xme_set (PROPERTY_GLOBAL "XME_CODE_EXAMPLE_ARGS_${TARGET_NAME}" "PATH;${CMAKE_CURRENT_LIST_DIR};COMPONENT;${COMPONENT};EXAMPLE;${EXAMPLE};${ARGN};BUILD_SYSTEM;${CMAKE_CURRENT_LIST_FILE}")
endfunction (xme_code_example)

function (_xme_create_code_example_builds)
    xme_get (CETARGETS PROPERTY_GLOBAL "XME_CODE_EXAMPLE_TARGETS")
    foreach (CETARGET ${CETARGETS})
        xme_message (VERBOSE "Generating code example target \"${CETARGET}\":")
        _xme_create_code_example_build (${CETARGET})
    endforeach (CETARGET)
endfunction (_xme_create_code_example_builds)

function (_xme_create_code_example_build TARGET_NAME)
    xme_get (CEARGS PROPERTY_GLOBAL "XME_CODE_EXAMPLE_ARGS_${TARGET_NAME}")
    list (LENGTH CEARGS CEARGC)

    set (ARG_COUNT 0)
    set (PATH_NAME "")
    set (COMPONENT_NAME "")
    set (EXAMPLE_NAME "")
    set (FILE_LIST)
    set (BUILD_FILE_LIST)
    set (BUILD_DEPENDS)
    set (BUILD NO)
    set (MODE 0)
    while (${ARG_COUNT} LESS ${CEARGC})
        list (GET CEARGS ${ARG_COUNT} TOKEN)

        if ("${TOKEN}" STREQUAL "PATH")
            xme_assert(MODE EQUAL 0)
            set (MODE 1)
        elseif ("${TOKEN}" STREQUAL "COMPONENT")
            xme_assert(MODE EQUAL 2)
            set (MODE 3)
        elseif ("${TOKEN}" STREQUAL "EXAMPLE")
            xme_assert(MODE EQUAL 4)
            set (MODE 5)
        elseif ("${TOKEN}" STREQUAL "FILES")
            xme_assert(MODE EQUAL 6)
            set (MODE 7)
        elseif ("${TOKEN}" STREQUAL "BUILD")
            xme_assert(MODE EQUAL 7)
            set (BUILD YES)
            set (MODE 8)
        elseif ("${TOKEN}" STREQUAL "BUILD_SYSTEM")
            set (MODE 9)
        elseif (MODE EQUAL 1)
            # PATH argument
            set (PATH_NAME ${TOKEN})
            set (MODE 2)
        elseif (MODE EQUAL 3)
            # COMPONENT argument
            set (COMPONENT_NAME ${TOKEN})
            set (MODE 4)
        elseif (MODE EQUAL 5)
            # EXAMPLE argument
            set (EXAMPLE_NAME ${TOKEN})
            set (MODE 6)
        elseif (MODE EQUAL 7)
            # FILES argument

            set (FILE ${TOKEN})
            if (NOT IS_ABSOLUTE ${FILE})
                set (FILE "${PATH_NAME}/${FILE}")
            endif ()

            if (NOT EXISTS ${FILE})
                xme_message (WARNING "Unable to find file '${FILE}' for code example '${EXAMPLE_NAME}' of component '${COMPONENT_NAME}'!")
            endif ()

            xme_append (VARIABLE FILE_LIST ${FILE} UNIQUE)
        elseif (MODE EQUAL 8)
            # BUILD argument(s)

            # Auto-detect item type. Everything that is neither a source
            # nor a header file is treated as a component dependency
            xme_is_header_file(${TOKEN} __FLAG_HEADER__)
            xme_is_source_file(${TOKEN} __FLAG_SOURCE__)
            xme_is_asm_file(${TOKEN} __FLAG_ASM__)

            if (__FLAG_HEADER__ OR __FLAG_SOURCE__ OR __FLAG_ASM__)
                set (FILE ${TOKEN})
                if (NOT IS_ABSOLUTE ${FILE})
                    set (FILE "${PATH_NAME}/${FILE}")
                endif ()
            
                if (NOT EXISTS ${FILE})
                    xme_message (WARNING "Unable to find file '${FILE}' for code example '${EXAMPLE_NAME}' of component '${COMPONENT_NAME}'!")
                endif ()

                # Add the file to the list of required files in any case (even if it is missing).
                # If the file is missing, a respective error message will be output by CMake
                # when the build system is generated.
                xme_append (VARIABLE BUILD_FILE_LIST ${FILE} UNIQUE)
            else ()
                # Dependency
                list (APPEND BUILD_DEPENDS ${TOKEN})
            endif ()
        elseif (MODE EQUAL 9)
            # BUILD_SYSTEM argument(s)

            set (FILE ${TOKEN})
            if (NOT IS_ABSOLUTE ${FILE})
                set (FILE "${PATH_NAME}/${FILE}")
            endif ()

            xme_append (VARIABLE BUILD_SYSTEM_FILE_LIST ${FILE} UNIQUE)
        else ()
            xme_message (FATAL_ERROR "Invalid arguments to xme_code_example() or internal error!")
        endif ()
        
        math (EXPR ARG_COUNT "${ARG_COUNT}+1")
    endwhile ()

    #message("XME_ENABLE_CODE_EXAMPLE_BUILDS '${XME_ENABLE_CODE_EXAMPLE_BUILDS}'")
    #message("COMPONENT_NAME '${COMPONENT_NAME}')
    #message("EXAMPLE_NAME '${EXAMPLE_NAME}'")
    #message("FILE_LIST '${FILE_LIST}'")
    #message("BUILD '${BUILD}'")
    #message("BUILD_FILE_LIST '${BUILD_FILE_LIST}'")
    #message("BUILD_SYSTEM_FILE_LIST '${BUILD_SYSTEM_FILE_LIST}'")
    #message("BUILD_DEPENDS '${BUILD_DEPENDS}'")

    if (COMPONENT_NAME STREQUAL "")
        xme_message (FATAL_ERROR "Missing component name in call to xme_code_example()!")
    endif ()

    if (EXAMPLE_NAME STREQUAL "")
        xme_message (FATAL_ERROR "Missing example name in call to xme_code_example()!")
    endif ()

    if (NOT FILE_LIST)
        xme_message (FATAL_ERROR "Missing input file(s) in call to xme_code_example()!")
    endif ()

    # Consider all files for the EXAMPLE_PATH Doxygen setting
    foreach (FILE ${FILE_LIST})
        xme_append (PROPERTY_GLOBAL "XME_CODE_EXAMPLE_PATHS" ${FILE} UNIQUE)
    endforeach ()

    # Schedule the code example for build if it is requested
    if (BUILD AND XME_ENABLE_CODE_EXAMPLE_BUILDS)
        xme_add_executable(
            ${TARGET_NAME}
            ${FILE_LIST}
            ${BUILD_FILE_LIST}
            ${BUILD_SYSTEM_FILE_LIST}
        )

        # Add "Build System" source group
        source_group ("Build System" FILES ${BUILD_SYSTEM_FILE_LIST})

        if (BUILD_DEPENDS)
            xme_link_components(
                ${TARGET_NAME}
                ${BUILD_DEPENDS}
            )
        endif ()
    endif ()
endfunction (_xme_create_code_example_build)



# Function    xme_buildsystem_stats ( )
#
# Purpose:    Prints statistiocal information about the number of components in certain categories,
#             both used and unused by the current project.
#
# Parameters: (none)
#
function (xme_buildsystem_stats)
    set (NAMESPACES "xme_adv_;xme_prim_;xme_core_;xme_hal_;xme_")
    xme_get (COMPONENTS PROPERTY_GLOBAL XME_COMPONENTS)
    list (SORT COMPONENTS)

    xme_message (NOTE "--- Build system statistics:")
    xme_message (NOTE "> Component statistics (numeric):")
    xme_message (NOTE "  Namespace   Used   Unused   Total")
    foreach (NS ${NAMESPACES})
        set (COUNT 0)
        set (USED 0)
        foreach (COMP ${COMPONENTS})
            if (${COMP} MATCHES "^${NS}.*")
                math (EXPR COUNT "${COUNT} + 1")
                string (TOUPPER ${COMP} "COMP_UPPER")
                xme_get (IS_USED PROPERTY_GLOBAL "${COMP_UPPER}_USED")
                if (IS_USED)
                    math (EXPR USED "${USED} + 1")
                endif ()
            endif ()
        endforeach (COMP)

        math (EXPR UNUSED "${COUNT} - ${USED}")

        set (STR "  ${NS}* ")
        xme_pad_string (STR 14)
        set (STR "${STR}${USED} ")
        xme_pad_string (STR 21)
        set (STR "${STR}${UNUSED} ")
        xme_pad_string (STR 30)
        set (STR "${STR}${COUNT}")
        xme_message (NOTE "${STR}")
    endforeach (NS)

    xme_message (NOTE "> Component statistics (by name):")
    xme_message (NOTE "  Component                                      Used   Headers   Sources")
    foreach (COMP ${COMPONENTS})
        string (TOUPPER ${COMP} "COMP_UPPER")

        xme_get (HEADERS PROPERTY_GLOBAL "${COMP_UPPER}_HEADERS")
        xme_get (SOURCES PROPERTY_GLOBAL "${COMP_UPPER}_SOURCES")
        xme_get (IS_USED PROPERTY_GLOBAL "${COMP_UPPER}_USED")

        list (LENGTH "HEADERS" NUM_HEADERS)
        list (LENGTH "SOURCES" NUM_SOURCES)

        set (STR "  ${COMP} ")
        xme_pad_string (STR 48 ".")
        if (IS_USED)
            set (STR "${STR} yes ")
        else ()
            set (STR "${STR}  no ")
        endif ()
        xme_pad_string (STR 56)
        set (STR "${STR}${NUM_HEADERS}")
        xme_pad_string (STR 66)
        set (STR "${STR}${NUM_SOURCES}")
        xme_message (NOTE "${STR}")
    endforeach (COMP)

    xme_message (NOTE "---")
endfunction (xme_buildsystem_stats)
