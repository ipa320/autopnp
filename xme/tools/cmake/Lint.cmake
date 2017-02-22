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
# $Id: Lint.cmake 5565 2013-10-21 16:29:52Z geisinger $
#

# This file contains functions and configurations for generating Lint build
# targets for your CMake projects.

# --------------------------------------------------------------------------------
# To use in Jenkins with "splint" you have to do following:
#
# To configure the custom parser, try this in the Jenkins configuration under "Compiler Warnings":
# 
# Name:
# Splint Warnings
# 
# Regular Expression:
# ^splint:\d+\,\d+\,(.*)\,\d+\,(.*)\,(\d+)\,\d+\,\"(.*)\"\,\"(.*)\"$
# 
# Mapping Script:
# import hudson.plugins.warnings.parser.Warning
# import hudson.plugins.analysis.util.model.Priority
# 
# String category = 'Splint Warnings'
# String fileName = matcher.group(2)
# String lineNumber = matcher.group(3)
# String type = matcher.group(1)
# String message = matcher.group(5)
# 
# return new Warning(fileName, Integer.parseInt(lineNumber), type, category, message, Priority.LOW);
# --------------------------------------------------------------------------------

# Function:   xme_add_lint ( <component-name> <source1.c> [ <source2.c> ... ] )
#
# Purpose:    Creates a target representing a run of Lint (static analysis) against the given
#             source files. Notice that by default, all sources of XME components will
#             automatically be considered for static analysis if Lint is found.
#
# Parameters: <component-name>
#                     Name of the component for which the testsuite is defined.
#
#             <source1.c> [ <source2.c> ... ]
#                     List of source files to pass to lint for static analysis.
#
function (xme_add_lint COMPONENT_NAME)
    set (__USAGE__ "Usage: xme_add_lint ( <component-name> <source1.c> [ <source2.c> ... ] )")

    if (${ARGC} LESS 1)
        xme_message (FATAL_ERROR "No component name provided to xme_add_lint()! ${__USAGE__}")
    endif (${ARGC} LESS 1)
    
    list (LENGTH ARGN __argc__)
    if (${__argc__} LESS 1)
        xme_message (FATAL_ERROR "No source files provided to xme_add_lint() for component \"${COMPONENT_NAME}\"! ${__USAGE__}")
    endif (${__argc__} LESS 1)

    #message("LINT ${COMPONENT_NAME}: ${ARGN}")

    if (LINT_FOUND)
        if (NOT LINT_OPTION_PREFIX_DEFINE)
            xme_message (FATAL_ERROR "LINT_OPTION_PREFIX_DEFINE not set!")
        endif ()
        if (NOT LINT_OPTION_PREFIX_INCLUDEPATH)
            xme_message (FATAL_ERROR "LINT_OPTION_PREFIX_INCLUDEPATH not set!")
        endif ()

        set (_lint_defines "")
        set (_lint_include_path "")

        get_directory_property (__include_paths INCLUDE_DIRECTORIES)
        get_directory_property (__compile_defines COMPILE_DEFINITIONS)
        xme_get (_XME_CURRENT_SOURCE_DIR PROPERTY_GLOBAL XME_CURRENT_SOURCE_DIR)

        # Prepend each include directory, also quotes the directory
        foreach (_include_path ${__include_paths})
            list (APPEND _lint_include_path "${LINT_OPTION_PREFIX_INCLUDEPATH}${_include_path}")
        endforeach (_include_path ${__include_paths})

        # When defining toolchain definitions, the use of add_definitions()
        # is inappropriate, because it will only account for the toolchain
        # directory and not any other non-subdirectory. This is why toolchain
        # definitions typically append compile definitions directly by using
        # -D or /D to CMAKE_{C|CXX}_FLAGS*. Hence, we have to parse these
        # variables here in order to ensure that Lint uses exactly the same
        # preprocessor definitions than the compiler.
        set (__parsing "${CMAKE_C_FLAGS} ${CMAKE_CXX_FLAGS}")
        if (NOT CMAKE_BUILD_TYPE STREQUAL "")
            string (TOUPPER ${CMAKE_BUILD_TYPE} __build_type_upper)
            set (__parsing "${__parsing} ${CMAKE_C_FLAGS_${__build_type_upper}} ${CMAKE_CXX_FLAGS_${__build_type_upper}}")
        endif ()
        string(REGEX MATCHALL "(^| )(-|/)D([^ ]+)( |$)" _tokens_temp "${__parsing}")
        set (_definitions)
        foreach (_token ${_tokens_temp})
            string (STRIP ${_token} _token)
            string (REGEX REPLACE "^(-|/)D([^ ]+)$" "\\2" _definition ${_token})
            xme_append (VARIABLE _definitions ${_definition} UNIQUE)
        endforeach (_token)

        # Prepend each definition
        foreach (_definition ${__compile_defines})
            list (APPEND _lint_defines "${LINT_OPTION_PREFIX_DEFINE}${_definition}")
        endforeach (_definition)
        foreach (_definition ${_definitions})
            list (APPEND _lint_defines "${LINT_OPTION_PREFIX_DEFINE}${_definition}")
        endforeach (_definition)

        list (REMOVE_DUPLICATES _lint_defines)
        list (REMOVE_DUPLICATES _lint_include_path)

        set (_lint_source_files)
        foreach (_sourcefile ${ARGN})
            # Really only include source files
            xme_is_source_file (${_sourcefile} _is_sourcefile)
            if (_is_sourcefile)
                xme_resolve_path (_sourcefile_abs ${_sourcefile})
                xme_append (VARIABLE _lint_source_files ${_sourcefile_abs} UNIQUE)
            endif()
        endforeach (_sourcefile)

        set (__lint_command ${LINT_OPTIONS} ${_lint_defines} ${_lint_include_path} ${_lint_source_files})

        # Add a custom target consisting of all the commands generated above
        add_custom_target (${COMPONENT_NAME}_lint ${LINT_EXECUTABLE} ${__lint_command} VERBATIM)

        # Make the lint target depend on each and every *_lint target
        add_dependencies(lint ${COMPONENT_NAME}_lint)
    endif (LINT_FOUND)
endfunction(xme_add_lint)

if (NOT CMAKE_HOST_WIN32)
    if (NOT LINT_FOUND)
        find_program (LINT_EXECUTABLE
                      NAMES lint flint flexelint pclint
                      PATHS "$ENV{ProgramFiles}/pclint/bin"
                            "C:/Program Files/pclint/bin"
                            "$ENV{PCPLINT_HOME}/bin"
                            "/usr/bin"
                            "/usr/local/bin"
                            "/opt/flexelint/"
                            "/opt/flexelint/bin"
                            "/opt/flexelint9/"
                            "/opt/flexelint9/bin"
                      DOC "Path to the Lint executable. Lint is a tool for statically checking C programs for security vulnerabilities and programming mistakes."
        )

        if (LINT_EXECUTABLE)
            get_filename_component (LINT_EXE_PATH "${LINT_EXECUTABLE}" PATH)
            get_filename_component (LINT_ROOT ${LINT_EXE_PATH} PATH)
            xme_message (VERBOSE "XME: LINT_EXE_PATH = ${LINT_EXE_PATH}")
            xme_message (VERBOSE "XME: LINT_ROOT = ${LINT_ROOT}")
            
            set (LINT_OPTION_PREFIX_DEFINE "-d" CACHE STRING "" FORCE)
            set (LINT_OPTION_PREFIX_INCLUDEPATH "-i" CACHE STRING "" FORCE)

            include (FindPackageHandleStandardArgs)
            FIND_PACKAGE_HANDLE_STANDARD_ARGS (LINT DEFAULT_MSG LINT_EXECUTABLE)
            
            find_path(LINT_FILE_PATH co-gcc.lnt 
                      "${LINT_ROOT}"
                      "${LINT_ROOT}/supp"
                      "${LINT_ROOT}/supp/gcc"
                      "${LINT_ROOT}/supp/lnt"
                      "${LINT_EXE_PATH}"
                      "${LINT_EXE_PATH}/supp"
                      "${LINT_EXE_PATH}/supp/gcc"
                      "${LINT_EXE_PATH}/supp/lnt"
                      "/usr/local/flint/supp/gcc/")
            
            find_path(LINT_XME_FILE_PATH xme.lnt 
                      "${CMAKE_CURRENT_LIST_DIR}")
            
            set (LINT_OPTIONS "${LINT_OPTION_PREFIX_INCLUDEPATH}${LINT_FILE_PATH}" "co-gcc.lnt"
                              "${LINT_OPTION_PREFIX_INCLUDEPATH}${LINT_XME_FILE_PATH}" "xme.lnt"
                              "+ffn"                             # Force full path names
                              "-width(0,0)"                      # Don't insert line breaks (unlimited output width).
                              "-hf1"                             # Message height one
                              "-format=%f(%l): %t %n: %m"        # Format qualifier needed for jenkins plugin
                              #"-w1"                             # Set Warning level
                              #"-wlib(1)"                        # Only report errors from the libraries -> improve later with 2 or 3                    
                              #"-zero(99)"                       # Don't stop make because of warnings
                              "-zero"                            # Don't stop make because of warnings
                              "-passes(2)"                       # Make sure to make two passes (for better error messages)
                              "-restore_at_end"                  # Don't let -e<nnnn> options bleed to other source files
                              #"-summary()"                      # Produce a summary of all produced messages
            )

        else (LINT_EXECUTABLE)

            find_program (LINT_EXECUTABLE
                          NAMES lint splint
                          PATHS "$ENV{ProgramFiles}/splint/bin"
                                "C:/Program Files/splint/bin"
                                "$ENV{ProgramFiles}/splint-3.1.1/bin"
                                "C:/Program Files/splint-3.1.1/bin"
                                "$ENV{SPLINT_HOME}/bin"
                                "/usr/bin"
                                "/usr/local/bin"
                                "/opt/splint/bin"
                          DOC "Lint is a tool for statically checking C programs for security vulnerabilities and programming mistakes."
            )

            if (LINT_EXECUTABLE)

                get_filename_component (LINT_EXE_PATH "${LINT_EXECUTABLE}" PATH)
                get_filename_component (LINT_ROOT ${LINT_EXE_PATH} PATH)
                xme_message (VERBOSE "XME: LINT_EXE_PATH = ${LINT_EXE_PATH}")
                xme_message (VERBOSE "XME: LINT_ROOT = ${LINT_ROOT}")
                
                set (LINT_OPTION_PREFIX_DEFINE "-D" CACHE STRING "" FORCE)
                set (LINT_OPTION_PREFIX_INCLUDEPATH "-I" CACHE STRING "" FORCE)

                find_file (LINT_LCL 
                           NAMES lclinit.lci
                           PATHS "${LINT_ROOT}/supp"
                                 "$ENV{ProgramFiles}/splint/lib"
                                 "C:/Program Files/splint/lib"
                                 "$ENV{ProgramFiles}/splint-3.1.1/lib"
                                 "C:/Program Files/splint-3.1.1/lib"
                                 "$ENV{SPLINT_HOME}/lib"
                                 "$ENV{LARCH_PATH}"
                                 "/usr/lib"
                                 "/usr/local/lib"
                                 "/usr/share/splint/lib/"
                                 "/opt/splint/lib"
                                 "/opt/flexelint/"
                                 "/opt/flexelint9/"
                                 "/opt/flexelint9/bin"
                           DOC "Searching for standard Larch C Interface Language (LCL) init files"
                )

                get_filename_component (LINT_LCL_PATH "${LINT_LCL}" PATH)
    
                find_file (LINT_STANDARD_LCL
                           NAMES stdlib.lcl
                           PATHS "$ENV{ProgramFiles}/splint/imports"
                                 "C:/Program Files/splint/imports"
                                 "$ENV{ProgramFiles}/splint-3.1.1/imports"
                                 "C:/Program Files/splint-3.1.1/imports"
                                 "$ENV{SPLINT_HOME}/imports"
                                 "$ENV{LCLIMPORTDIR}"
                                 "/usr/imports"
                                 "/usr/local/imports"
                                 "/usr/share/splint/imports/"
                                 "/opt/splint/imports"
                           DOC "Searching for Standard Larch C Interface Language (LCL) import files"
                )
                
                get_filename_component (LINT_STANDARD_PATH "${LINT_STANDARD_LCL}" PATH)
        
                set (LINT_OPTIONS ##"+weak"
                                  "+enumindex"
                                  "-mayaliasunique" # FIXME: XME deprecation yields some valid warnings here, see Issue #3549
                                  "-temptrans"
                                  ##"-fixedformalarray"
                                  ##"-formattype"
                                  ##"-fullinitblock"
                                  ##"-incompletetype"
                                  ##"-initallelements"
                                  ##"-likelybool"
                                  ##"-redef"
                                  ##"-sizeofformalarray"
                                  ##"-unrecog"
                                  ##"-boolint"
                                  ##"-bufferoverflowhigh"
                                  ##"-charindex"
                                  ##"-charint"
                                  ##"-charintliteral"
                                  ##"-charunsignedchar"
                                  ##"-checkstrictglobs"
                                  ##"-enumint"
                                  ##"-fcnuse"
                                  ##"-floatdouble"
                                  ##"-type"
                                  ##"-globuse"
                                  ##"-ifempty"
                                  ##"-longint"
                                  ##"-longunsignedintegral"
                                  ##"-macromatchname"
                                  ##"-numabstractindex"
                                  ##"-numabstractlit"
                                  ##"-numliteral"
                                  ##"-predboolothers"
                                  ##"-ptrnegate"
                                  ##"-relaxquals"
                                  ##"-retvalother"
                                  ##"-shortint"
                                  ##"-stackref"
                                  ##"-statemerge"
                                  ##"-statetransfer"
                                  ##"-stringliteralnoroom"
                                  ##"-stringliteralnoroomfinalnull"
                                  ##"-typeuse"
                                  ##"-varuse"
                                  ##"-voidabstract"
                                  ##"-zerobool"
                                  ##"+usedef"
                                  ##"+usereleased"
                                  ##"+mustfreeonly"
                                  ##"+mustfreefresh"
                                  ##"+varuse"
                                  ##"-retimponly"
                                  ##"-structimponly"
                                  ##"-nullret"
                                  ##"-warnposix"
                                  "+show-summary"
                                  #"+line-len 160"
                                  -linelen 999
                                  "-Dlint"
                                  "-D__builtin_va_list=va_list"
                                  #"+parenfileformat" #for DevStudio
                                  #-D__int64=long
                                  #"-D_int64=int"
                                  #"-D_wtoi=atoi"
                                  #"-DSTRICT"
                                  #"-D_M_IX86=400"
                                  #"-DWIN32"
                                  #"-D_NTSYSTEM"
                                  #"-D_WIN32_WINNT=0x0400"
                                  #"-DWINVER=600"
                                  #"-DNOGDI"
                                  #"-DNOKERNEL"
                                  #"-DNOUSER"
                                  #"-DWIN32_LEAN_AND_MEAN"
                                  "+indentspaces 8"
                                  #"+csv splint_out"
                                  #"+csvoverwrite"
                )

                set (ENV{LARCH_PATH} ${LINT_LCL_PATH})
                set (ENV{LCLIMPORTDIR} ${LINT_STANDARD_PATH})

                get_filename_component (LINT_EXE_PATH "${LINT_EXECUTABLE}" PATH)
                get_filename_component(LINT_ROOT ${LINT_EXE_PATH} PATH)    

                include(FindPackageHandleStandardArgs)
                FIND_PACKAGE_HANDLE_STANDARD_ARGS (LINT DEFAULT_MSG LINT_EXECUTABLE)

            endif (LINT_EXECUTABLE)

        endif (LINT_EXECUTABLE)

        # Maintain the _FOUND variables as "YES" or "NO" for backwards compatibility
        # (allows people to stuff them directly into Doxyfile with configure_file())
        if (LINT_FOUND)
            set (LINT_FOUND "YES")
            mark_as_advanced (LINT_EXECUTABLE)
        else ()
            set (LINT_FOUND "NO")
        endif ()
    endif (NOT LINT_FOUND)

    if (NOT TARGET lint)
        # Add a phony target which causes all available *_lint targets to be executed
        add_custom_target (lint)
    endif ()

    if (LINT_FOUND)
        macro (link_use_component_posthook COMPONENT)
            string (TOUPPER ${COMPONENT} __COMP_UPPER__)
            xme_get (__SOURCES__ PROPERTY_GLOBAL ${__COMP_UPPER__}_SOURCES)

            list (LENGTH __SOURCES__ __LENGTH__)
            if (__LENGTH__ GREATER 0)
                xme_message (VERBOSE "Scheduling ${__LENGTH__} source file(s) of \"${COMPONENT}\" for static analysis with Lint")
                foreach (__SOURCE__ ${__SOURCES__})
                    xme_message (DEBUG " |_ ${__SOURCE__}")
                endforeach (__SOURCE__)

                xme_add_lint (${COMPONENT} ${__SOURCES__})
            endif ()
        endmacro (link_use_component_posthook)

        # Whenever an XME component is used, make sure that all of its sources are processed by Lint
        xme_callback_register (XME_USE_COMPONENT_POSTHOOK "link_use_component_posthook")

        # TODO: Do the same for executables!
    endif (LINT_FOUND)

endif(NOT CMAKE_HOST_WIN32)
