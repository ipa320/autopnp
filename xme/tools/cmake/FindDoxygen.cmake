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
# $Id: FindDoxygen.cmake 6690 2014-02-07 15:58:08Z geisinger $
#

# Always prefer the version that is in the PATH
if (CMAKE_HOST_WIN32)
    find_path(
        DOXYGEN_BINARY_PATH
        NAMES "doxygen.exe"
        ENV PATH
        DOC "Doxygen binary path"
    )
else ()
    find_path(
        DOXYGEN_BINARY_PATH
        NAMES "doxygen" "doxygen.exe"
        ENV PATH
        DOC "Doxygen binary path"
    )
endif ()

# On Windows, prefer the 64 bit version if available (%PROGRAMW6432% will
# only be set on 64 bit systems) and fall back to 32 bit version if 64 bit
# version is not installed.
find_path(
    DOXYGEN_BINARY_PATH
    NAMES "doxygen" "doxygen.exe"
    PATHS "/usr/local" "/usr" "C:/cygwin" "$ENV{PROGRAMW6432}/doxygen" "$ENV{PROGRAMFILES}/doxygen"
    PATH_SUFFIXES "bin" "sbin"
    DOC "Doxygen binary path"
    NO_DEFAULT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

find_program(
    DOXYGEN_BINARY
    NAMES "doxygen" "doxygen.exe"
    HINTS ${DOXYGEN_BINARY_PATH}
    DOC "Doxygen binary"
    NO_DEFAULT_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_CMAKE_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
)

find_file(
    DOXYGEN_DOXYFILE_TEMPLATE
    NAMES "doxyfile.in"
    PATHS ${CMAKE_CURRENT_LIST_DIR}
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if (DOXYGEN_BINARY)
    set (DOXYGEN_FOUND YES)
    message (STATUS "Doxygen found in \"${DOXYGEN_BINARY_PATH}\"")
else ()
    set (DOXYGEN_FOUND NO)
    message (STATUS "Doxygen not found!")
endif ()

# Find GraphViz/dot
find_package (Graphviz REQUIRED)

# If on Windows platform, try to find HTML Help Workshop for compiling *.chm files
if (CMAKE_HOST_WIN32)
	find_package(HTMLHelpWorkshop)
endif (CMAKE_HOST_WIN32)



# Macro:      doxygen_generate_documentation ( <project> TARGET <target> BASE_DIR <base-dir> [ OUTPUT_DIR <output-dir> ] [ OUTPUT_NAME <output-name> ] [ INSTALL_DIR <install-dir> ] [ INTERNAL_DOCS ] [ AUTO ] [ CLEAN ] FILES <file1> [ <file2> ... ] )
#
# Purpose:    Schedule Doxygen documentation to be built for an amount of source files.
#
# Parameters: <project>                  Human-readable name of the Doxygen project.
#
#             TARGET <target>            Name of the build target for documentation creation.
#                                        A custom target with that name will be created.
#
#             BASE_DIR <base-dir>        Base directory of all documented files. This path will be
#                                        stripped from the beginning of all source file names.
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
#             INTERNAL_DOCS              If specified, Doxygen documentation also includes
#                                        items marked with \internal or @internal.
#
#             AUTO                       If specified, the given target will be schedules to be
#                                        built automatically when the project is built. Otherwise,
#                                        the documentation target build must be invoked manually
#                                        on demand.
#
#             CLEAN                      If specified, the output directory will be completely
#                                        removed before each Doxygen run.
#
#             FILES <file1> [ ... ]      List of source files to use as input for Doxygen.
#
macro (doxygen_generate_documentation PROJECT_NAME TARGET_KEYWORD TARGET_NAME BASE_DIR_KEYWORD BASE_DIR_NAME) # optional: OUTPUT_DIR_KEYWORD OUTPUT_DIR OUTPUT_NAME_KEYWORD OUTPUT_NAME INSTALL_DIR_KEYWORD INSTALL_DIR AUTO CLEAN FILES_KEYWORD FILES ...

	if (NOT DOXYGEN_FOUND)
		message (WARNING "Skipping generation of target '${TARGET_NAME}' because Doxygen was not found! If required, please install it manually to the default location!")
	else (NOT DOXYGEN_FOUND)

		set (__SYNTAX_OK__ TRUE)

		if ("" STREQUAL PROJECT_NAME)
			message (SEND_ERROR "Error: Empty project name argument!")
			set (__SYNTAX_OK__ FALSE)
		elseif (NOT "xTARGET" STREQUAL "x${TARGET_KEYWORD}") # TARGET is a special keyword to if(), hence needs to be escaped
			message (SEND_ERROR "Error: Missing TARGET keyword!")
			set (__SYNTAX_OK__ FALSE)
		elseif (TARGET_NAME STREQUAL "")
			message (SEND_ERROR "Error: Empty target name argument!")
			set (__SYNTAX_OK__ FALSE)
		elseif (NOT "xBASE_DIR" STREQUAL "x${BASE_DIR_KEYWORD}") # Somehow it doesn't work without the prefix...
			message (SEND_ERROR "Error: Missing BASE_DIR keyword!")
			set (__SYNTAX_OK__ FALSE)
		endif ("" STREQUAL PROJECT_NAME)

		# Parse remaining options
		set (DOXYGEN_OUTPUT_DIRECTORY "") # OUTPUT_DIR <output-dir>
		set (DOXYGEN_CHM_FILE "index.chm") # OUTPUT_NAME <output-name>
		set (__INSTALL_DIR__ "") # INSTALL_DIR <install-dir>
		set (__INTERNAL_DOCS__ FALSE) # INTERNAL_DOCS
		set (__AUTO__ FALSE) # AUTO
		set (__CLEAN__ FALSE) # CLEAN
		set (__FILES__) # FILES ...

		set (__MODE__ 0)
		foreach (ARG ${ARGN})
			if ("OUTPUT_DIR" STREQUAL ARG)
				set (__MODE__ 1)
			elseif ("OUTPUT_NAME" STREQUAL ARG)
				set (__MODE__ 2)
			elseif ("INSTALL_DIR" STREQUAL ARG)
				set (__MODE__ 3)
			elseif ("INTERNAL_DOCS" STREQUAL ARG)
				set (__INTERNAL_DOCS__ TRUE)
				set (__MODE__ 0)
			elseif ("AUTO" STREQUAL ARG)
				set (__AUTO__ TRUE)
				set (__MODE__ 0)
			elseif ("CLEAN" STREQUAL ARG)
				set (__CLEAN__ TRUE)
				set (__MODE__ 0)
			elseif ("FILES" STREQUAL ARG)
				set (__MODE__ 4)
			else ("OUTPUT_DIR" STREQUAL ARG)
				if (__MODE__ EQUAL 0)
					message (SEND_ERROR "Error: Unexpected arguments (probably missing keyword)!")
					set (__SYNTAX_OK__ FALSE)
					break()
				elseif (__MODE__ EQUAL 1)
					set (DOXYGEN_OUTPUT_DIRECTORY ${ARG})
				elseif (__MODE__ EQUAL 2)
					set (DOXYGEN_CHM_FILE ${ARG})
				elseif (__MODE__ EQUAL 3)
					set (__INSTALL_DIR__ ${ARG})
				elseif (__MODE__ EQUAL 4)
					list (APPEND __FILES__ ${ARG})
				endif (__MODE__ EQUAL 0)
			endif ("OUTPUT_DIR" STREQUAL ARG)
		endforeach (ARG)

		# Check for errors
		if (NOT __SYNTAX_OK__)
			message (FATAL_ERROR "Usage: doxygen_generate_documentation ( <project> TARGET <target> BASE_DIR <base-dir> [ OUTPUT_DIR <output-dir> ] [ OUTPUT_NAME <output-name> ] [ INSTALL_DIR <install-dir> ] [ INTERNAL_DOCS ] [ AUTO ] [ CLEAN ] FILES <file1> [ <file2> ... ] )")
		endif (NOT __SYNTAX_OK__)

		# Generate some filenames
		set (__OUTPUT__ "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${TARGET_NAME}.doxygen.dir")
		set (__SCRIPT__ "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${TARGET_NAME}.cmake")
		set (__CONFIG__ "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${TARGET_NAME}.doxyfile")

		#message("OUTPUT '${__OUTPUT__}'")
		#message("SCRIPT '${__SCRIPT__}'")
		#message("CONFIG '${__CONFIG__}'")

		# Apply default values
		if (DOXYGEN_OUTPUT_DIRECTORY STREQUAL "")
			set (DOXYGEN_OUTPUT_DIRECTORY "${__OUTPUT__}")
		endif (DOXYGEN_OUTPUT_DIRECTORY STREQUAL "")

		# Apply variable values for configuration file generation using @ONLY
        # - DOXYGEN_PROJECT_NAME
        # - DOXYGEN_SOURCE_ROOT
        # - DOXYGEN_IMAGE_PATH
		set (DOXYGEN_PROJECT_NAME ${PROJECT_NAME})
		set (DOXYGEN_SOURCE_ROOT ${BASE_DIR_NAME})
	#	set (DOXYGEN_IMAGE_PATH "") # TODO
		set (DOXYGEN_IMAGE_PATH "${BASE_DIR_NAME}/../doc/img/") 

        # - DOXYGEN_GRAPHVIZ_BINARY_PATH
        # - DOXYGEN_GRAPHVIZ_FOUND_YESNO
		if (GRAPHVIZ_FOUND)
			#message(STATUS "Graphviz found in \"${GRAPHVIZ_BINARY_PATH}\"")
			set (DOXYGEN_GRAPHVIZ_BINARY_PATH ${GRAPHVIZ_BINARY_PATH})
			set (DOXYGEN_GRAPHVIZ_FOUND_YESNO "YES")
		else (GRAPHVIZ_FOUND)
			message(SEND_ERROR "Graphviz not found! Please install it manually!")
			set (DOXYGEN_GRAPHVIZ_BINARY_PATH "")
			set (DOXYGEN_GRAPHVIZ_FOUND_YESNO "NO")
		endif (GRAPHVIZ_FOUND)

        # - DOXYGEN_HTMLHELPWORKSHOP_BINARY
        # - DOXYGEN_HTMLHELPWORKSHOP_FOUND_YESNO
		if (HTMLHELPWORKSHOP_FOUND)
			message(STATUS "Documentation for \"${PROJECT_NAME}\" (${TARGET_NAME}) will be a single CHM file")
			#message(STATUS "HTML Help Workshop found in \"${HTMLHELPWORKSHOP_BINARY_PATH}\"")
			set (DOXYGEN_HTMLHELPWORKSHOP_BINARY ${HTMLHELPWORKSHOP_BINARY})
			set (DOXYGEN_HTMLHELPWORKSHOP_FOUND_YESNO "YES")
		else (HTMLHELPWORKSHOP_FOUND)
			message(STATUS "Documentation for \"${PROJECT_NAME}\" (${TARGET_NAME}) will be HTML files")
			set (DOXYGEN_HTMLHELPWORKSHOP_BINARY "")
			set (DOXYGEN_HTMLHELPWORKSHOP_FOUND_YESNO "NO")
		endif (HTMLHELPWORKSHOP_FOUND)

        # - DOXYGEN_INPUT
		set (DOXYGEN_INPUT "")
		foreach (__FILE__ ${__FILES__})
			if (NOT IS_ABSOLUTE ${__FILE__})
				set (__FILE__ "${CMAKE_CURRENT_SOURCE_DIR}/${__FILE__}")
			endif (NOT IS_ABSOLUTE ${__FILE__})
			set (DOXYGEN_INPUT "${DOXYGEN_INPUT} \"${__FILE__}\"")
		endforeach (__FILE__)

        # - DOXYGEN_INTERNAL_DOCS_YESNO
		if (__INTERNAL_DOCS__)
			set (DOXYGEN_INTERNAL_DOCS_YESNO "YES")
		else (__INTERNAL_DOCS__)
			set (DOXYGEN_INTERNAL_DOCS_YESNO "NO")
		endif (__INTERNAL_DOCS__)

        # - DOXYGEN_EXAMPLE_PATH
        xme_get (__EXAMPLE_PATHS__ PROPERTY_GLOBAL "XME_CODE_EXAMPLE_PATHS")
        set (DOXYGEN_EXAMPLE_PATH "")
        foreach (__EXAMPLE_PATH__ ${__EXAMPLE_PATHS__})
            set (DOXYGEN_EXAMPLE_PATH "${DOXYGEN_EXAMPLE_PATH} \"${__EXAMPLE_PATH__}\"")
        endforeach (__EXAMPLE_PATH__)

		# Generate the configuration file
		configure_file ("${DOXYGEN_DOXYFILE_TEMPLATE}" "${__CONFIG__}" @ONLY)

		# Create output directory
		file (MAKE_DIRECTORY "${DOXYGEN_OUTPUT_DIRECTORY}")

		# Clean the workspace
		file (REMOVE "${__SCRIPT__}")
		if (${__CLEAN__})
			file(
				WRITE "${__SCRIPT__}"
				"message (STATUS \"Cleaning documentation output directory \\\"${DOXYGEN_OUTPUT_DIRECTORY}\\\"...\")
				execute_process (COMMAND \"${CMAKE_COMMAND}\" -E remove_directory \"${DOXYGEN_OUTPUT_DIRECTORY}\")
"
			)
		endif (${__CLEAN__})

		# Run Doxygen
		file(
			APPEND "${__SCRIPT__}"
			"message (STATUS \"Building '${TARGET_NAME}' into \\\"${DOXYGEN_OUTPUT_DIRECTORY}\\\" (logging stderr to \\\"${CMAKE_CURRENT_BINARY_DIR}/doxygen.${TARGET_NAME}.log\\\")...\")
			execute_process (COMMAND \"${DOXYGEN_BINARY}\" \"${__CONFIG__}\" ERROR_FILE \"${CMAKE_CURRENT_BINARY_DIR}/doxygen.${TARGET_NAME}.log\" TIMEOUT 3600)
"
		)

		# Add a custom target to execute the script
		add_custom_target(
			${TARGET_NAME}
			COMMAND ${CMAKE_COMMAND} -P "${__SCRIPT__}"
			COMMENT "Creating Doxygen documentation for ${PROJECT_NAME}..."
		)

		if (__AUTO__)
			message (STATUS "Scheduling '${TARGET_NAME}' documentation to be automatically built when '${CMAKE_PROJECT_NAME}' is built.")
			add_dependencies(
				${CMAKE_PROJECT_NAME}
				${TARGET_NAME}
			)
		endif (__AUTO__)

		# Install help file(s)
		if (NOT ${__INSTALL_DIR__} STREQUAL "")
			# Create output directory
			file (MAKE_DIRECTORY "${__INSTALL_DIR__}")

			if (HTMLHELPWORKSHOP_FOUND)
				install(
					FILES "${DOXYGEN_OUTPUT_DIRECTORY}/html/${DOXYGEN_CHM_FILE}"
					DESTINATION "${__INSTALL_DIR__}"
	    	    )
			else (HTMLHELPWORKSHOP_FOUND)
				install(
					DIRECTORY "${DOXYGEN_OUTPUT_DIRECTORY}/html"
					DESTINATION "${__INSTALL_DIR__}"
					PATTERN "*.dot" EXCLUDE
					PATTERN "*.map" EXCLUDE
					PATTERN "*.md5" EXCLUDE
				)
			endif (HTMLHELPWORKSHOP_FOUND)
		endif (NOT ${__INSTALL_DIR__} STREQUAL "")

	endif (NOT DOXYGEN_FOUND)

endmacro (doxygen_generate_documentation)
