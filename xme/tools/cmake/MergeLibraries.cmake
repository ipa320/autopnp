# Merge static libraries into a big static lib. 
# The resulting library should not not have dependencies on other static libraries.
macro(merge_static_libs TARGET LIBS_TO_MERGE)
    #message(STATUS "MERGE_STATIC_LIBS TARGET = ${TARGET}, LIBS_TO_MERGE = ${LIBS_TO_MERGE}")
    # To produce a library we need at least one source file.
    # It is created by ADD_CUSTOM_COMMAND below and will helps also help to track dependencies.
    set(SOURCE_FILE ${CMAKE_CURRENT_BINARY_DIR}/${TARGET}_depends.c)
    add_library(${TARGET} STATIC ${SOURCE_FILE})
    
    set(OSLIBS)
    foreach(LIB ${LIBS_TO_MERGE})
        get_target_property(LIB_LOCATION ${LIB} LOCATION)
        get_target_property(LIB_TYPE ${LIB} TYPE)
        if(NOT LIB_LOCATION)
            # 3rd party library like libz.so. Make sure that everything
            # that links to our library links to this one as well.
            list(APPEND OSLIBS ${LIB})
        else(NOT LIB_LOCATION)
            # This is a target in current project (can be a static or shared lib)
            if(LIB_TYPE STREQUAL "STATIC_LIBRARY")
                set(STATIC_LIBS ${STATIC_LIBS} ${LIB_LOCATION})
                add_dependencies(${TARGET} ${LIB})
                # Extract dependend OS libraries
                GET_DEPENDEND_OS_LIBS(${LIB} LIB_OSLIBS)
                list(APPEND OSLIBS ${LIB_OSLIBS})
            else(LIB_TYPE STREQUAL "STATIC_LIBRARY")
                # This is a shared library our static lib depends on.
                list(APPEND OSLIBS ${LIB})
            endif(LIB_TYPE STREQUAL "STATIC_LIBRARY")
        endif(NOT LIB_LOCATION)
    endforeach(LIB ${LIBS_TO_MERGE})
  
    if(OSLIBS)
        list(REMOVE_DUPLICATES OSLIBS)
        target_link_libraries(${TARGET} ${OSLIBS})
    endif(OSLIBS)

    # Make the generated dummy source file depended on all static input libs. 
    # If input lib changes,the source file is touched which causes the desired effect (relink).
    add_custom_command(OUTPUT  ${SOURCE_FILE}
                       COMMAND ${CMAKE_COMMAND} -E touch ${SOURCE_FILE}
                       DEPENDS ${STATIC_LIBS}
                       IMPLICIT_DEPENDS ${STATIC_LIBS})

    if(MSVC)
        # To merge libs, just pass them to lib.exe command line.
        set(LINKER_EXTRA_FLAGS "")
        foreach(LIB ${STATIC_LIBS})
            set(LINKER_EXTRA_FLAGS "${LINKER_EXTRA_FLAGS} ${LIB}")
        endforeach()
        set_target_properties(${TARGET} PROPERTIES STATIC_LIBRARY_FLAGS "${LINKER_EXTRA_FLAGS}")
    elseif(APPLE)
        get_target_property(TARGET_LOCATION ${TARGET} LOCATION)  
        # Use OSX's libtool to merge archives (ihandles universal binaries properly)
        add_custom_command(TARGET ${TARGET} POST_BUILD
                           COMMAND rm ${TARGET_LOCATION}
                            COMMAND /usr/bin/libtool -static -o ${TARGET_LOCATION} 
                            ${STATIC_LIBS})  
    elseif(UNIX OR  PikeOS)
        get_target_property(TARGET_LOCATION ${TARGET} LOCATION)
        # Generic Unix, Cygwin or MinGW. In post-build step, call script, 
        # that extracts objects from archives with "ar x" and repacks them with "ar r"
        set(TARGET ${TARGET})

        find_file(MERGE_ARCHIVE_TEMPLATE
                  NAMES "merge_archives.in"
                  PATHS ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_MODULE_PATH}
                  NO_DEFAULT_PATH
                  NO_CMAKE_FIND_ROOT_PATH)
        
        configure_file(${MERGE_ARCHIVE_TEMPLATE} 
                       ${CMAKE_CURRENT_BINARY_DIR}/merge_${TARGET}.cmake
                       @ONLY)
        
        add_custom_command(TARGET ${TARGET}
                           COMMAND rm ${TARGET_LOCATION}
                           COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/merge_${TARGET}.cmake)
    else()
        message(FATAL_ERROR "Platform doesn't support merge of static libaries. Thus you have to implement it. Good bye and have a nice day.")
    endif(MSVC)
endmacro()

# Create libs from libs.
# Merges static libraries, creates shared libraries out of convenience libraries.
macro(merge_libraries)
    set (__USAGE__ "Usage: merge_libraries ( <library name> STATIC|DYNAMIC <library 1> ... <library N> )")
    if (${ARGC} LESS 1)
        message (FATAL_ERROR ${__USAGE__})
    endif (${ARGC} LESS 1)
    
    set (__ARGS__ ${ARGV})
    list (GET __ARGS__ 0 __TARGET_NAME__)
    list (REMOVE_AT __ARGS__ 0)
    list (LENGTH __ARGS__ __ARGC__)
    if (${__ARGC__} LESS 1)
        message (FATAL_ERROR ${__USAGE__})
    endif (${__ARGC__} LESS 1)
    
    set(_target_name "${__TARGET_NAME__}")
    set(_libs )
    set(_export)
    
    set(_static 0)
    set(_shared 0)
    set(_module 0)
    set(_install 0)
    set(__MODE__ 0)
    foreach(_sourcefile ${__ARGS__})
        if(${_sourcefile} MATCHES "STATIC")
            set(_static 1)
        elseif(${_sourcefile} MATCHES "SHARED")
            set(_shared 1)
        elseif(${_sourcefile} MATCHES "MODULE")
            set(_module 1)
        elseif(${_sourcefile} MATCHES "EXPORTS")
            set(__MODE__ 4)
        elseif(${_sourcefile} MATCHES "NOINSTALL")
            set(_install 1)
        elseif(__MODE__ EQUAL 4)
            list(APPEND _export ${_sourcefile})
            set(__MODE__ 0)
        else()
            list(APPEND _libs ${_sourcefile})
        endif()
    endforeach(_sourcefile)
     
    if(_static)
        merge_static_libs(${_target_name} "${_libs}") 
    elseif(_shared OR _module)
        if(_shared)
            set(LIBTYPE SHARED)
        else(_module)
            set(LIBTYPE MODULE)
        endif(_shared)
        
        # check for non-PIC libraries
        if(NOT _SKIP_PIC)
            foreach(LIB ${_libs})
                get_target_property(${LIB} TYPE LIBTYPE)
                if(LIBTYPE STREQUAL "STATIC_LIBRARY")
                    get_target_property(LIB COMPILE_FLAGS LIB_COMPILE_FLAGS)
                    string(REPLACE "${CMAKE_SHARED_LIBRARY_C_FLAGS}" "<PIC_FLAG>" LIB_COMPILE_FLAGS ${LIB_COMPILE_FLAG})
                    if(NOT LIB_COMPILE_FLAGS MATCHES "<PIC_FLAG>")
                        message(FATAL_ERROR "Attempted to link non-PIC static library ${LIB} to shared library ${TARGET}\n"
                                            "Please use ADD_CONVENIENCE_LIBRARY, instead of ADD_LIBRARY for ${LIB}")
                    endif(NOT LIB_COMPILE_FLAGS MATCHES "<PIC_FLAG>")
                endif(LIBTYPE STREQUAL "STATIC_LIBRARY")
            endforeach(LIB ${_libs})
        endif(NOT _SKIP_PIC)
        
        CREATE_EXPORT_FILE(SRC ${_target_name} "${_export}")
        add_library(${_target_name} ${LIBTYPE} ${SRC})
        target_link_libraries(${_target_name} ${_libs})
    else(_static)
        message(FATAL_ERROR "Unknown library type")
    endif(_static)
endmacro()

function(GET_DEPENDEND_OS_LIBS target result)
    set(deps ${${target}_LIB_DEPENDS})
    if(deps)
        foreach(lib ${deps})
            # Filter out keywords for used for debug vs optimized builds
            if(NOT lib MATCHES "general" AND NOT lib MATCHES "debug" AND NOT lib MATCHES "optimized")
                GET_TARGET_PROPERTY(lib_location ${lib} LOCATION)
                if(NOT lib_location)
                    set(ret ${ret} ${lib})
                endif(NOT lib_location)
            endif(NOT lib MATCHES "general" AND NOT lib MATCHES "debug" AND NOT lib MATCHES "optimized")
        endforeach(lib ${deps})
    endif(deps)
    set(${result} ${ret} PARENT_SCOPE)
endfunction(GET_DEPENDEND_OS_LIBS target result)