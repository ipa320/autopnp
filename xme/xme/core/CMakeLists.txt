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
# $Id: CMakeLists.txt 6330 2014-01-14 18:27:53Z geisinger $
#
# File:
#         CMake configuration file for XME core components.
#

#------------------------------------------------------------------------------#
#-     Core components                                                        -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_core"
    component.h
    componentInfo.h
    componentInfo.c
    componentList.h
    container.h
    coreTypes.h
    dataChannel.h
    device.h
    manifestTypes.h
    testUtils.h
    topic.h
    topicData.h
    xme_api.h
)

xme_add_component(
    "xme_core_fallback"
    deprecated.h
)

#------------------------------------------------------------------------------#
#-     xme_core_log                                                           -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_log"
    log.h
    logUtils.h
    log.c
    logUtils.c
    xme_hal_sync
    xme_hal_table
)

xme_unit_test(
    "xme_core_log"
    TYPE smoke
    test/smokeTestCoreLog.cpp
)

#------------------------------------------------------------------------------#
#-     xme_core_node                                                          -#
#------------------------------------------------------------------------------#

xme_add_component(
    "xme_core_node"
    node.h
    node.c
    xme_com_interface
    xme_defines
    xme_hal_mem
    xme_hal_safeString
    xme_hal_table
)

xme_unit_test(
    "xme_core_node"
    TYPE smoke
    test/smokeTestCoreNode.cpp
)

xme_unit_test(
    "xme_core_node"
    TYPE interface
    test/interfaceTestCoreNode.cpp
)

#------------------------------------------------------------------------------#
#-     Compilation tests                                                      -#
#------------------------------------------------------------------------------#

if (XME_ENABLE_DEPRECATION)
    set (IS_DEPRECATION_DISABLED FALSE)
else ()
    set (IS_DEPRECATION_DISABLED TRUE)
endif ()

# The following snippets of code should either fail or succeed compiling
# depending on whether the XME_ENABLE_DEPRECATION CMake variable is TRUE
# or FALSE.
# When deprecation is enabled (i.e., XME_ENABLE_DEPRECATION is TRUE),
# then it is not allowed to call those functions directly.
# Instead, the respective replacements from the CHROMOSOME HAL should
# be used. Disabling XME_ENABLE_DEPRECATION will flip the logic such
# that compiling the respective files must succeed.
xme_compilation_tests(
    "deprecation_generic"
    # Heap related
    TEST malloc    CODE "{void* p = malloc(0); (void)p;}"                EXPECT ${IS_DEPRECATION_DISABLED}
    TEST calloc    CODE "{void* p = calloc(0, 0); (void)p;}"             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST realloc   CODE "ptr = realloc(NULL, 0)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST free      CODE "free(NULL)"                                     EXPECT ${IS_DEPRECATION_DISABLED}
    TEST memset    CODE "(void)memset(buf, 0, 0)"                        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST memcpy    CODE "(void)memcpy(buf, str, 0)"                      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST memcmp    CODE "(void)memcmp(str, str, 0)"                      EXPECT ${IS_DEPRECATION_DISABLED}
    # printf related
    TEST printf    CODE "(void)printf(\"Test\")"                         EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use str, because GCC expects a string literal.
    TEST vprintf   CODE "(void)vprintf(\"Test\", args)"                  EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use str, because GCC expects a string literal.
    TEST vwprintf  CODE "(void)vwprintf(L\"Test\", args)"                EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use wstr, because GCC expects a string literal.
    # String related
#    TEST scprintf  CODE "(void)scprintf(str)"                            EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
    TEST swprintf  CODE "(void)swprintf(wbuf, BSZ, wstr)"                EXPECT ${IS_DEPRECATION_DISABLED}
    TEST vswprintf CODE "(void)vswprintf(wbuf, BSZ, wstr, args)"         EXPECT ${IS_DEPRECATION_DISABLED}
#    TEST snprintf  CODE "(void)snprintf(buf, 0, str)"                    EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
#    TEST strlen    CODE "(void)strlen(str)"                              EXPECT ${IS_DEPRECATION_DISABLED}   # Not yet marked as deprecated.
    TEST strnlen   CODE "(void)strnlen(str, 0)"                          EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcslen    CODE "(void)wcslen(wstr)"                             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcsnlen   CODE "(void)wcsnlen(wstr, 0)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    # File input/output related
    TEST fclose    CODE "(void)fclose(stdout)"                           EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fread     CODE "{size_t s = fread(NULL, 0, 0, NULL); (void)s;}" EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fwrite    CODE "(void)fwrite(str, 0, 0, stdout)"                EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fseek     CODE "(void)fseek(stdin, 0, 0)"                       EXPECT ${IS_DEPRECATION_DISABLED}
    TEST ftell     CODE "{long int n = ftell(stdin); (void)n;}"          EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fsetpos   CODE "{fpos_t pos;(void)fsetpos(stdout, &pos);}"      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fgetpos   CODE "{fpos_t pos;(void)fgetpos(stdin, &pos);}"       EXPECT ${IS_DEPRECATION_DISABLED}
    TEST rewind    CODE "rewind(stdin)"                                  EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fgetc     CODE "(void)fgetc(stdout)"                            EXPECT ${IS_DEPRECATION_DISABLED}   # Intentionally using stdout here, because reading on stdin would block.
    TEST fgetwc    CODE "(void)fgetwc(stdout)"                           EXPECT ${IS_DEPRECATION_DISABLED}   # Intentionally using stdout here, because reading on stdin would block.
    TEST fputc     CODE "(void)fputc(0, stdout)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fputwc    CODE "(void)fputwc(0, stdout)"                        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fputs     CODE "(void)fputs(str, stdout)"                       EXPECT ${IS_DEPRECATION_DISABLED}
    TEST fputws    CODE "(void)fputws(wstr, stdout)"                     EXPECT ${IS_DEPRECATION_DISABLED}
    TEST vfprintf  CODE "(void)vfprintf(stdout, str, args)"              EXPECT ${IS_DEPRECATION_DISABLED}
    TEST vfwprintf CODE "(void)vfwprintf(stdout, wstr, args)"            EXPECT ${IS_DEPRECATION_DISABLED}
)

#------------------------------------------------------------------------------#
#-     Subdirectrories                                                        -#
#------------------------------------------------------------------------------#

xme_add_subdirectory (broker)
xme_add_subdirectory (dataHandler)
xme_add_subdirectory (directory)
xme_add_subdirectory (executionManager)
xme_add_subdirectory (login)
xme_add_subdirectory (manifestRepository)
xme_add_subdirectory (nodeManager)
xme_add_subdirectory (plugAndPlay)
xme_add_subdirectory (rteScheduler)
