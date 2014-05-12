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
# $Id: CMakeLists.txt 4349 2013-07-24 13:52:45Z geisinger $
#
# File:
#         CMake configuration script for posix version of XME core components.
#

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/defines/checkSideEffects)
xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/deprecated/generic)
xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/log/winPosix)
xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/time/generic)

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
    "deprecation_posix"
    # String related
    TEST sprintf   CODE "(void)sprintf(buf, \"Test\")"                   EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use str, because GCC expects a string literal.
    TEST vsprintf  CODE "(void)vsprintf(buf, \"Test\", args)"            EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use str, because GCC expects a string literal.
    TEST snprintf  CODE "(void)snprintf(buf, BSZ, \"Test\")"             EXPECT ${IS_DEPRECATION_DISABLED}   # Can't use str, because GCC expects a string literal.
    TEST strcpy    CODE "(void)strcpy(buf, str)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strcat    CODE "(void)strcat(buf, str)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcscat    CODE "(void)wcscat(wbuf, wstr)"                       EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcsncat   CODE "(void)wcsncat(wbuf, wstr, 0)"                   EXPECT ${IS_DEPRECATION_DISABLED}
    # File input/output related
    TEST fscanf    CODE "{int n = fscanf(stdout, \"Test\"); (void)n;}"   EXPECT ${IS_DEPRECATION_DISABLED}   # Intentionally using stdout here, because reading on stdin would block. Can't use str, because GCC expects a string literal.
    TEST fwscanf   CODE "{int n = fwscanf(stdout, L\"Test\"); (void)n;}" EXPECT ${IS_DEPRECATION_DISABLED}   # Intentionally using stdout here, because reading on stdin would block. Can't use wstr, because GCC expects a string literal.
)
