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
# $Id: CMakeLists.txt 6273 2014-01-09 13:37:24Z geisinger $
#
# File:
#         CMake configuration script for Windows version of XME core components.
#

xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/defines/checkSideEffects)
xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/log/winPosix)
xme_add_subdirectory(${XME_SRC_DIR}/ports/software/os/generic-os/time/generic)

xme_add_component(
    "xme_core_fallback"
    deprecated_arch.h
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
    "deprecation_windows"
    # Heap related
    TEST wmemset CODE "(void)wmemset(wbuf, 0, 0)"                                  EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wmemcpy CODE "(void)wmemcpy(wbuf, wstr, 0)"                               EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wmemcmp CODE "(void)wmemcmp(wstr, wstr, 0)"                               EXPECT ${IS_DEPRECATION_DISABLED}
    # String related
    TEST vsnprintf       CODE "(void)vsnprintf(buf, 0, str, args)"                 EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST strncpy         CODE "(void)strncpy(buf, str, 0)"                         EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST strncat         CODE "(void)strncat(buf, str, 0)"                         EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _scprintf       CODE "(void)_scprintf(str)"                               EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _scwprintf      CODE "(void)_scwprintf(wstr)"                             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vscprintf      CODE "(void)_vscprintf(str, args)"                        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vscwprintf     CODE "(void)_vscwprintf(wstr, args)"                      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _snprintf       CODE "(void)_snprintf(buf, BSZ, str)"                     EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _snwprintf      CODE "(void)_snwprintf(wbuf, BSZ, wstr)"                  EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST vsprintf_s      CODE "(void)vsprintf_s(buf, BSZ, str, args)"              EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsprintf_s_l   CODE "(void)_vsprintf_s_l(buf, BSZ, str, 0, args)"        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST vswprintf_s     CODE "(void)vswprintf_s(wbuf, BSZ, wstr, args)"           EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vswprintf_s_l  CODE "(void)_vswprintf_s_l(wbuf, BSZ, wstr, 0, args)"     EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsnprintf      CODE "(void)_vsnprintf(buf, BSZ, str, args)"              EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _vsnprintf_l    CODE "(void)_vsnprintf_l(buf, BSZ, str, 0, args)"         EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST vsnprintf_s     CODE "(void)vsnprintf_s(buf, BSZ, 0, str, args)"          EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsnprintf_s    CODE "(void)_vsnprintf_s(buf, BSZ, 0, str, args)"         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsnprintf_s_l  CODE "(void)_vsnprintf_s_l(buf, BSZ, 0, str, 0, args)"    EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsnwprintf     CODE "(void)_vsnwprintf(wbuf, BSZ, wstr, args)"           EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _vsnwprintf_s   CODE "(void)_vsnwprintf_s(wbuf, BSZ, 0, wstr, args)"      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _vsnwprintf_l   CODE "(void)_vsnwprintf_l(wbuf, BSZ, wstr, 0, args)"      EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _vsnwprintf_s_l CODE "(void)_vsnwprintf_s_l(wbuf, BSZ, 0, wstr, 0, args)" EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strlen_l        CODE "(void)strlen_l(buf, 0)"                             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcslen_l        CODE "(void)wcslen_l(wstr, 0)"                            EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbslen         CODE "(void)_mbslen(mbstr)"                               EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbslen_l       CODE "(void)_mbslen_l(mbstr, 0)"                          EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbstrlen       CODE "(void)_mbstrlen(str)"                               EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbstrlen_l     CODE "(void)_mbstrlen_l(str, 0)"                          EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strnlen_s       CODE "(void)strnlen_s(str, 0)"                            EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strnlen_l       CODE "(void)strnlen_l(str, 0, 0)"                         EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcsnlen_s       CODE "(void)wcsnlen_s(wstr, 0)"                           EXPECT ${IS_DEPRECATION_DISABLED}
    TEST wcsnlen_l       CODE "(void)wcsnlen_l(wstr, 0, 0)"                        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbsnlen        CODE "(void)_mbsnlen(mbstr, 0)"                           EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbsnlen_l      CODE "(void)_mbsnlen_l(mbstr, 0, 0)"                      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbstrnlen      CODE "(void)_mbstrnlen(str, 0)"                           EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbstrnlen_l    CODE "(void)_mbstrnlen_l(str, 0, 0)"                      EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strncpy_s       CODE "(void)strncpy_s(buf, BSZ, str, 0)"                  EXPECT ${IS_DEPRECATION_DISABLED}
    #TEST _strncpy_s_l    CODE "(void)_strncpy_s_l(buf, 0, str, 0, 0)"              EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
    TEST wcsncpy_s       CODE "(void)wcsncpy_s(wbuf, BSZ, wstr, 0)"                EXPECT ${IS_DEPRECATION_DISABLED}
    #TEST _wcsncpy_s_l    CODE "(void)_wcsncpy_s_l(wbuf, BSZ, wstr, 0, 0)"          EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
    TEST _mbsncpy_s      CODE "(void)_mbsncpy_s(mbbuf, BSZ, mbstr, 0)"             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbsncpy_s_l    CODE "(void)_mbsncpy_s_l(mbbuf, BSZ, mbstr, 0, 0)"        EXPECT ${IS_DEPRECATION_DISABLED}
    TEST strncat_s       CODE "(void)strncat_s(buf, BSZ, str, 0)"                  EXPECT ${IS_DEPRECATION_DISABLED}
    #TEST _strncat_s_l    CODE "(void)_strncat_s_l(buf, BSZ, str, 0, 0)"            EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
    TEST wcsncat_s       CODE "(void)wcsncat_s(wbuf, BSZ, wstr, 0)"                EXPECT ${IS_DEPRECATION_DISABLED}
    #TEST _wcsncat_s_l    CODE "(void)_wcsncat_s_l(wbuf, BSZ, wstr, 0, 0)"          EXPECT ${IS_DEPRECATION_DISABLED}   # Causes linking problems.
    TEST _mbsncat_s      CODE "(void)_mbsncat_s(mbbuf, BSZ, mbstr, 0)"             EXPECT ${IS_DEPRECATION_DISABLED}
    TEST _mbsncat_s_l    CODE "(void)_mbsncat_s_l(mbbuf, BSZ, mbstr, 0, 0)"        EXPECT ${IS_DEPRECATION_DISABLED}
    # File input/output related
    TEST fopen           CODE "(void)fopen(\"nonexisting\", \"r\")"                EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _wfopen         CODE "(void)_wfopen(L\"nonexisting\", L\"r\")"            EXPECT FAIL   # Will always fail except if _CRT_SECURE_NO_WARNINGS is defined.
    TEST _fcloseall      CODE "(void)_fcloseall()"                                 EXPECT ${IS_DEPRECATION_DISABLED}
)
