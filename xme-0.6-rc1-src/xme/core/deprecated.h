/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: deprecated.h 4595 2013-08-07 13:49:46Z ruiz $
 */

/**
 * \file
 *         Flagging of various native functions as deprecated.
 */

#ifndef XME_CORE_DEPRECATED_H
#define XME_CORE_DEPRECATED_H

#ifndef XME_DEFINES_H
    #error This header file should not be included directly. Include xme/defines.h instead.
#endif // #ifndef XME_DEFINES_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// Avoid warnings about C++ templates that contain calls to deprecated
// functions. These warnings appear, because templates are only instantiated
// when they are actually needed. Note that the same applies to C/C++ defines.
// However, currently there does not seem to be such a case.
#if defined(__cplusplus) && defined(XME_ENABLE_DEPRECATION)
#undef XME_ENABLE_DEPRECATION /* no deprecation on C++ */
#endif

#ifndef XME_ENABLE_DEPRECATION
    #define XME_DEPRECATE_TEXT(text, func) func
#else // #ifndef XME_ENABLE_DEPRECATION
    #ifdef _MSC_VER
        // Microsoft compiler supports output of nice warnings for deprecated functions
        #define XME_DEPRECATE_TEXT(text, func) __declspec(deprecated(text)) func
    #else // #ifdef _MSC_VER
        // No nice warnings on other compilers
        #define XME_DEPRECATE_TEXT(text, func) func __attribute__((deprecated))
    #endif // #ifdef _MSC_VER
#endif // #ifndef XME_ENABLE_DEPRECATION

#define XME_DEPRECATE_FUNCTION(modifier, rval, original, params) \
    XME_DEPRECATE_TEXT("Calling " #original "() directly is not recommended in CHROMOSOME, but no suitable platform abstraction exists yet. You might want to create one. To disable this warning, undefine XME_ENABLE_DEPRECATION.", modifier rval original params)

#define XME_DEPRECATE_REPLACED_FUNCTION(modifier, rval, original, params, replacement) \
    XME_DEPRECATE_TEXT("Calling " #original "() directly is not recommended in CHROMOSOME. Use " #replacement "() with appropriate arguments instead. To disable this warning, undefine XME_ENABLE_DEPRECATION.", modifier rval original params)

#define XME_DEPRECATE_FUNCTION_WITH_FALLBACK(modifier, rval, original, params, fallback_body) \
    static INLINE rval xme_fallback_##original params; /* Avoid 'no previous prototype' warning */ \
    static INLINE rval xme_fallback_##original params { fallback_body } \
    XME_DEPRECATE_FUNCTION(modifier, rval, original, params)

#define XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK(modifier, rval, original, params, replacement, fallback_body) \
    static INLINE rval xme_fallback_##original params; /* Avoid 'no previous prototype' warning */ \
    static INLINE rval xme_fallback_##original params { fallback_body } \
    XME_DEPRECATE_REPLACED_FUNCTION(modifier, rval, original, params, replacement)

#define XME_CORE_MODIFIER_NONE /* nothing */

#ifdef _WIN32
    #define XME_CORE_MODIFIER_DECLSPEC _declspec(dllimport)
#else // #ifdef _WIN32
    #define XME_CORE_MODIFIER_DECLSPEC /* nothing */
#endif // #ifdef _WIN32

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/core/deprecated_arch.h"

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

//----------------------------------------------------------------------------//
//     Heap related                                                           //
//----------------------------------------------------------------------------//

// malloc()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, void*, malloc, (size_t size), xme_hal_mem_alloc,
    {
        return malloc(size);
    }
);

// calloc()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, void*, calloc, (size_t num, size_t size), xme_hal_mem_alloc,
    {
        return calloc(num, size);
    }
);

// realloc()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, void*, realloc, (void* ptr, size_t size), xme_hal_mem_realloc,
    {
        return realloc(ptr, size);
    }
);

// free()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, void, free, (void* ptr), xme_hal_mem_free,
    {
        free(ptr);
    }
);

// memset()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_NONE, void*, memset, (void* ptr, int value, size_t num), xme_hal_mem_set,
    {
        return memset(ptr, value, num);
    }
);

// memcpy()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_NONE, void*, memcpy, (void* destination, const void* source, size_t num), xme_hal_mem_copy,
    {
        return memcpy(destination, source, num);
    }
);

// memcmp()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_NONE, int, memcmp, (const void* buf1, const void* buf2, size_t num), xme_hal_mem_compare,
    {
        return memcmp(buf1, buf2, num);
    }
);

//----------------------------------------------------------------------------//
//     printf related                                                         //
//----------------------------------------------------------------------------//

// printf()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, printf, (const char * format, ...), XME_LOG,
    {
        int result;
        va_list args;

        va_start(args, format);
        result = vprintf(format, args);
        va_end(args);

        return result;
    }
);

// vprintf()
XME_DEPRECATE_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, vprintf, (const char* format, va_list args),
    {
        return vprintf(format, args);
    }
);

// vwprintf()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, vwprintf, (const wchar_t* format, va_list args));

//----------------------------------------------------------------------------//
//     String related                                                         //
//----------------------------------------------------------------------------//

// sprintf()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, sprintf, (char* buffer, const char* format, ...), xme_hal_safeString_snprintf);

// swprintf()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, int, swprintf, (wchar_t* buffer, size_t count, const wchar_t* format, ...));

// vsprintf()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, vsprintf, (char* buffer, const char* format, va_list args), xme_hal_safeString_vsnprintf);

// vswprintf()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, int, vswprintf, (wchar_t* buffer, size_t count, const wchar_t* format, va_list args));

// snprintf()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_NONE, int, snprintf, (char* buffer, size_t n, const char * format, ...), xme_hal_safeString_snprintf);

// strcpy()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_NONE, char*, strcpy, (char* destination, const char* source), xme_hal_safeString_strncpy);

// strcat()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_NONE, char*, strcat, (char* destination, const char* source), xme_hal_safeString_strncat);

// wcscat()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, wchar_t*, wcscat, (wchar_t* destination, const wchar_t* source));

// wcsncat()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, wchar_t*, wcsncat, (wchar_t* destination, const wchar_t* source, size_t count));

//----------------------------------------------------------------------------//
//     File input/output related                                              //
//----------------------------------------------------------------------------//

// fclose()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, fclose, (FILE* stream), xme_hal_fileio_fclose,
    {
        return fclose(stream);
    }
);

// fread()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, size_t, fread, (void* buffer, size_t size, size_t count, FILE* stream), xme_hal_fileio_fread,
    {
        return fread(buffer, size, count, stream);
    }
);

// fwrite()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, size_t, fwrite, (const void* buffer, size_t size, size_t count, FILE* stream), xme_hal_fileio_fwrite,
    {
        return fwrite(buffer, size, count, stream);
    }
);

// fseek()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, fseek, (FILE* stream, long offset, int origin), xme_hal_fileio_fseek,
    {
        return fseek(stream, offset, origin);
    }
);

// ftell()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, long, ftell, (FILE* stream), xme_hal_fileio_ftell,
    {
        return ftell(stream);
    }
);

// fsetpos()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, fsetpos, (FILE* stream, const fpos_t* pos), xme_hal_fileio_fseek);

// fgetpos()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, fgetpos, (FILE* stream, fpos_t* pos), xme_hal_fileio_ftell);

// rewind()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, void, rewind, (FILE* stream), xme_hal_fileio_fseek);

// fgetc()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, fgetc, (FILE* stream), xme_hal_fileio_fgetc,
    {
        return fgetc(stream);
    }
);

// fgetwc()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, wint_t, fgetwc, (FILE* stream));

// fputc()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, fputc, (int c, FILE* stream), xme_hal_fileio_fputc,
    {
        return fputc(c, stream);
    }
);

// fputwc()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, wint_t, fputwc, (wchar_t c, FILE* stream));

// fputs()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, fputs, (const char* string, FILE* stream), xme_hal_fileio_fputs,
    {
        return fputs(string, stream);
    }
);

// fputws()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, fputws, (const wchar_t* string, FILE* stream));

// fscanf()
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, fscanf, (FILE* stream, const char* format, ...), xme_hal_fileio_fscanf);

// fwscanf()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, fwscanf, (FILE* stream, const wchar_t* format, ...));

// vfprintf()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, vfprintf, (FILE* stream, const char* format, va_list args), xme_hal_fileio_vfprintf,
    {
        return vfprintf(stream, format, args);
    }
);

// vfwprintf()
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, vfwprintf, (FILE* stream, const wchar_t* format, va_list args));

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DEPRECATED_H
