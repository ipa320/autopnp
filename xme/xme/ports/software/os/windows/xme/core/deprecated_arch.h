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
 * $Id: deprecated_arch.h 6273 2014-01-09 13:37:24Z geisinger $
 */

/**
 * \file
 *         Flagging of various native functions as deprecated
 *         (Windows implementation).
 */

#ifndef XME_CORE_DEPRECATED_ARCH_H
#define XME_CORE_DEPRECATED_ARCH_H

#ifndef XME_CORE_DEPRECATED_H
    #error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_CORE_DEPRECATED_H

// See also http://msdn.microsoft.com/en-us/library/ms235384%28v=vs.80%29.aspx

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <errno.h>

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

//----------------------------------------------------------------------------//
//     Heap related                                                           //
//----------------------------------------------------------------------------//

// wmemset()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, wchar_t*, wmemset, (wchar_t* dest, wchar_t c, size_t count));
#endif // #ifdef _MSC_VER

// wmemcpy()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, wchar_t*, wmemcpy, (wchar_t* dest, const wchar_t* src, size_t count));
#endif // #ifdef _MSC_VER

// wmemcmp()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, int, wmemcmp, (const wchar_t* buf1, const wchar_t* buf2, size_t num));
#endif // #ifdef _MSC_VER

//----------------------------------------------------------------------------//
//     String related                                                         //
//----------------------------------------------------------------------------//

// vsnprintf()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, vsnprintf, (char* buffer, size_t count, const char* format, va_list args), xme_hal_safeString_vsnprintf,
    {
        return vsnprintf_s(buffer, count, _TRUNCATE, format, args);
    }
);

/* strcpy */

// strncpy()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, char*, strncpy, (char* destination, const char* source, size_t num), xme_hal_safeString_strncpy,
    {
        // Discard error indicator
        strncpy_s(destination, num, source, _TRUNCATE);
        return destination;
    }
);

/* strcat */

// strncat()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, char*, strncat, (char* destination, const char* source, size_t num), xme_hal_safeString_strncat,
    {
        // Discard error indicator
        strncat_s(destination, num, source, _TRUNCATE);
        return destination;
    }
);

/* scprintf */

// _scprintf()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _scprintf, (const char* format, ...), xme_hal_safeString_scprintf);
#endif // #ifdef _MSC_VER

// _scwprintf()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _scwprintf, (const wchar_t* format, ...));
#endif // #ifdef _MSC_VER

// _vscprintf()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, _vscprintf, (const char* format, va_list args), xme_hal_safeString_vscprintf,
    {
        return _vscprintf(format, args);
    }
);
#endif // #ifdef _MSC_VER

// _vscwprintf()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vscwprintf, (const wchar_t* format, va_list args));
#endif // #ifdef _MSC_VER

/* sprintf */

// _snprintf()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _snprintf, (char* buffer, size_t n, const char * format, ...), xme_hal_safeString_snprintf);
#endif // #ifdef _MSC_VER

// _snwprintf()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _snwprintf, (wchar_t* buffer, size_t count, const wchar_t* format, ...));
#endif // #ifdef _MSC_VER

// vsprintf_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, vsprintf_s, (char* buffer, size_t numberOfElements, const char* format, va_list args), xme_hal_safeString_vsnprintf,
    {
        return vsprintf_s(buffer, numberOfElements, format, args);
    }
);
#endif // #ifdef _MSC_VER

// _vsprintf_s_l
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsprintf_s_l, (char* buffer, size_t numberOfElements, const char* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

// vswprintf_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, vswprintf_s, (wchar_t* buffer, size_t numberOfElements, const wchar_t* format, va_list args));
#endif // #ifdef _MSC_VER

// _vswprintf_s_l
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vswprintf_s_l, (wchar_t* buffer, size_t numberOfElements, const wchar_t* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

// _vsnprintf()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnprintf, (char* buffer, size_t count, const char* format, va_list args), xme_hal_safeString_vsnprintf);
#endif // #ifdef _MSC_VER

// _vsnprintf_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnprintf_l, (char* buffer, size_t count, const char* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

// vsnprintf_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, vsnprintf_s, (char* buffer, size_t bufferSizeInBytes, size_t count, const char* format, va_list args), xme_hal_safeString_vsnprintf);
#endif // #ifdef _MSC_VER

// _vsnprintf_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, int, _vsnprintf_s, (char* buffer, size_t bufferSizeInBytes, size_t count, const char* format, va_list args), xme_hal_safeString_vsnprintf,
    {
        return _vsnprintf_s(buffer, bufferSizeInBytes, count, format, args);
    }
);
#endif // #ifdef _MSC_VER

// _vsnprintf_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnprintf_s_l, (char* buffer, size_t bufferSizeInBytes, size_t count, const char* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

// _vsnwprintf()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnwprintf, (wchar_t* buffer, size_t count, const wchar_t* format, va_list args));
#endif // #ifdef _MSC_VER

// _vsnwprintf_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnwprintf_s, (wchar_t* buffer, size_t bufferSizeInBytes, size_t count, const wchar_t* format, va_list args));
#endif // #ifdef _MSC_VER

// _vsnwprintf_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnwprintf_l, (wchar_t* buffer, size_t count, const wchar_t* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

// _vsnwprintf_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _vsnwprintf_s_l, (wchar_t* buffer, size_t bufferSizeInBytes, size_t count, const wchar_t* format, _locale_t locale, va_list args));
#endif // #ifdef _MSC_VER

/* strlen */

// strlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, strlen_l, (const char* string, _locale_t locale));
#endif // #ifdef _MSC_VER

// wcslen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, wcslen_l, (const wchar_t* string, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbslen()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbslen, (const unsigned char* string));
#endif // #ifdef _MSC_VER

// _mbslen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbslen_l, (const unsigned char* string, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbstrlen()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbstrlen, (const char* string));
#endif // #ifdef _MSC_VER

// _mbstrlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbstrlen_l, (const char* string, _locale_t locale));
#endif // #ifdef _MSC_VER

// strnlen_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION(XME_CORE_MODIFIER_NONE, size_t, strnlen_s, (const char* string, size_t count), xme_hal_safeString_strnlen);
#endif // #ifdef _MSC_VER

// strnlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, strnlen_l, (const char* string, size_t count, _locale_t locale));
#endif // #ifdef _MSC_VER

// wcsnlen_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, size_t, wcsnlen_s, (const wchar_t* string, size_t count));
#endif // #ifdef _MSC_VER

// wcsnlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, wcsnlen_l, (const wchar_t* string, size_t count, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbsnlen()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbsnlen, (const unsigned char* string, size_t count));
#endif // #ifdef _MSC_VER

// _mbsnlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbsnlen_l, (const unsigned char* string, size_t count, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbstrnlen()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbstrnlen, (const char* string, size_t count));
#endif // #ifdef _MSC_VER

// _mbstrnlen_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, size_t, _mbstrnlen_l, (const char* string, size_t count, _locale_t locale));
#endif // #ifdef _MSC_VER

/* strcpy */

// strncpy_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, errno_t, strncpy_s, (char* destination, size_t bufferSizeInBytes, const char* source, size_t num), xme_hal_safeString_strncpy,
    {
        return strncpy_s(destination, bufferSizeInBytes, source, num);
    }
);
#endif // #ifdef _MSC_VER

// strncpy_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _strncpy_s_l, (char* destination, size_t bufferSizeInBytes, const char* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

// wcsncpy_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, errno_t, wcsncpy_s, (wchar_t* destination, size_t bufferSizeInWords, const wchar_t* source, size_t num));
#endif // #ifdef _MSC_VER

// _wcsncpy_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _wcsncpy_s_l, (wchar_t* destination, size_t bufferSizeInWords, const wchar_t* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbsncpy_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _mbsncpy_s, (unsigned char* destination, size_t bufferSizeInBytes, const unsigned char* source, size_t num));
#endif // #ifdef _MSC_VER

// _mbsncpy_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _mbsncpy_s_l, (unsigned char* destination, size_t bufferSizeInBytes, const unsigned char* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

/* strcat */

// strncat_s()
#ifdef _MSC_VER
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, errno_t, strncat_s, (char* destination, size_t bufferSizeInBytes, const char* source, size_t num), xme_hal_safeString_strncat,
    {
        return strncat_s(destination, bufferSizeInBytes, source, num);
    }
);
#endif // #ifdef _MSC_VER

// _strncat_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _strncat_s_l, (char* destination, size_t bufferSizeInBytes, const char* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

// wcsncat_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, errno_t, wcsncat_s, (wchar_t* destination, size_t bufferSizeInWords, const wchar_t* source, size_t num));
#endif // #ifdef _MSC_VER

// _wcsncat_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _wcsncat_s_l, (wchar_t* destination, size_t bufferSizeInWords, const wchar_t* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

// _mbsncat_s()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _mbsncat_s, (unsigned char* destination, size_t bufferSizeInBytes, const unsigned char* source, size_t num));
#endif // #ifdef _MSC_VER

// _mbsncat_s_l()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_NONE, errno_t, _mbsncat_s_l, (unsigned char* destination, size_t bufferSizeInBytes, const unsigned char* source, size_t num, _locale_t locale));
#endif // #ifdef _MSC_VER

//----------------------------------------------------------------------------//
//     File input/output related                                              //
//----------------------------------------------------------------------------//

// fopen()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
    XME_CORE_MODIFIER_DECLSPEC, FILE*, fopen, (const char* filename, const char* mode), xme_hal_fileio_fopen,
    {
        FILE* fileHandle;
        fopen_s(&fileHandle, filename, mode);
        return fileHandle;
    }
);

// _wfopen()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, FILE*, _wfopen, (const wchar_t* filename, const wchar_t* mode));
#endif // #ifdef _MSC_VER

// _fcloseall()
#ifdef _MSC_VER
XME_DEPRECATE_FUNCTION(XME_CORE_MODIFIER_DECLSPEC, int, _fcloseall, (void));
#endif // #ifdef _MSC_VER

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DEPRECATED_ARCH_H
