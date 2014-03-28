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
 * $Id: deprecated_arch.h 2304 2013-01-30 10:32:43Z geisinger $
 */

/**
 * \file
 *         Flagging of various native functions as deprecated
 *         (generic OS-based implementation).
 */

#ifndef XME_CORE_DEPRECATED_ARCH_H
#define XME_CORE_DEPRECATED_ARCH_H

#ifndef XME_CORE_DEPRECATED_H
	#error This architecture-specific header file should not be included directly. Include the generic header file (usually without "_arch" suffix) instead.
#endif // #ifndef XME_CORE_DEPRECATED_H

/******************************************************************************/
/***   Forward declarations                                                 ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

//----------------------------------------------------------------------------//
//     String related                                                         //
//----------------------------------------------------------------------------//

// vsnprintf()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
	XME_CORE_MODIFIER_DECLSPEC, int, vsnprintf, (char* buffer, size_t count, const char* format, va_list args), xme_hal_safeString_vsnprintf,
	{
		return vsnprintf(buffer, count, format, args);
	}
);

// strncpy()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
	XME_CORE_MODIFIER_DECLSPEC, char*, strncpy, (char* destination, const char* source, size_t num), xme_hal_safeString_strncpy,
	{
		return strncpy(destination, source, num);
	}
);

// strncat()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
	XME_CORE_MODIFIER_DECLSPEC, char*, strncat, (char* destination, const char* source, size_t num), xme_hal_safeString_strncat,
	{
		return strncat(destination, source, num);
	}
);

//----------------------------------------------------------------------------//
//     File input/output related                                              //
//----------------------------------------------------------------------------//

// fopen()
XME_DEPRECATE_REPLACED_FUNCTION_WITH_FALLBACK
(
	XME_CORE_MODIFIER_DECLSPEC, FILE*, fopen, (const char* filename, const char* mode), xme_hal_fileio_fopen,
	{
		return fopen(filename, mode);
	}
);

XME_EXTERN_C_END

#endif // #ifndef XME_CORE_DEPRECATED_ARCH_H
