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
 */

/**
 * \file
 *         File input/output utility functions.
 */

#ifndef XME_HAL_FILEIO_UTIL_H
#define XME_HAL_FILEIO_UTIL_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdarg.h>
#include <stdio.h>

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

INLINE
xme_status_t
xme_hal_fileio_fclose
(
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    return (0 == xme_fallback_fclose(fileHandle)) ? XME_STATUS_SUCCESS : XME_STATUS_INVALID_HANDLE;
}

INLINE
xme_status_t
xme_hal_fileio_ftell
(
    xme_hal_fileio_fileHandle_t fileHandle,
    uint64_t* position
)
{
    long int result;

    XME_CHECK(NULL != position, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != fileHandle, XME_STATUS_INVALID_HANDLE);

    result = (uint64_t)xme_fallback_ftell(fileHandle);

    if (-1L == result)
    {
        return XME_STATUS_INVALID_HANDLE;
    }
    else
    {
        *position = result;
        return XME_STATUS_SUCCESS;
    }
}

INLINE
xme_status_t
xme_hal_fileio_fseek
(
    xme_hal_fileio_fileHandle_t fileHandle,
    int64_t offset,
    xme_hal_fileio_seekOrigin_t origin
)
{
    XME_CHECK(NULL != fileHandle, XME_STATUS_INVALID_HANDLE);

    return (0 == xme_fallback_fseek(fileHandle, (long)offset, origin)) ? XME_STATUS_SUCCESS : XME_STATUS_INVALID_HANDLE;
}

INLINE
uint32_t
xme_hal_fileio_fread
(
    void* buffer,
    uint32_t size,
    uint32_t count,
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    return (uint32_t)xme_fallback_fread(buffer, size, count, fileHandle);
}

INLINE
uint32_t
xme_hal_fileio_fwrite
(
    const void* buffer,
    uint32_t size,
    uint32_t count,
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    return (uint32_t)xme_fallback_fwrite(buffer, size, count, fileHandle);
}

INLINE
int16_t
xme_hal_fileio_fgetc
(
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    int result = xme_fallback_fgetc(fileHandle);
    return (EOF == result) ? (int16_t)XME_HAL_FILEIO_EOF : (int16_t)result;
}

INLINE
int16_t
xme_hal_fileio_fputc
(
    int16_t character,
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    int result = xme_fallback_fputc(character, fileHandle);
    return (EOF == result) ? (int16_t)XME_HAL_FILEIO_EOF : (int16_t)result;
}

INLINE
char*
xme_hal_fileio_fgets
(
    char* str,
    int size,
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    // TODO: Implement this using fgets(), but ensuring
    //       str[0] to be null even in error case!

    int i = 0;

    for (i = 0; i < size-1; i++)
    {
        str[i] = (char) xme_hal_fileio_fgetc(fileHandle);
        if(str[i] == EOF && i == 0)
            return NULL;
        if((str[i] == '\n' || str[i] == EOF) && i != 0)
        {  
            str[i] = '\0';
            break;
        }
    }

    // invariably do this, to avoid one if branching
    // TODO: Shouldn't this be done at i'th (or i+1'th?) position instead?
    str[size-1]='\0';

    return str;
}

INLINE
int16_t
xme_hal_fioid_fputs
(
    const char* string,
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    int result = xme_fallback_fputs(string, fileHandle);
    return (EOF == result) ? (int16_t)XME_HAL_FILEIO_EOF : (int16_t)result;
}

INLINE
int32_t
xme_hal_fileio_fprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* format,
    ...
)
{
    va_list args;
    int32_t result;

    va_start(args, format);
    result = (int32_t)xme_fallback_vfprintf(fileHandle, format, args);
    va_end(args);

    return result;
}

INLINE
int32_t
xme_hal_fileio_vfprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* format,
    va_list args
)
{
    return (int32_t)xme_fallback_vfprintf(fileHandle, format, args);
}

// vfscanf is not part of the C standard, hence currently this one is not supported
#if 0
INLINE
xme_status_t
xme_hal_fileio_fscanf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    int16_t* itemsScanned,
    const char* format,
    ...
)
{
    va_list args;
    int16_t result;

    va_start(args, format);
    result = (int16_t)vfscanf(fileHandle, format, args);
    va_end(args);

    if (itemsScanned)
    {
        *itemsScanned = result;
    }

    if (feof(fileHandle) || ferror(fileHandle))
    {
        if (itemsScanned && (EOF == *itemsScanned))
        {
            *itemsScanned = XME_HAL_FILEIO_EOF;
        }
        return XME_CORE_STATUS_INTERNAL_ERROR;
    }
    else
    {
        return XME_CORE_STATUS_SUCCESS;
    }
}
#endif // #if 0

INLINE
FILE*
xme_hal_fileio_toStream
(
    xme_hal_fileio_fileHandle_t fileHandle
)
{
    // Ensure that XME_HAL_FILEIO_INVALID_FILE_HANDLE satisfies the specification
    XME_ASSERT_RVAL(NULL == XME_HAL_FILEIO_INVALID_FILE_HANDLE, NULL);

    return fileHandle;
};

INLINE
xme_hal_fileio_fileHandle_t
xme_hal_fileio_fromStream
(
    FILE* stream
)
{
    // Ensure that XME_HAL_FILEIO_INVALID_FILE_HANDLE satisfies the specification
    XME_ASSERT_RVAL(NULL == XME_HAL_FILEIO_INVALID_FILE_HANDLE, NULL);

    return stream;
}

XME_EXTERN_C_END

#endif //#ifndef XME_HAL_FILEIO_UTIL_H
