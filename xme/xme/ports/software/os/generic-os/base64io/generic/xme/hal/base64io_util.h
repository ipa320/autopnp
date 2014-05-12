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
 *         BASE64 encoded file input/output utility functions.
 */

#ifndef XME_HAL_BASE64IO_UTIL_H
#define XME_HAL_BASE64IO_UTIL_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdarg.h>
#include "xme/hal/include/mem.h"
#include "xme/hal/include/fileio.h"
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
// TODO: Prefix!
#define MAX_INPUT_STRING_LENGTH 256

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
// TODO: Prefix!
typedef uint8_t string_b64;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief encodes a given string to string_b64 byte stream.
 *
 * \param str Input String in ASCII.
 * \param str_b64 Address of Pointer where address to the encoded string will be stored.
 * 
 * \retval returns the size of the encoded string
 */
size_t
xme_hal_base64io_encode
(
    char* str,
    string_b64** str_b64
);

/**
 * \brief Decodes a Base64 string
 *
 * \param str_b64 Pointer to the Base64 encoded string.
 * \param ret_str Pointer to the buffer where the decoded string must be stored.
 *
 * \retval returns the size of decoded string
 */
size_t
xme_hal_base64io_decode
(
    string_b64* str_b64,
    char** ret_str
);

/**
 * \brief Returns the size of the encoded string
 * 
 * \param ascii_input_strlen Length of the ASCII input string to be encoded
 * 
 * \retval size of the Base64 encoded string
 */
// TODO: Prefix!
INLINE
size_t
strlen_after_encoding(size_t ascii_input_strlen)
{
    size_t output_len_bits;
    size_t output_len;
    // Every character is represented in 6 bits in base64.
    output_len_bits = ascii_input_strlen * 6;

    // padding with 0x3f(63)
    // The string length in bits must be a multiple of 8 and 6.
    switch(output_len_bits % 8)
    {
        case 0:
            break;
        case 2: output_len_bits += 6;
            break;
        case 4: output_len_bits += 12;
            break;
        case 6: output_len_bits += 18;
            break;
        default: break;
    }

    // one extra byte for end byte '\0'
    output_len = (output_len_bits/8) + 1;

    return output_len;
}

/**
 * \brief Returns the size of the decoded ASCII string
 * 
 * \param base64_input_strlen Length of the Base64 input string to be encoded
 * 
 * \retval size of the decoded ASCII string
 */
// TODO: Prefix!
INLINE
size_t
strlen_after_decoding(size_t base64_input_strlen)
{
    return (base64_input_strlen*4/3) + 1;
}

/**
 * \brief Encodes an input string, to base64 encoded stream, and writes to a file
 *
 * \param fileHandle the file handle
 * \param message formatted string that must be encoded
 *
 * \retval the size of the encoded string written to file
 */
INLINE
uint32_t
FORMAT_FUNCTION(2, 3)
xme_hal_base64io_fprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* message,
    ...
)
{
    va_list args;
    char str[MAX_INPUT_STRING_LENGTH];
    string_b64* str_b64;
    size_t str_b64_len=0;
    uint32_t ret;
    
    va_start(args, message);
    xme_hal_safeString_vsnprintf(str, MAX_INPUT_STRING_LENGTH, message, args);
    //perror(str);
    va_end (args);

    str_b64_len = strlen_after_encoding(strlen(str));
    str_b64 = (string_b64 *) xme_hal_mem_alloc(str_b64_len);

    str_b64_len = xme_hal_base64io_encode(str, &str_b64);

    // we write only str_b64_len-1 bytes to the string to remove the end character '\0'
    ret=xme_hal_fileio_fwrite(str_b64, sizeof(uint8_t), str_b64_len-1, fileHandle);

    xme_hal_mem_free(str_b64);

    return ret;
}

/**
 * \brief Reads encoded Base64 from binary file, decodes it to ASCII, and returns
 *
 * \param fileHandle the file handle
 * \param str Pointer to the buffer where the decoded ASCII string should be stored
 * \param ascii_strlen length of the decoded string
 *
 * \retval the size of the decoded string
 */
INLINE
size_t 
xme_hal_base64io_read_file
(
    xme_hal_fileio_fileHandle_t fileHandle,
    char **str,
    size_t ascii_strlen
)
{
    string_b64* str_b64;
    size_t num_bytes_read = 0;
    size_t encoded_strlen;
    size_t nbytes_decoded;

    encoded_strlen = strlen_after_encoding(ascii_strlen);
    str_b64 = (string_b64*)xme_hal_mem_alloc(sizeof(uint8_t)*encoded_strlen);

    num_bytes_read = xme_hal_fileio_fread(str_b64, sizeof(uint8_t), encoded_strlen, fileHandle);
    XME_ASSERT(0 != num_bytes_read);

    str_b64[num_bytes_read] = '\0';
    nbytes_decoded = xme_hal_base64io_decode(str_b64, str);

    xme_hal_mem_free(str_b64);

    return nbytes_decoded;
}

XME_EXTERN_C_END

#endif //#ifndef XME_HAL_BASE64IO_UTIL_H
