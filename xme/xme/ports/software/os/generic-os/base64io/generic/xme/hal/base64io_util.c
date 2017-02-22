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
 * $Id: base64io_util.c 4474 2013-08-01 10:48:14Z geisinger $
 */

/**
 * \file
 *         BASE64 encoded file input/output utility functions.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/base64io.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

/**
 * \brief encodes a given string to string_b64 byte stream.
 *
 * \param str Input String in ASCII.
 * \param str_b64 Address of Pointer where address to the encoded string will be stored.
 * 
 * \retval the size of the encoded string
 */
size_t
xme_hal_base64io_encode(char* str, string_b64** str_b64)
{
    string_b64* temp = *str_b64;
    size_t input_len;
    size_t output_len;
    size_t output_len_bits;
    uint32_t i=0;
    char c;
    char c_b64;
    size_t pad_length = 0;

    input_len = strlen(str);

    // Every character is represented in 6 bits in base64.
    output_len_bits = input_len * 6;

    // padding with 0x3f(63)
    // The string length in bits must be a multiple of 8 and 6.
    switch(output_len_bits % 8)
    {
        case 0: pad_length = 0;
            break;
        case 2: output_len_bits += 6;
            pad_length=1;
            break;
        case 4: output_len_bits += 12;
            pad_length=2;
            break;
        case 6: output_len_bits += 18;
            pad_length=3;
            break;
        default: break;
    }

    // one extra byte for end byte '\0'
    output_len = (output_len_bits/8) + 1;

    xme_hal_mem_set(*str_b64, 0, output_len);

    // converting each byte in input string to base64
    for(c = str[0]; str[i] != '\0'; c = str[++i])
    {
        if(c>=65 && c<=90)
            c_b64 = c - 65;
        else if(c>=97 && c<=122)
            c_b64 = c - 71; // (-91+26)
        else if(c>=48 && c<=57)
            c_b64 = c + 4; // (-48+52)
        else if(c == ' ' || c == '_')
            c_b64 = 62; // We use 62 for space
        else
            return -1;

        switch(i % 4)
        {
            case 0: 
                temp[i*3/4] = (string_b64) (c_b64 << 2);
                break;
            case 1: 
                temp[i*3/4] += (string_b64) (c_b64 >> 4);
                temp[i*3/4 + 1] = (string_b64) (c_b64 << 4);
                break;
            case 2: 
                temp[i*3/4] += (string_b64) (c_b64 >> 2);
                temp[i*3/4 + 1] = (string_b64) (c_b64 << 6);
                break;
            case 3: 
                temp[i*3/4] += (string_b64) c_b64;
                break;
            default : break;
        }
    }

    // padding
    switch(pad_length)
    {
        case 0: break;
        case 1: temp[output_len-2] = temp[output_len-2] + 63;
            break;
        case 2: temp[output_len-3] = temp[output_len-3] + 15;
            temp[output_len-2] = temp[output_len-2] + 255;
            break;
        case 3: temp[output_len-4] = temp[output_len-4] + 3;
            temp[output_len-3] = temp[output_len-3] + 255;
            temp[output_len-2] = temp[output_len-2] + 255;
            break;
        default: break;
    }

    // end byte character
    temp[output_len-1] = '\0';

    return output_len;
}

/**
 * \brief Decodes a Base64 string
 *
 * \param str_b64 Pointer to the Base64 encoded string.
 * \param ret_str Pointer to the buffer where the decoded string must be stored.
 *
 * \retval the size of decoded string
 */
size_t
xme_hal_base64io_decode(string_b64* str_b64, char** ret_str)
{
    size_t input_len;
    size_t output_len;
    char c;
    char c_b64;
    char *temp = *ret_str;
    uint32_t i;
    uint32_t j;

    input_len = 0;
    while(str_b64[input_len]!='\0')
        input_len++;

    // Length of decoded string will be maximum 4/3 times length of encoded string.
    // Actual Length can be less due to alignment.
    output_len=strlen_after_decoding(input_len);

    for(i=0, j=0;i<output_len; i++)
    {
        j=i*3/4;

        switch(i%4)
        {
            case 0: c_b64 = str_b64[j] >> 2;
                    break;
            case 1: c_b64 = (str_b64[j] % 4) << 4;
                    c_b64 += str_b64[j+1] >> 4;
                    break;
            case 2: c_b64 = (str_b64[j] % 16) << 2;
                    c_b64 += (str_b64[j+1]) >> 6;
                    break;
            case 3: c_b64 = (str_b64[j] % 64);
                    break;
            default: break;
        }

        if(c_b64 <= 25)
            c = c_b64 + 65;
        else if(c_b64 <= 51)
            c = c_b64 + 71;
        else if(c_b64 <=61)
            c = c_b64 - 4;
        else if(c_b64 == 62)
            c = ' ';
        else if(c_b64 == 63)
            break;
            //c = ' '; // Special Case. Occurs when actual length of decoded string is 3 (mod 4)
        else
            return -1;

        temp[i] = c;
    }
    temp[i] = '\0';

    return output_len;
}
