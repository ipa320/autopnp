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
 * $Id: base64io.h 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         BASE64 encoded file input/output abstraction.
 */

#ifndef XME_HAL_BASE64IO_H
#define XME_HAL_BASE64IO_H

/**
 * \defgroup hal_base64io BASE64 encoded file I/O.
 *
 * @{
 *
 * \brief  BASE64 encoded file I/O for reading and writing of files.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/fileio.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

 /**
 * \brief Returns the size of the encoded string
 * 
 * \param ascii_input_strlen Length of the ASCII input string to be encoded
 * 
 * \retval size of the Base64 encoded string
 */
size_t
strlen_after_encoding
(
    size_t ascii_input_strlen
);

/**
 * \brief Returns the size of the decoded ASCII string
 * 
 * \param base64_input_strlen Length of the Base64 input string to be encoded
 * 
 * \retval size of the decoded ASCII string
 */
size_t
strlen_after_decoding
(
    size_t base64_input_strlen
);

/**
 * \brief Encodes an input string, to base64 encoded stream, and writes to a file
 *
 * \param fileHandle the file handle
 * \param message formatted string that must be encoded
 *
 * \retval the size of the encoded string written to file
 */
uint32_t
FORMAT_FUNCTION(2, 3)
xme_hal_base64io_fprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* message,
    ...
);

/**
 * \brief Reads encoded Base64 from binary file, decodes it to ASCII, and returns
 *
 * \param fileHandle the file handle
 * \param str Pointer to the buffer where the decoded ASCII string should be stored
 * \param ascii_strlen length of the decoded string
 *
 * \retval the size of the decoded string
 */
size_t 
xme_hal_base64io_read_file
(
    xme_hal_fileio_fileHandle_t fileHandle,
    char **str,
    size_t ascii_strlen
);


XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/base64io_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_BASE64IO_H

