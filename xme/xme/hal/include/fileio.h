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
 * $Id: fileio.h 7716 2014-03-07 16:13:55Z geisinger $
 */

/**
 * \file
 *         File input/output abstraction.
 */

#ifndef XME_HAL_FILEIO_H
#define XME_HAL_FILEIO_H

/**
 * \defgroup hal_fileio File I/O.
 *
 * @{
 *
 * \brief  File I/O for reading and writing of files.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdio.h>
#include <stdint.h>

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \typedef xme_hal_fileio_fileHandle_t
 *
 * \brief  File stream handle.
 */
typedef FILE* xme_hal_fileio_fileHandle_t;

/**
 * \enum   xme_hal_fileio_mode_t
 *
 * \brief  Defines the possible values for specifying the access mode to a file.
 *
 * Used in ::xme_hal_fileio_fopen().
 */
typedef enum
{
    XME_HAL_FILEIO_MODE_INVALID_MODE = 0,   ///< Invalid mode.

    XME_HAL_FILEIO_MODE_READONLY      = ('r' << 24),                 ///< Open file for input operations. The file must exist.
    XME_HAL_FILEIO_MODE_WRITEONLY     = ('w' << 24),                 ///< Create an empty file for output operations. If a file with the same name already exists, its contents are discarded and the file is treated as a new empty file.
    XME_HAL_FILEIO_MODE_APPEND        = ('a' << 24),                 ///< Open file for output at the end of a file. Output operations always write data at the end of the file, expanding it. Repositioning operations (e.g., xme_hal_fileio_fseek()) are ignored. The file is created if it does not exist.
    XME_HAL_FILEIO_MODE_READ_UPDATE   = ('r' << 24) + ('+' << 16),   ///< Open a file for update (both for input and output). The file must exist.
    XME_HAL_FILEIO_MODE_WRITE_UPDATE  = ('w' << 24) + ('+' << 16),   ///< Create an empty file and open it for update (both for input and output). If a file with the same name already exists its contents are discarded and the file is treated as a new empty file.
    XME_HAL_FILEIO_MODE_APPEND_UPDATE = ('a' << 24) + ('+' << 16),   ///< Open a file for update (both for input and output) with all output operations writing data at the end of the file. Repositioning operations (e.g., xme_hal_fileio_fseek()) affects the next input operations, but output operations move the position back to the end of file. The file is created if it does not exist.

    XME_HAL_FILEIO_MODE_READONLY_BINARY      = ('r' << 24) + ('b' << 16),                ///< The same as ::XME_HAL_FILEIO_MODE_READONLY, but for binary files.
    XME_HAL_FILEIO_MODE_WRITEONLY_BINARY     = ('w' << 24) + ('b' << 16),                ///< The same as ::XME_HAL_FILEIO_MODE_WRITEONLY, but for binary files.
    XME_HAL_FILEIO_MODE_APPEND_BINARY        = ('a' << 24) + ('b' << 16),                ///< The same as ::XME_HAL_FILEIO_MODE_APPEND, but for binary files.
    XME_HAL_FILEIO_MODE_READ_UPDATE_BINARY   = ('r' << 24) + ('+' << 16) + ('b' << 8),   ///< The same as ::XME_HAL_FILEIO_MODE_READ_UPDATE, but for binary files.
    XME_HAL_FILEIO_MODE_WRITE_UPDATE_BINARY  = ('w' << 24) + ('+' << 16) + ('b' << 8),   ///< The same as ::XME_HAL_FILEIO_MODE_WRITE_UPDATE, but for binary files.
    XME_HAL_FILEIO_MODE_APPEND_UPDATE_BINARY = ('a' << 24) + ('+' << 16) + ('b' << 8)    ///< The same as ::XME_HAL_FILEIO_MODE_APPEND_UPDATE, but for binary files.
}
xme_hal_fileio_mode_t;

/**
 * \enum   xme_hal_fileio_seekOrigin_t
 *
 * \brief  Defines the origin for seeking a position in a file stream.
 *
 * Used in ::xme_hal_fileio_fseek().
 */
typedef enum
{
    XME_HAL_FILEIO_SEEKORIGIN_SET = SEEK_SET, ///< Seek relative to the beginning of the file stream.
    XME_HAL_FILEIO_SEEKORIGIN_CUR = SEEK_CUR, ///< Seek relative to current position of the file stream pointer.
    XME_HAL_FILEIO_SEEKORIGIN_END = SEEK_END  ///< Seek relative to the end of the file stream. This value might not be supported by all implementations.
}
xme_hal_fileio_seekOrigin_t;

/******************************************************************************/
/***   Constants                                                            ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \var  XME_HAL_FILEIO_INVALID_FILE_HANDLE
 *
 * \brief  Invalid file handle constant.
 *
 * Returned by ::xme_hal_fileio_fopen() in case of errors.
 */
const xme_hal_fileio_fileHandle_t XME_HAL_FILEIO_INVALID_FILE_HANDLE = NULL;

/**
 * \var  XME_HAL_FILEIO_EOF
 *
 * \brief  End of file constant.
 *
 * Returned by ::xme_hal_fileio_fgetc() and ::xme_hal_fileio_fputc() in case
 * the end of file has been reached.
 */
const int16_t XME_HAL_FILEIO_EOF = (int16_t) EOF;

XME_EXTERN_C_END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Returns whether a file with the given name exists or not.
 *
 * \param[in] filename Name of the file to check for existence.
  *
 * \return A non-zero value if the file exists and zero otherwise.
 */
int8_t
xme_hal_fileio_fileExists
(
    const char* filename
);

/**
 * \brief Returns whether a directory with the given name exists or not.
 *
 * \param[in] dirname Name of the directory to check for existence.
  *
 * \return A non-zero value if the directory exists and zero otherwise.
 */
int8_t
xme_hal_fileio_directoryExists
(
    const char* dirname
);

/**
 * \brief  Opens a file stream.
 *
 * \param[in] filename Name of the file to be opened.
 * \param[in] mode Mode for opening the file.
 *
 * \return On succes, returns a non-NULL file stream handle.
 *         On error, returns XME_HAL_FILEIO_INVALID_FILE_HANDLE.
 */
xme_hal_fileio_fileHandle_t
xme_hal_fileio_fopen
(
    const char* filename,
    xme_hal_fileio_mode_t mode
);

/**
 * \brief Deletes (removes, unlinks) the file with the given name.
 *
 * On Windows, this function cannot delete a file that is currently open,
 * in which case XME_STATUS_PERMISSION_DENIED is returned.
 *
 * On Linux, even a currently opened file is deleted and kept in memory
 * until all processes using the file close it.
 *
 * This difference of behavior between Windows and Linux is an anamoly
 * that will be fixed subsequently (Issue #3037).
 *
 * \param[in] fileName Address with the name of the file to delete.
 *
 * \retval XME_STATUS_SUCCESS if the file has been successfully deleted.
 * \retval XME_STATUS_NOT_FOUND if the file was not found.
 * \retval XME_STATUS_PERMISSION_DENIED if the file was open or access
 *         to the file was denied due to insufficient permissions.
 * \retval XME_STATUS_INTERNAL_ERROR in case an unspecified error has
 *         occurred.
 */
INLINE
xme_status_t
xme_hal_fileio_deleteFile
(
    const char* fileName
);

/**
 * \brief  Closes an open file stream.
 *
 * \param[in] fileHandle File stream handle to close.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the file stream was successfully closed.
 * \retval XME_CORE_STATUS_INVALID_HANDLE if the given file handle did
 *                                        not specify an open file stream handle.
 */
xme_status_t
xme_hal_fileio_fclose
(
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Gets the current position in a file stream.
 *
 * Returns the current value of the position indicator of the stream.
 * For binary streams, this is the number of bytes from the beginning
 * of the file.
 * For text streams, the numerical value may not be meaningful but can
 * still be used to restore the position to the same position later
 * using xme_hal_fileio_fseek().
 *
 * \param[in] fileHandle File stream handle to return position for.
 * \param[out] position If non-NULL, the memory pointed to by this argument is
 *             set the current value of the position indicator.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the current value of the position
 *                                 indicator is returned in the position parameter.
 * \retval XME_CORE_STATUS_INVALID_HANDLE on failure.
 * \retval XME_CORE_STATUS_INVALID_PARAMETER if position was NULL.
 */
xme_status_t
xme_hal_fileio_ftell
(
    xme_hal_fileio_fileHandle_t fileHandle,
    uint64_t* position
);

/**
 * \brief  Repositions the file stream position indicator.
 *
 * Sets the position indicator associated with the stream to a new position.
 * For streams open in binary mode, the new position is defined by adding
 * offset to a reference position specified by origin.
 * For streams open in text mode, offset shall either be zero or a value
 * returned by a previous call to xme_hal_fileio_ftell(), and origin shall
 * be XME_HAL_FILEIO_SEEKORIGIN_SET.
 *
 * On streams open for update (read+write), a call to xme_hal_fileio_fseek()
 * allows to switch between reading and writing.
 *
 * \param[in] fileHandle File stream handle to set position for.
 * \param[in] offset For binary files, the number of bytes to offset from origin.
 *            For text files either zero, or a value returned by
 *            xme_hal_fileio_ftell().
 * \param[in] origin One of XME_HAL_FILEIO_SEEKORIGIN_SET,
 *            XME_HAL_FILEIO_SEEKORIGIN_CUR or XME_HAL_FILEIO_SEEKORIGIN_END.
 *
 * \retval XME_CORE_STATUS_SUCCESS if the file stream position indicator
 *                                 has been successfully set.
 * \retval XME_CORE_STATUS_INVALID_HANDLE on failure.
 */
xme_status_t
xme_hal_fileio_fseek
(
    xme_hal_fileio_fileHandle_t fileHandle,
    int64_t offset,
    xme_hal_fileio_seekOrigin_t origin
);

/**
 * \brief  Reads from a file stream.
 *
 * \param[in,out] buffer Pointer to a block of memory with a size of at least
 *                (size*count) bytes.
 * \param[in] size Size in bytes, of each element to be read.
 * \param[in] count Number of elements (each one of the given size) to be read
 *            from the file.
 * \param[in] fileHandle File stream handle that specifies the input stream.
 *
 * \return Returns the total number of elements (not necessarily the number of
 *         characters) successfully read.
 */
uint32_t
xme_hal_fileio_fread
(
    void* buffer,
    uint32_t size,
    uint32_t count,
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Writes to a file stream.
 *
 * \param[in] buffer Pointer to a block of memory with a size of at least
 *            (size*count) bytes.
 * \param[in] size Size in bytes, of each element to be written.
 * \param[in] count Number of elements (each one of the given size) to be written to the file.
 * \param[in] fileHandle File stream handle that specifies the output stream.
 *
 * \return Returns the total number of elements (i.e., not necessarily the number of
 *         characters!) successfully written.
 */
uint32_t
xme_hal_fileio_fwrite
(
    const void* buffer,
    uint32_t size,
    uint32_t count,
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Reads one character from the given file stream.
 *
 * \param[in] fileHandle File stream handle that specifies the input stream.
 *
 * \return Returns the character read from the file stream on success or
 *         XME_HAL_FILEIO_EOF if the end-of-file has been reached.
 */
int16_t
xme_hal_fileio_fgetc
(
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Writes one character to the given file stream.
 *
 * \param[in] character Character to be written to the file stream.
 * \param[in] fileHandle File stream handle that specifies the output stream.
 *
 * \return Returns the character written on success or XME_HAL_FILEIO_EOF
 *         on error.
 */
int16_t
xme_hal_fileio_fputc
(
    int16_t character,
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Reads characters from the given stream and stores them as a C string
 *         into str until (num-1) characters have been read or either a newline
 *         character or the end-of-file is reaches, whichever happens first.
 *
 * A newline character makes xme_hal_fileio_fgets() stop reading, but it is
 * considered a valid character by the function and included in the string
 * copied to str.
 *
 * A terminating null character is automatically appended after the characters
 * copied to str.
 * 
 * \param[in,out] str Address of a buffer of at least size bytes to store the
 *                string read from the file stream.
 * \param[in] size Maximum number of characters to be copied into str
 *            (including the terminating null-character).
 * \param[in] fileHandle File stream handle that specifies the input stream.
 *
 * \return On success, the function returns str. On end-of-file (i.e., no
 *         characters read) or read error, returns NULL.
 */
char*
xme_hal_fileio_fgets
(
    char* str,
    int size,
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Writes a string to a file stream.
 *
 * \details The function begins copying from the address specified until it
 *          reaches the terminating null character ('\\0'). This terminating
 *          null-character is not copied to the stream.
 *
 * \param[in] string String with the content to be written to stream.
 * \param[in] fileHandle File stream handle that specifies the output stream.
 *
 * \return Returns  a non-negative value on success or XME_HAL_FILEIO_EOF
 *         on error.
 */
int16_t
xme_hal_fioid_fputs
(
    const char* string,
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief  Writes formatted data to a file stream.
 *
 *         Writes the string pointed by format to the file stream, replacing any
 *         format specifiers (subsequences beginning with %) by the additional
 *         arguments.
 *
 * \param[in] fileHandle File stream handle that specifies the output stream.
 * \param[in] format String that contains the text to be written to the stream.
 *            See the documentation of the C standard function fprintf().
 * \param[in] ... Depending on the format string, the function may expect a sequence
 *            of additional arguments, each containing a value to be used to replace
 *            a format specifier in the format string.
 *
 * \return Returns the total number of characters written.
 *         If a writing error occurs, a negative number is returned.
 */
int32_t
FORMAT_FUNCTION(2, 3)
xme_hal_fileio_fprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* format,
    ...
);

/**
 * \brief  Writes formatted data from a variable argument list to a file stream.
 *
 *         Writes the string pointed by format to the file stream, replacing any
 *         format specifiers (subsequences beginning with %) by the elements of
 *         the given variable argument list.
 *
 * \param[in] fileHandle File stream handle that specifies the output stream.
 * \param[in] format String that contains the text to be written to the stream.
 *            See the documentation of the C standard function vfprintf().
 * \param[in] args Variable argument list with additional parameters.
 *
 * \return Returns the total number of characters written.
 *         If a writing error occurs, a negative number is returned.
 */
int32_t
FORMAT_FUNCTION(2, 0)
xme_hal_fileio_vfprintf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    const char* format,
    va_list args
);

// vfscanf is not part of the C standard, hence currently this one is not supported
#if 0
/**
 * \brief  Reads formatted data from a file stream.
 *
 *         Reads data from the stream and stores them according to the format
 *         parameter into the locations pointed by the additional arguments.
 *         The additional arguments should point to already allocated objects
 *         of the type specified by their corresponding format specifier within
 *         the format string.
 *
 * \param[in] fileHandle File stream handle that specifies the input stream.
 * \param[in,out] itemsScanned If non-NULL, the memory pointed to by this argument is
 *                set the number of items of the argument list successfully filled.
 *                This count can match the expected number of items or be less (even
 *                zero) due to a matching failure, a reading error, or the reach of
 *                the end-of-file.
 *                If a reading error happens or the end-of-file is reached before any
 *                data could be successfully read, XME_HAL_FILEIO_EOF is returned in
 *                itemsScanned.
 * \param[in] format String that contains a sequence of characters that control
 *            how characters extracted from the stream are treated.
 *            See the documentation of the C standard function fscanf().
 * \param[in] ... Depending on the format string, the function may expect a
 *            sequence of additional arguments, each containing a pointer to
 *            allocated storage where the interpretation of the extracted
 *            characters is stored with the appropriate type.
 *
 * \retval XME_CORE_STATUS_SUCCESS if all items have been successfully scanned.
 * \retval XME_CORE_STATUS_INTERNAL_ERROR if at least one item could not be
 *         successfully scanned.
 */
xme_status_t
SCAN_FUNCTION(3, 4)
xme_hal_fileio_fscanf
(
    xme_hal_fileio_fileHandle_t fileHandle,
    int16_t* itemsScanned,
    const char* format,
    ...
);
#endif // #if 0

/**
 * \brief Returns a file stream object associated to the given file handle.
 *
 * \param[in] fileHandle File stream handle.
 *
 * \return Returns the non-null file stream object corresponding to the given
 *         file handle if the file handle is not
 *         XME_HAL_FILEIO_INVALID_FILE_HANDLE valid and NULL otherwise.
 */
FILE*
xme_hal_fileio_toStream
(
    xme_hal_fileio_fileHandle_t fileHandle
);

/**
 * \brief Returns a file handle for the given file stream.
 *
 * \param[in] stream File stream.
 *
 * \return Returns a file handle other than XME_HAL_FILEIO_INVALID_FILE_HANDLE
 *         if stream was not NULL and XME_HAL_FILEIO_INVALID_FILE_HANDLE
 *         otherwise.
 */
xme_hal_fileio_fileHandle_t
xme_hal_fileio_fromStream
(
    FILE* stream
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/fileio_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_FILEIO_H
