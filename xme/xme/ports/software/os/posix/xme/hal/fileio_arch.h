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
 *         File input/output abstraction (POSIX based implementation).
 */

#ifndef XME_HAL_FILEIO_ARCH_H
#define XME_HAL_FILEIO_ARCH_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/fileio_util.h"

#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

INLINE
int8_t
xme_hal_fileio_fileExists
(
    const char* filename
)
{
    struct stat st;
    XME_CHECK(-1 != stat(filename, &st), 0);
    return (S_IFDIR != (st.st_mode & S_IFMT));
}

INLINE
int8_t
xme_hal_fileio_directoryExists
(
    const char* dirname
)
{
    struct stat st;
    XME_CHECK(-1 != stat(dirname, &st), 0);
    return (S_IFDIR == (st.st_mode & S_IFMT));
}

INLINE
xme_hal_fileio_fileHandle_t
xme_hal_fileio_fopen
(
    const char* filename,
    xme_hal_fileio_mode_t mode
)
{
    char m[5];

    m[0] = mode >> 24;
    m[1] = mode >> 16;
    m[2] = mode >> 8;
    m[3] = mode;
    m[4] = 0;

    return xme_fallback_fopen(filename, m);
}

/**
 * \brief Deletes a file. In Windows, this function does not delete a file that is currently
 *        in use by some other process. In linux, an open file is also deleted, and kept in
 *        memory until all processes using the file, close it. This difference of behavior is
 *		  an anamoly that will be fixed. Issue #3037
 *		  
 * \param fileName pointer to the filename
 *
 * \retval XME_STATUS_SUCCESS if the file was deleted
 *		   XME_STATUS_NOT_FOUND if the file was not found
 *		   XME_STATUS_PERMISSION_DENIED if the file was open, or access was denied to the file
 *		   XME_STATUS_INTERNAL_ERROR in case of some internal error
 */
INLINE
xme_status_t
xme_hal_fileio_deleteFile
(
	const char* fileName
)
{
	int ret;
	ret = unlink(fileName);

	if(ret == 0)
		return XME_STATUS_SUCCESS;
	else
	{
		if(errno == EFAULT || errno == EPERM || errno == EACCES || errno == EROFS)
			return XME_STATUS_PERMISSION_DENIED;
		else if(errno == EISDIR || errno == ENOENT || errno == ENOTDIR)
			return XME_STATUS_NOT_FOUND;
		else
			return XME_STATUS_INTERNAL_ERROR;
	}
}

XME_EXTERN_C_END

#endif //#ifndef XME_HAL_FILEIO_ARCH_H
