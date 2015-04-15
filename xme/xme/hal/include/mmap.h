/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: mmap.h 6626 2014-02-05 14:36:43Z geisinger $
 */

/**
 * \file
 * \brief Memory mapping abstraction.
 */

#ifndef XME_HAL_MMAP_H
#define XME_HAL_MMAP_H

/**
 * \defgroup hal_mmap Memory Mapping Abstraction
 * @{
 *
 * \brief Memory mapping abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Creates a new private mapping into the virtual address space of the
 *        calling process.
 *
 * \param[in] addr Starting address for the new mapping.
 *            This parameter may be NULL to let the operating system choose
 *            the address at which to create the mapping; this is the most
 *            portable method of creating a new mapping. If addr is not NULL,
 *            then the operating system uses it as a hint about where to place
 *            the mapping (except if the XME_HAL_MMAP_FIXED flag is specified).
 *            The resulting address of the memory is returned in outAddr.
 *            On Linux, the mapping is usually created at a nearby page
 *            boundary. The system page size may be obtained with the
 *            xme_hal_mmap_getSystemPageSize() function.
 * \param[in] length Length of the mapping.
 * \param[in] protection Desired memory protection of the mapping. Should be
 *            either XME_HAL_MMAP_PROTECTION_NONE or the bitwise OR of one or
 *            more of XME_HAL_MMAP_PROTECTION_READ, XME_HAL_MMAP_PROTECTION_WRITE,
 *            and XME_HAL_MMAP_PROTECTION_EXECUTE.
 * \param[in] flags Bitwise OR of zero or more of the following flags:
 *             - XME_HAL_MMAP_FIXED: Do not interpret addr as a hint, but
 *               place the mapping at exactly that address. addr must be a
 *               multiple of the page size. If the memory region specified by
 *               addr and length overlaps pages of any existing mapping(s),
 *               then the overlapped part of the existing mapping(s) will be
 *               discarded. If the specified address cannot be used, the call
 *               will fail. The use of this option is discouraged.
 * \param[in,out] outAddr Address of a variable to initialize with a pointer
 *                to the resulting address of the memory mapping.
 *
 * \retval XME_STATUS_SUCCESS if the memory mapping has been successfully
 *         created.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the following was the case:
 *          - outAddr was NULL.
 *          - addr, length or offset were invalid (e.g., too large or not
 *            properly aligned on a page boundary).
 *          - flags were invalid.
 * \retval XME_STATUS_PERMISSION_DENIED if too much memory has been locked
 *        (see setrlmit() on Linux).
 * \retval XME_STATUS_OUT_OF_RESOURCES if one of the following was the case:
 *          - No memory was available.
 *          - The process's maximum number of mappings would have been
 *            exceeded.
 *          - The system limit on the total number of open files has been
 *            reached.
 */
xme_status_t
xme_hal_mmap_mapPrivateMemory
(
    void* addr,
    size_t length,
    uint8_t protection,
    uint32_t flags,
    void** const outAddr
);

/**
 * \brief Creates a new private mapping into the virtual address space of the
 *        calling process.
 *
 * \param[in] addr Starting address for the new mapping.
 *            This parameter may be NULL to let the operating system choose
 *            the address at which to create the mapping; this is the most
 *            portable method of creating a new mapping. If addr is not NULL,
 *            then the operating system uses it as a hint about where to place
 *            the mapping (except if the XME_HAL_MMAP_FIXED flag is specified).
 *            The resulting address of the memory is returned in outAddr.
 *            On Linux, the mapping is usually created at a nearby page
 *            boundary. The system page size may be obtained with the
 *            xme_hal_mmap_getSystemPageSize() function.
 * \param[in] length Length of the mapping.
 * \param[in] protection Desired memory protection of the mapping (must not
 *            conflict with the open mode of the file). Should be either
 *            XME_HAL_MMAP_PROTECTION_NONE or the bitwise OR of one or more
 *            of XME_HAL_MMAP_PROTECTION_READ, XME_HAL_MMAP_PROTECTION_WRITE,
 *            and XME_HAL_MMAP_PROTECTION_EXECUTE.
 * \param[in] fileDescriptor If not set to -1, specifies a file descriptor
 *            that has been opened in a mode compatible to the specified
 *            protection that is mapped into the memory space of the calling
 *            process. If set to -1, offset should be zero and flags should
 *            include XME_HAL_MMAP_ANONYMOUS to indicate an anonymous mapping.
 * \param[in] offset Starting offset within the file where to start mapping
 *            from. Must be a multiple of the page size as returned by the
 *            xme_hal_mmap_getSystemPageSize() function.
 * \param[in] flags Bitwise OR of zero or more of the following flags:
 *             - XME_HAL_MMAP_FIXED: Do not interpret addr as a hint, but
 *               place the mapping at exactly that address. addr must be a
 *               multiple of the page size. If the memory region specified by
 *               addr and length overlaps pages of any existing mapping(s),
 *               then the overlapped part of the existing mapping(s) will be
 *               discarded. If the specified address cannot be used, the call
 *               will fail. The use of this option is discouraged.
 * \param[in,out] outAddr Address of a variable to initialize with a pointer
 *                to the resulting address of the memory mapping.
 *
 * \retval XME_STATUS_SUCCESS if the memory mapping has been successfully
 *         created.
 * \retval XME_STATUS_INVALID_PARAMETER if one of the following was the case:
 *          - outAddr was NULL.
 *          - addr, length or offset were invalid (e.g., too large or not
 *            properly aligned on a page boundary).
 *          - fileDescriptor was not a valid file descriptor.
 *          - flags were invalid.
 * \retval XME_STATUS_PERMISSION_DENIED if one of the following was the case:
 *          - fileDescriptor referred to a non-regular file.
 *          - fileDescriptor is not open in read/write mode, but flags
 *            contained XME_HAL_MMAP_PROTECTION_WRITE.
 *          - fileDescriptor is open in append-only mode, but flags contained
 *            XME_HAL_MMAP_PROTECTION_WRITE.
 *          - fileDescriptor corresponds to a file that is mounted no-exec,
 *            but flags contained XME_HAL_MMAP_PROTECTION_EXEC.
 *          - fileDescriptor corresponds to a locked file.
 *          - Too much memory has been locked (see setrlmit() on Linux).
 * \retval XME_STATUS_OUT_OF_RESOURCES if one of the following was the case:
 *          - No memory was available.
 *          - The process's maximum number of mappings would have been
 *            exceeded.
 *          - The system limit on the total number of open files has been
 *            reached.
 * \retval XME_STATUS_UNSUPPORTED if the underlying file system of the
 *         specified file does not support memory mapping.
 */
xme_status_t
xme_hal_mmap_mapSharedMemory
(
    void* addr,
    size_t length,
    uint8_t protection,
    int fileDescriptor,
    off_t offset,
    uint32_t flags,
    void** const outAddr
);

/**
 * \brief Flushes changes made to the in-core copy of a file that was mapped
 *        into memory back to disk.
 *
 * \details This function updates the part of the file that corresponds to
 *          the memory area starting at addr and having length length.
 *          When this function is not called, there is no guarantee that any
 *          changes are written back before xme_hal_mmap_unmapMemory() is
 *          called on the mapped memory.
 *
 * \param[in] addr Starting address of memory are to flush.
 *            Must be a multiple of the system page size.
 * \param[in] length Length of memory area to flush.
 * \param[in] flags Bitwise OR of either XME_HAL_MAP_SYNC_ASYNC or
 *            XME_HAL_MAP_SYNC_SYNC and optionally XME_HAL_MAP_SYNC_INVALIDATE.
 *
 * \retval XME_STATUS_SUCCESS if the memory area was successfully flushed.
 * \retval XME_STATUS_INVALID_PARAMETER if addr was not a multiple of the
 *         system page size, invalid flags have been specified or both
 *         XME_HAL_MAP_SYNC_ASYNC and XME_HAL_MAP_SYNC_SYNC have been
 *         specified in flags.
 * \retval XME_STATUS_INVALID_CONFIGURATION if the memory specified by addr
 *         and length has not been mapped.
 * \retval XME_STATUS_WOULD_BLOCK if XME_HAL_MAP_SYNC_INVALIDATE was specified
 *         in flags and a memory lock exists for the specified address range.
 */
xme_status_t
xme_hal_mmap_syncMappedMemory
(
    void* addr,
    size_t length,
    uint32_t flags
);

/**
 * \brief Deletes the mapping(s) for the specified address range.
 *
 * \note Further references to addresses within the range after calling this
 *       function will generate invalid memory references.
 *       Closing an associated file descriptor does not unmap the associated
 *       memory region.
 *
 * \param[in] addr Starting address of memory are to unmap.
 *            Must be a multiple of the system page size.
 * \param[in] length Length of memory area to unmap.
 *
 * \retval XME_STATUS_SUCCESS if the memory area was successfully unmapped.
 * \retval XME_STATUS_INVALID_PARAMETER if addr was not a multiple of the
 *         system page size.
 */
xme_status_t
xme_hal_mmap_unmapMemory
(
    void* addr,
    size_t length
);

/**
 * \brief Returns the system memory page size.
 *
 * \return System memory page size.
 */
uint32_t
xme_hal_mmap_getSystemPageSize(void);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/mmap_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_MMAP_H
