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
 * $Id: mem.h 3977 2013-07-02 15:22:10Z geisinger $
 */

/**
 * \file
 * \brief Memory abstraction.
 */

#ifndef XME_HAL_MEM_H
#define XME_HAL_MEM_H

/**
 * \defgroup hal_mem General Memory Abstraction
 * @{
 *
 * \brief General Memory abstraction.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"

#include <stdint.h>
#include <stddef.h>

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief  Allocates a block of size bytes of initialized memory, returning a
 *         pointer to the beginning of the block.
 *
 * The content of the newly allocated block of memory is guaranteed to be
 * initialized with all zeroes.
 *
 * \param  size Size of the memory block, in bytes.
 *
 * \return On success, returns a non-NULL pointer to the memory block allocated
 *         by the function. If the function failed to allocate the requested
 *         block of memory, a NULL pointer is returned.
 */
void*
xme_hal_mem_alloc
(
    size_t size
);

/**
 * \brief  Changes the size of the given memory block, expanding or reducing
 *         the amount of memory available in the block.
 *
 * The function may move the memory block to a new location, in which case the
 * new location is returned. The content of the memory block is preserved up
 * to the smaller of the current and the new size even if the block is moved.
 *
 * If the new memory block is larger than the old memory block, the new part
 * is not guaranteed to be initialized with all zeroes. If the new memory block
 * is smaller than the old memory block, the memory range after the new block
 * may still contain the old data. Hence, make sure that the newly allocated
 * memory area is properly initialized and the to-be abandoned memory area is
 * overwritten before calling this function if the contained data are sensitive.
 * Although xme_hal_mem_alloc() guarantees the newly allocated memory to be
 * initialized, this does not protect an application from in-memory leakage of
 * sensitive data.
 *
 * \note   NULL may be passed in mem. In this case, the function behaves
 *         exactly as xme_hal_mem_alloc(). In case NULL is passed, a new
 *         block is allocated and a pointer to it is returned by the function
 *         if the memory was successfully allocated.
 *
 * \param  mem Pointer to a memory block previously allocated with
 *         xme_hal_mem_alloc() to be reallocated. This parameter can be NULL.
 * \param  size New size for the memory block, in bytes. If zero is passed and
 *         mem is non-NULL, the memory block pointed by mem is deallocated
 *         and a NULL pointer is returned.
 *
 * \return A pointer to the reallocated memory block, which may be one of the
 *         following:
 *          - A memory block with the same content as the memory block passed
 *            in mem, but with the given new size, if mem was non-NULL, size
 *            was non-zero and any required memory allocation was successful.
 *          - A newly allocated memory block in case mem was NULL, size was
 *            non-zero and the required memory was successfully allocated.
 *          - NULL if mem was non-NULL, size was zero and the memory was
 *            successfully deallocated or if mem was non-NULL, size was
 *            non-zero and the function failed to allocate the required
 *            memory. In the latter case, the memory block pointed to by mem
 *            is not deallocated (it is still valid and its content unchanged).
 */
void*
xme_hal_mem_realloc
(
    void* mem,
    size_t size
);

/**
 * \brief  Releases the given block of memory.
 *
 * Make sure that the to-be abandoned memory area is overwritten before calling
 * this function if the contained data are sensitive. Although xme_hal_mem_alloc()
 * guarantees the newly allocated memory to be initialized, this does not protect
 * an application from in-memory leakage of sensitive data.
 *
 * \param  mem Block of memory to release.
 */
void
xme_hal_mem_free
(
    void* mem
);

/**
 * \brief  Sets the first num bytes of the block of memory pointed by mem
 *         to the specified value (interpreted as an unsigned char).
 *
 * \param  mem Pointer to the block of memory to fill.
 * \param  value Value to be set.
 * \param  num Number of bytes to be set to the value.
 *
 * \return Returns mem.
 */
void*
xme_hal_mem_set
(
    void* mem,
    uint8_t value,
    size_t num
);

/**
 * \brief  Copies the values of num bytes from the location pointed by source
 *         directly to the memory block pointed by destination.
 *
 *         The arrays pointed by both the destination and source parameters
 *         shall be at least num bytes and should not overlap.
 *
 * \param  destination Pointer to the destination array where the content is to
 *         be copied, type-casted to a pointer of type void*.
 * \param  source Pointer to the source of data to be copied, type-casted to
 *         a pointer of type void*.
 * \param  num Number of bytes to copy.
 *
 * \return Returns destination.
 */
void*
xme_hal_mem_copy
(
    void* destination,
    const void* source,
    size_t num
);

/**
 * \brief  Compares the first num bytes of the block of memory pointed by buf1
 *         to the first num bytes pointed by buf2, returning zero if they all
 *         match or a value different from zero representing which is greater
 *         if they do not.
 *
 * \param  buf1 Pointer to the first block of memory.
 * \param  buf2 Pointer to the second block of memory.
 * \param  num Number of bytes to compare.
 *
 * \return Returns an integral value indicating the relationship between the
 *         content of the memory blocks: A zero value indicates that the
 *         contents of both memory blocks are equal. A value greater than
 *         zero indicates that the first byte that does not match in both
 *         memory blocks has a greater value in ptr1 than in ptr2 as if
 *         evaluated as unsigned char values; And a value less than zero
 *         indicates the opposite.
 */
int
xme_hal_mem_compare
(
    const void* buf1,
    const void* buf2,
    size_t num
);

XME_EXTERN_C_END

/******************************************************************************/
/***   Platform-specific includes                                           ***/
/******************************************************************************/
#include "xme/hal/mem_arch.h"

/**
 * @}
 */

#endif // #ifndef XME_HAL_MEM_H
