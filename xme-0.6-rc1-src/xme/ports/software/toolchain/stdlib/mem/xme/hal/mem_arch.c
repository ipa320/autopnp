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
 * $Id: mem_arch.c 4945 2013-09-04 07:37:46Z ruiz $
 */

/**
 * \file
 *         Memory abstraction (architecture specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_mem 
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void*
xme_hal_mem_alloc
(
    size_t size
)
{
    return xme_fallback_calloc(size, 1);
}

void*
xme_hal_mem_realloc
(
    void* mem,
    size_t size
)
{
    return xme_fallback_realloc(mem, size);
}

void
xme_hal_mem_free
(
    void* mem
)
{
    xme_fallback_free(mem);
}

void*
xme_hal_mem_set
(
    void* mem,
    uint8_t value,
    size_t num
)
{
    return xme_fallback_memset(mem, value, num);
}

void*
xme_hal_mem_copy
(
    void* destination,
    const void* source,
    size_t num
)
{
    return xme_fallback_memcpy(destination, source, num);
}

int
xme_hal_mem_compare
(
    const void* buf1,
    const void* buf2,
    size_t num
)
{
    return xme_fallback_memcmp(buf1, buf2, num);
}

/**
 * @}
 */
