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
 * $Id: circularBuffer_arch.c 7735 2014-03-10 13:30:51Z ruiz $
 */

/**
 * \file
 *         Circular Buffer abstraction (architecture specific part: generic OS
 *         based implementation).
 */

/**
 * \addtogroup hal_circularBuffer
 * @{
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/hal/include/circularBuffer.h"

#include "xme/hal/include/mem.h"

#include "xme/core/log.h"

#include <inttypes.h>

//******************************************************************************//
//***   Protoypes                                                            ***//
//******************************************************************************//

/**
 * \brief Advances one position in the tail of the circular buffer. 
 *
 * \note The number of elements of the circular buffer are reduced by one. 
 *
 * \param[in] circularBuffer The circular buffer. 
 */
static void
advanceReadTailPosition
(
    xme_hal_circularBuffer_t* circularBuffer
);

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//

/*
 * The following implementation of the circular buffer are inspired by a
 * StackOverflow post under the cc-wiki-sa 3.0 license
 * (see http://creativecommons.org/licenses/by-sa/3.0/
 *  and http://blog.stackoverflow.com/2009/06/attribution-required/).
 *
 * Original question:
 *  - http://stackoverflow.com/questions/827691/
 *
 * The following authors contributed to this question:
 *  - paxdiablo: http://stackoverflow.com/users/14860/
 *  - Adam Rosenfield: http://stackoverflow.com/users/9530/
 *
 * The following code until the "----- END cc-by-sa 3.0 -----" marker may be
 * licensed under the license of CHROMOSOME or the cc-wiki-sa 3.0 license.
 */

static void
advanceReadTailPosition
(
    xme_hal_circularBuffer_t* circularBuffer
)
{
    // Advance a position in the read.
    circularBuffer->readTail = (char*)circularBuffer->readTail + circularBuffer->itemSizeInBytes;

    // If the circular buffer arrives to the last position, go back to the first position. 
    if(circularBuffer->readTail == circularBuffer->circularBufferEnd)
    {
        circularBuffer->readTail = circularBuffer->circularBufferStart;
    }

    // Reduce the number of elements to read. 
    circularBuffer->count--;
}

xme_status_t
xme_hal_circularBuffer_init
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t maximumSize, 
    size_t itemSizeInBytes,
    bool overwrite
)
{
    uint32_t circularBufferSizeInBytes;

    XME_CHECK(maximumSize > 0, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(itemSizeInBytes > 0, XME_STATUS_INVALID_PARAMETER);

    circularBufferSizeInBytes = maximumSize * itemSizeInBytes;

    circularBuffer->circularBufferStart = (xme_hal_circularBuffer_t*) xme_hal_mem_alloc(circularBufferSizeInBytes);

    XME_CHECK(NULL != circularBuffer->circularBufferStart, XME_STATUS_OUT_OF_RESOURCES);

    circularBuffer->circularBufferEnd = (char*) circularBuffer->circularBufferStart + circularBufferSizeInBytes;

    circularBuffer->sizeOfCircularBuffer = maximumSize;
    circularBuffer->count = 0;
    circularBuffer->itemSizeInBytes = itemSizeInBytes;

    // Both read and write head and tail point to initial buffer position. 
    circularBuffer->writeHead = circularBuffer->circularBufferStart;
    circularBuffer->readTail = circularBuffer->circularBufferStart;

    // Set the overwrite policy. 
    circularBuffer->overwrite = overwrite;

    return XME_STATUS_SUCCESS;
}

void 
xme_hal_circularBuffer_fini
(
    xme_hal_circularBuffer_t* circularBuffer
)
{
    xme_hal_mem_free(circularBuffer->circularBufferStart);
    // clear out other fields too, just to be safe
}

xme_status_t
xme_hal_circularBuffer_pushBack
(
    xme_hal_circularBuffer_t* circularBuffer, 
    const void* item
)
{
    XME_ASSERT(NULL != circularBuffer);

    // If the circular buffer write position (head) is going to overwrite the
    // the read position (tail)
    if(circularBuffer->count == circularBuffer->sizeOfCircularBuffer)
    {
        if (circularBuffer->overwrite)
        {
            uint32_t writePosition;
            uint32_t readPosition;

            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_circularBuffer_getHeadPosition(circularBuffer, &writePosition), XME_STATUS_INVALID_CONFIGURATION);
            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_circularBuffer_getTailPosition(circularBuffer, &readPosition), XME_STATUS_INVALID_CONFIGURATION);

            XME_LOG
            (
                XME_LOG_DEBUG,
                "CircularBuffer: Overwriting a position that was not previously read.\n"
                "   Current Elements in the queue: %" PRIu32 "\n"
                "   Current Read Position:         %" PRIu32 "\n"
                "   Current Write Position:        %" PRIu32 "\n",
                circularBuffer->count, readPosition, writePosition
            );

            advanceReadTailPosition(circularBuffer);
        }
        else
        {
            return XME_STATUS_PERMISSION_DENIED;
        }
    }

    // TODO: Check the return value. 
    (void) xme_hal_mem_copy(circularBuffer->writeHead, item, circularBuffer->itemSizeInBytes);

    // Advance a position of the head. 
    circularBuffer->writeHead = (char*)circularBuffer->writeHead + circularBuffer->itemSizeInBytes;

    // If we have already arrive to the end, set back to the first position. 
    if(circularBuffer->writeHead == circularBuffer->circularBufferEnd)
    {
        circularBuffer->writeHead = circularBuffer->circularBufferStart;
    }

    // Increment the count of elements. 
    circularBuffer->count++;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_circularBuffer_popFront
(
    xme_hal_circularBuffer_t* circularBuffer, 
    void* item
)
{
    XME_ASSERT(NULL != circularBuffer);

    // If there are no more elements to read, return error status. 
    if(circularBuffer->count == 0)
    {
        // handle error
        return XME_STATUS_PERMISSION_DENIED;
    }

    // Copy the read tail to the output variable. 
    (void) xme_hal_mem_copy(item, circularBuffer->readTail, circularBuffer->itemSizeInBytes);

    // Advance a position in the read tail.
    advanceReadTailPosition(circularBuffer);

    return XME_STATUS_SUCCESS;
}

/* ----- END cc-by-sa 3.0 ----- */

xme_status_t
xme_hal_circularBuffer_getTailPosition
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t* const position
)
{
    uintptr_t tailRelativeOffset;

    XME_ASSERT(NULL != circularBuffer);

    tailRelativeOffset = (uintptr_t) circularBuffer->readTail - (uintptr_t) circularBuffer->circularBufferStart;

    *position = (uint32_t) (((uint32_t) tailRelativeOffset) / circularBuffer->itemSizeInBytes);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_hal_circularBuffer_getHeadPosition
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t* const position
)
{
    uintptr_t headRelativeOffset;

    XME_ASSERT(NULL != circularBuffer);

    headRelativeOffset = (uintptr_t) circularBuffer->writeHead - (uintptr_t) circularBuffer->circularBufferStart;

    *position = (uint32_t) (((uint32_t) headRelativeOffset) / circularBuffer->itemSizeInBytes);

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */
