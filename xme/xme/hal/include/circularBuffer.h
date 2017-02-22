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
 * $Id: circularBuffer.h 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Circular Buffer abstraction.
 */

#ifndef XME_HAL_CIRCULARBUFFER_H
#define XME_HAL_CIRCULARBUFFER_H

/**
 * \defgroup hal_circularBuffer Circular buffer abstraction
 * @{
 *
 * \brief   General circular buffer abstraction.
 *
 * \details The functions in this group allow to manage a circular buffer that
 *          may store up to a given number of elements of equal size. Elements
 *          are read in the order that they have been written. A read operation
 *          on an empty buffer and a write operation to a full buffer result in
 *          an error.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/defines.h"

#include <stdint.h>
#include <stdbool.h>

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

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/**
 * \struct xme_hal_circularBuffer_t
 * \brief The struct storing the circular buffer. 
 */
typedef struct
{
    void *circularBufferStart;     ///< The pointer to the first position of the circular buffer. 
    void *circularBufferEnd;       ///< The pointer to the end position of the circular buffer.
    uint32_t sizeOfCircularBuffer; ///< The size of the circular buffer. 
    uint32_t count;                ///< Number of items stored in the buffer.
    size_t itemSizeInBytes;        ///< Size of each item in the buffer.
    void* writeHead;               ///< Pointer to write head.
    void* readTail;                ///< Pointer to read tail.
    bool overwrite;                ///< Establish if the push operation overwrites. 
} xme_hal_circularBuffer_t;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

// TODO: Modify circular buffer API in order to allow static allocation of memory!

/**
 * \brief Initializes a circular buffer. 
 *
 * \note If overwrite is set to true, the tail position advances
 *       one position, and the push operation is always success. 
 * \note If overwrite is set to false, the push operation fail
 *       when the circular buffer is full. 
 *
 * \param[in] circularBuffer The circular buffer to create. 
 * \param[in] maximumSize The maximum size of the queue. 
 * \param[in] itemSizeInBytes The size of each individual item. 
 * \param[in] overwrite Establishes the overwrite flag for push operation. 
 *
 * \retval XME_STATUS_SUCCESS If the circular buffer is successfully created. 
 * \retval XME_STATUS_INVALID_PARAMETER If the provided reference is not
 *         a valid database structure. 
 * \retval XME_STATUS_OUT_OF_RESOURCES When there is no more free space for 
 *         creating the circular buffer. 
 */
xme_status_t
xme_hal_circularBuffer_init
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t maximumSize, 
    size_t itemSizeInBytes,
    bool overwrite
);

/**
 * \brief Frees the resources occupied by the circular buffer. 
 *
 * \param[in] circularBuffer The circular buffer. 
 */
void 
xme_hal_circularBuffer_fini
(
    xme_hal_circularBuffer_t* circularBuffer
);

/**
 * \brief Adds a new element to the end of the circular buffer. 
 * \details Writes in the next position and increments the number of
 *          elements. 
 *
 * \note If overwrite flag is set to true, the tail position advances
 *       one position, and the push operation is always success. 
 * \note If overwrite is set to false, the push operation fail
 *       when the circular buffer is full and an error is returned. 
 *
 * \param[in] circularBuffer The circular buffer. 
 * \param[in] item The item to write. 
 *
 * \retval XME_STATUS_SUCCESS If the push operation in the back
 *         of the circular buffer is success.
 * \retval XME_STATUS_PERMISSION_DENIED If cannot push the item
 *         at the back because the circular buffer is full. 
 */
xme_status_t
xme_hal_circularBuffer_pushBack
(
    xme_hal_circularBuffer_t* circularBuffer, 
    const void* item
);

/**
 * \brief Reads an element from the tail of the circular buffer. 
 *
 * \param[in] circularBuffer The circular buffer. 
 * \param[in] item The item to read. 
 *
 * \retval XME_STATUS_SUCCESS If the pop operation in the front
 *         of the circular buffer is success.
 * \retval XME_STATUS_PERMISSION_DENIED If there are no items to
 *         pop from the front of the circular buffer. 
 */
xme_status_t
xme_hal_circularBuffer_popFront
(
    xme_hal_circularBuffer_t* circularBuffer, 
    void* item
);

/* ----- END cc-by-sa 3.0 ----- */

/**
 * \brief Gets the relative position of the tail item in the circular buffer. 
 *
 * \param[in] circularBuffer The circular buffer. 
 * \param[out] position The position of the tail item. 
 *
 * \retval XME_STATUS_SUCCESS If the position is successfully stored in position variable.
 */
xme_status_t
xme_hal_circularBuffer_getTailPosition
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t* const position
);

/**
 * \brief Gets the relative position of the head item in the circular buffer. 
 *
 * \param[in] circularBuffer The circular buffer. 
 * \param[out] position The position of the head item. 
 *
 * \retval XME_STATUS_SUCCESS If the position is successfully stored in position variable.
 */
xme_status_t
xme_hal_circularBuffer_getHeadPosition
(
    xme_hal_circularBuffer_t* circularBuffer, 
    uint32_t* const position
);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_HAL_CIRCULARBUFFER_H
