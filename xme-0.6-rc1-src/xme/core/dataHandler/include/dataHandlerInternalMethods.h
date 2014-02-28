/*
 * Copyright (c) 2011-2012, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dataHandlerInternalMethods.h 5121 2013-09-19 13:51:44Z camek $
 */

/**
 * \file
 *         Data Handler Types.
 */

#ifndef XME_CORE_DATAHANDLER_DATAHANDLERINTERNALMETHODS_H
#define XME_CORE_DATAHANDLER_DATAHANDLERINTERNALMETHODS_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/defines.h"

//******************************************************************************//
//***   Defines                                                              ***//
//******************************************************************************//

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Creates date entry in the data store and the shadow store.
 * \param[in] value describes all parameters to create the data entry element.
 * \param[out] port is the given port handle, which identifies the data element or 0 if failed
 *             to create the data entry.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if data structure was correct created.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t
xme_core_dataHandler_createDataEntry
(
        xme_core_dataHandler_setup_t const * const value,
        xme_core_dataManager_dataPacketId_t * const port
);

/**
 * \brief Reads data from a port into the given buffer. This is the internal version of \a xme_core_dataHandler_readData.
 *        For parameters and details please \sa xme_core_dataHandler_readData.
 * \param dataPacketId specifies the handle for an allocated memory in the database or queue.
 *             The memory contains the data which shall be read.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller then at least the given bufferSize will be returned.
 * \param directive the directive. 
 * \retval XME_STATUS_SUCCESS if the data is successfully read. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data is not successfully read. 
 */
extern xme_status_t
xme_core_dataHandler_readDataWithDirective
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint32_t offset,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead,
    xme_core_dataHandler_states_t directive
);

/**
 *\brief Reads attributes from a port into the given buffer. This is the internal version of \a xme_core_dataHandler_readAttribute.
 *       For parameters and details please \sa xme_core_dataHandler_readAttribute.
 * \param dataPacketId specifies the handle for an allocated memory in the database or queue.
 *             The memory contains the data which shall be read.
 * \param attributeKey the attribute key. 
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller then at least the given bufferSize will be returned.
 * \param directive the directive. 
 * \retval XME_STATUS_SUCCESS if the data is successfully read. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data is not successfully read. 
 */
extern xme_status_t
xme_core_dataHandler_readAttributeWithDirective
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_attribute_key_t attributeKey,
    uint32_t offset,
    void * const buffer,
    uint32_t bufferSize,
    uint32_t * const bytesRead,
    xme_core_dataHandler_states_t directive
);

/**
  *\brief Write data to a port from the given buffer. This is the internal version of \a xme_core_dataHandler_writeData.
 *       For parameters and details please \sa xme_core_dataHandler_writeData.
 * \param dataPacketId specifies the handle for an allocated memory in the database or queue.
 *             The memory contains the data which shall be read.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param directive the directive. 
 * \retval XME_STATUS_SUCCESS if the data is successfully written. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data is not successfully written. 
 */
extern xme_status_t
xme_core_dataHandler_writeDataWithDirective
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint32_t offset,
    void const * const buffer,
    uint32_t bufferSize,
    xme_core_dataHandler_states_t directive
);

/**
  *\brief Writes attributes to a port from the given buffer. This is the internal version of \a xme_core_dataHandler_writeAttribute.
 *        For parameters and details please \sa xme_core_dataHandler_writeAttribute.
 * \param dataPacketId specifies the handle for an allocated memory in the database or queue.
 *             The memory contains the data which shall be read.
 * \param attributeKey the attribute key.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param directive the directive. 
 * \retval XME_STATUS_SUCCESS if the data is successfully written. 
 * \retval XME_STATUS_INTERNAL_ERROR if the data is not successfully written. 
 */
extern xme_status_t
xme_core_dataHandler_writeAttributeWithDirective
(
     xme_core_dataManager_dataPacketId_t dataPacketId,
     xme_core_attribute_key_t attributeKey,
     uint32_t offset,
     void const * const buffer,
     uint32_t bufferSize,
     xme_core_dataHandler_states_t directive
);

/**
 * \brief Initializes the internal information used to organize the database or the queue.
 * \param amountOfMemory the amount of memory. 
 */
extern void xme_core_dataHandler_initializeInternalDataStructure(size_t amountOfMemory);

/**
 * \brief Uninitializes the internal information.
 */
extern void xme_core_dataHandler_uninitializeInternalDataStructure(void);

/**
 * \brief Initializes the data handler from a given file image. This image was stored during runtime,
 *        in order to allow later a reload of data handler layout. Additionally, the image will be
 *        generated when the finalization of the data handler will be invoked.
 * \param image describes the image file which shall be loaded.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if data structure was correct initialized.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_initializeDataStructure(xme_core_dataHandler_imageDataStructure_t image);

/**
 * \brief When a image shall be loaded it will be checked if it is correct and the data it contains
 *        is valid. If all these checks are correct, then the loaded structure will be used
 *        as a data handler instance.
 * \param image describes the image file which shall be checked.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if image is valid.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_checkImageForConsistancy(xme_core_dataHandler_imageDataStructure_t image);

/**
 * \brief This functions checks whether the internal structure is a valid one or not.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if internal structure is valid.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_checkInternalConistency(void);

/**
 * \brief This function searches in the given data structure layout for a given element.
 * \param[in] dataPacketId is the identifier returned by the database or queue.
 * \return the matching data structure element or NULL
 */
extern xme_core_dataHandler_dataStructure_t *
xme_core_dataHandler_findPortInInternalDataStructure
(
     xme_core_dataManager_dataPacketId_t dataPacketId
);

/**
 * \brief This function searches in the data handler for a specific attribute.
 * \param[in] dataPacketId is the identifier returned by the database or queue.
 * \param[in] attributeKey is the key of the attribute stored in the data packet.
 * \return the matching data structure element or NULL
 */
extern xme_core_dataHandler_attributeElement_t *
xme_core_dataHandler_findAttribute
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_attribute_key_t attributeKey,
    bool useShadow
);

/**
 * \brief This function checks if a persistent storage contains an image of the data handler
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if an image exists.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 *          - XME_CORE_STATUS_INVALID_PARAMETER
 */
extern xme_status_t xme_core_dataHandler_checkDataStorageForImage(void);

/**
 * \brief This function reads a data image from disk.
 * \return the parsed image if it was correct parsed otherwise NULL
 */
extern xme_core_dataHandler_imageDataStructure_t xme_core_dataHandler_loadImage(void);

/**
 *\brief This function calculates a hash value by using Jenkins hash function.
 */
extern uint32_t xme_core_dataHandler_calculateHash(char const * const key, size_t length);

/**
 * \brief This function checks if a port is valid.
 * \param[in] the current used port.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if port is valid.
 *          - XME_CORE_STATUS_INTERNAL_ERROR
 */
static INLINE xme_status_t xme_core_dataHandler_checkPort(xme_core_dataManager_dataPacketId_t port);
static INLINE xme_status_t xme_core_dataHandler_checkPort(xme_core_dataManager_dataPacketId_t port){
    XME_CHECK(xme_core_dataHandler_dataHandlerInternals.initialized == true, XME_STATUS_INTERNAL_ERROR);
    XME_CHECK((XME_CORE_DATAMANAGER_DATAPACKETID_INVALID < port) &&
              (XME_CORE_DATAMANAGER_DATAPACKETID_MAX > port),
              XME_STATUS_INVALID_PARAMETER);
    return XME_STATUS_SUCCESS;
}

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATAHANDLERINTERNALMETHODS_H */
