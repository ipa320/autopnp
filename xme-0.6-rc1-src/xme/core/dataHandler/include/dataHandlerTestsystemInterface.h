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
 * $Id: dataHandlerTestsystemInterface.h 5099 2013-09-18 09:29:24Z camek $
 */

/**
 * \file
 *         Data Handler Testsystem Interface contains all important function to use the shadow database,
 *         which contains all information as the original database. Normally both databases are in synch.
 *         When a testsystem overwrites port specific information then the shadow database contains different
 *         data compared to the original one.
 *         To conserve the overwritten data in the shadow database the testsystem must lock the specific
 *         port, thus the internals of the database cannot resynch both databases.
 *         Locking of a port is not used to provide the information to the specific component. In order to
 *         use this, the testsystem must call activateDatabase, which enforces to all given data of the
 *         shadow database, or activateShadowElement, which activates only the data given of a specific port.
 *         To keep symmetry, deactivation and unlock functions exists. This allow different usage of the
 *         anti-functions. Deactivation and unlocking can be called in combination, which allows an
 *         resynch of both databases. When deactivation is called without unlocking avoids a resynch of
 *         both databases, but enforces other components to use the data from the original database.
 *         Additionally, unlocking can be used without deactivation. This enforces other components to
 *         read their data from the shadow database, but allows the system to resynch both databases.
 *
 */

#ifndef XME_CORE_DATAHANDLER_TESTSYSTEM_INTERFACE_H
#define XME_CORE_DATAHANDLER_TESTSYSTEM_INTERFACE_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

//******************************************************************************//
//***   Static variables                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Writes data into the shadow database. This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database on which the data should be written
 * \param buffer user provided storage, from which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_writeData
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 void const * const buffer,
 uint32_t bufferSize
);

/**
 * \brief Writes data with a given offset into the shadow database.
 * This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database on which the data should be written
 * \param offset indicates the value which will be added to the starting point of the buffer
 * \param buffer user provided storage, from which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It indicates how much data shall be copied to the database or queue.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_writeDataWithOffset
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 uint32_t offset,
 void const * const buffer,
 uint32_t bufferSize
);

/**
 * \brief Writes data to the shadow data base. This function should only be called by the Testsystem. 
 *
 * \param dataPacketId the data packet of the database from which the data should be write. 
 * \param attributeKey the attribute key. 
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_writeAttribute
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 xme_core_attribute_key_t attributeKey,
 void const * const buffer,
 uint32_t bufferSize
);

/**
 * \brief Writes data to the shadow data base with a given offset.
 *        This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database from which the data should be write.
 * \param attributeKey the key of an existing attribute.
 * \param offset indicates the value which will be added to the starting point of the buffer
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Subscription-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_writeAttributeWithOffset
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 xme_core_attribute_key_t attributeKey,
 uint32_t offset,
 void const * const buffer,
 uint32_t bufferSize
);

/**
 * \brief Reads data from the shadow database. This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database from which the data should be read.
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller, then at least the given bufferSize will be returned.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_readData
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 void * const buffer,
 uint32_t bufferSize,
 uint32_t * const bytesRead
);

/**
 * \brief Reads data from the shadow database with an offset.
 *        This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database from which the data should be read.
 * \param offset indicates the value which will be added to the starting point of the buffer
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller, then at least the given bufferSize will be returned.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if writing data into the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_readDataWithOffset
(
 xme_core_dataManager_dataPacketId_t dataPacketId,
 uint32_t offset,
 void * const buffer,
 uint32_t bufferSize,
 uint32_t * const bytesRead
);

/**
 * \brief Reads attribute from the shadow database. This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database from which the data should be read.
 * \param attributeKey the attribute key. 
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller, then at least the given bufferSize will be returned.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if attribute reading from the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_readAttribute
(
  xme_core_dataManager_dataPacketId_t dataPacketId,
  xme_core_attribute_key_t attributeKey,
  void * const buffer,
  uint32_t bufferSize,
  uint32_t * const bytesRead
);

/**
 * \brief Reads attribute from the shadow database with an offset.
 *        This function should only be called by the Testsystem.
 *
 * \param dataPacketId the data packet of the database from which the data should be read.
 * \param attributeKey the attribute key.
 * \param offset indicates the value which will be added to the starting point of the buffer
 * \param buffer is a user provided storage, to which the data is copied.
 * \param bufferSize specifies the size of the provided storage.
 *                   It can be bigger, equal, or smaller than the data element stored in the
 *                   database or queue.
 * \param bytesRead indicates how much data was copied. If the \a bufferSize is bigger or equal,
 *                  then the maximum amount of the stored memory will be returned. If the given \a bufferSize
 *                  is smaller, then at least the given bufferSize will be returned.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if attribute reading from the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if given port is a Publication-Port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_readAttributeWithOffset
(
  xme_core_dataManager_dataPacketId_t dataPacketId,
  xme_core_attribute_key_t attributeKey,
  uint32_t offset,
  void * const buffer,
  uint32_t bufferSize,
  uint32_t * const bytesRead
);

/**
 * \brief Locks the given port in the shadow database avoiding overwriting by a different component.
 *        This hinders other component, when they call writeData, to update the shadow db by new information.
 *        It also allows to setup bigger test information in the shadow db, and then after finishing that
 *        to activate the whole test information with one shot.
 *        For activation of the usage of the shadow db please see \see xme_core_dataHandlerShadow_activateShadowElement.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the activation of the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
extern xme_status_t
xme_core_dataHandlerShadow_lock(xme_core_dataManager_dataPacketId_t port);

/**
 * \brief Unlocks the given port. This allows to resynch the shadwo database with the orginial database.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the activation of the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 *
 */
extern xme_status_t
xme_core_dataHandlerShadow_unlock(xme_core_dataManager_dataPacketId_t port);

/**
 * \brief Activates the shadow database. This function should only be called by the Testsystem.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the activation of the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
extern xme_status_t
xme_core_dataHandlerShadow_activateShadowDatabase(void);

/**
 * \brief Deactivates the (currently active) shadow database. This function should only be called by the Testsystem.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the deactivation of the shadow database was successful.
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_deactivateShadowDatabase(void);

/**
 * \brief Activates a single element of the shadow database. This function should only be called by the Testsystem.
 *
 * \param port the element of the shadow database that should be activated.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the activation of the element of the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if the given port was not a publication nor a RTE port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error.
 */
extern xme_status_t
xme_core_dataHandlerShadow_activateShadowElement
(
 xme_core_dataManager_dataPacketId_t port
);

/**
 * \brief Deactivates one (currently active) element of the shadow database. This function should only be called by the Testsystem.
 *
 * \param port the (currently active) element of the shadow database that should be deactivated
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the deactivation of the element of the shadow database was successful.
 *          - XME_CORE_STATUS_PERMISSION_DENIED if the given port was not a publication nor a RTE port
 *          - XME_CORE_STATUS_INTERNAL_ERROR in case of any other error
 */
extern xme_status_t
xme_core_dataHandlerShadow_deactivateShadowElement
(
    xme_core_dataManager_dataPacketId_t port
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_TESTSYSTEM_INTERFACE_H */
