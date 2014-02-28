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
 * $Id: dataHandlerShadow.c 5099 2013-09-18 09:29:24Z camek $
 */

/**
 * \file
 *         Data Handler Shadow.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/hal/include/mem.h"

#include "xme/core/dataManagerTypes.h"
#include "xme/core/dataHandler/include/dataHandler.h"
#include "xme/core/dataHandler/include/dataHandlerInternalTypes.h"
#include "xme/core/dataHandler/include/dataHandlerInternalMethods.h"
#include "xme/core/dataHandler/include/dataHandlerTestsystemInterface.h"
#include "xme/core/log.h"

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
static xme_status_t
setUsageOfShadowGlobal(bool activation);

static xme_status_t
setUsageOfShadowForSpecificPort(xme_core_dataManager_dataPacketId_t port, bool activation);

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_writeData(xme_core_dataManager_dataPacketId_t port,
                                     void const * const buffer, unsigned int bufferSize) {
    return xme_core_dataHandler_writeDataWithDirective(port, 0U, buffer, bufferSize,
                                                       DATAHANDLER_STATE_WRITE_SHADOWELEMENT);
}

xme_status_t
xme_core_dataHandlerShadow_writeDataWithOffset(xme_core_dataManager_dataPacketId_t port,
                                               unsigned int offset, void const * const buffer,
                                               unsigned int bufferSize) {
    return xme_core_dataHandler_writeDataWithDirective(port, offset, buffer, bufferSize,
                                                        DATAHANDLER_STATE_WRITE_SHADOWELEMENT_WITHOFFSET);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_writeAttribute(xme_core_dataManager_dataPacketId_t port,
                                          xme_core_attribute_key_t attributeKey,
                                          void const * const buffer, unsigned int bufferSize) {
    return xme_core_dataHandler_writeAttributeWithDirective(port, attributeKey, 0U, buffer,
                                                            bufferSize,
                                                            DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE);
}

xme_status_t
xme_core_dataHandlerShadow_writeAttributeWithOffset(xme_core_dataManager_dataPacketId_t port,
                                                    xme_core_attribute_key_t attributeKey,
                                                    unsigned int offset, void const * const buffer,
                                                    unsigned int bufferSize) {
    return xme_core_dataHandler_writeAttributeWithDirective(
                    port, attributeKey, offset, buffer, bufferSize,
                    DATAHANDLER_STATE_WRITE_SHADOWATTRIBUTE_WITHOFFSET);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_readData(xme_core_dataManager_dataPacketId_t port, void * const buffer,
                                    unsigned int bufferSize, unsigned int * const bytesRead) {
    return xme_core_dataHandler_readDataWithDirective(port, 0U, buffer, bufferSize, bytesRead,
                                                      DATAHANDLER_STATE_READ_SHADOWELEMENT);
}

xme_status_t
xme_core_dataHandlerShadow_readDataWithOffset(xme_core_dataManager_dataPacketId_t port,
                                              unsigned int offset, void * const buffer,
                                              unsigned int bufferSize,
                                              unsigned int * const bytesRead) {
    return xme_core_dataHandler_readDataWithDirective(
                    port, offset, buffer, bufferSize, bytesRead,
                    DATAHANDLER_STATE_READ_SHADOWELEMENT_WITHOFFSET);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_readAttribute(xme_core_dataManager_dataPacketId_t port,
                                         xme_core_attribute_key_t attributeKey, void * const buffer,
                                         unsigned int bufferSize, unsigned int * const bytesRead) {
    return xme_core_dataHandler_readAttributeWithDirective(port, attributeKey, 0U, buffer,
                                                           bufferSize, bytesRead,
                                                           DATAHANDLER_STATE_READ_SHADOWATTRIBUTE);
}

xme_status_t
xme_core_dataHandlerShadow_readAttributeWithOffset(xme_core_dataManager_dataPacketId_t port,
                                                   xme_core_attribute_key_t attributeKey,
                                                   unsigned int offset, void * const buffer,
                                                   unsigned int bufferSize,
                                                   unsigned int * const bytesRead) {
    return xme_core_dataHandler_readAttributeWithDirective(
                    port, attributeKey, offset, buffer, bufferSize, bytesRead,
                    DATAHANDLER_STATE_READ_SHADOWATTRIBUTE_WITHOFFSET);
}

//******************************************************************************//

xme_status_t
xme_core_dataHandlerShadow_lock(xme_core_dataManager_dataPacketId_t port){
    xme_core_dataHandler_dataStructure_t * const shadowPort =
        (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    XME_CHECK((NULL != shadowPort), XME_STATUS_INTERNAL_ERROR);
    shadowPort->shadowEntry.lockedByTestSystem = true;
    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_dataHandlerShadow_unlock(xme_core_dataManager_dataPacketId_t port){
    xme_core_dataHandler_dataStructure_t * const shadowPort =
        (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.shadowHandlerPorts[port];
    XME_CHECK((NULL != shadowPort), XME_STATUS_INTERNAL_ERROR);
    shadowPort->shadowEntry.lockedByTestSystem = false;
    return XME_STATUS_SUCCESS;
}

//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_activateShadowDatabase(void)
{
    return setUsageOfShadowGlobal((bool)true);
}

xme_status_t
xme_core_dataHandlerShadow_deactivateShadowDatabase(void)
{
    return setUsageOfShadowGlobal((bool)false);
}

//******************************************************************************//
xme_status_t
xme_core_dataHandlerShadow_activateShadowElement
(
    xme_core_dataManager_dataPacketId_t port
)
{
    return setUsageOfShadowForSpecificPort(port, (bool)true);
}

xme_status_t
xme_core_dataHandlerShadow_deactivateShadowElement
(
    xme_core_dataManager_dataPacketId_t port
)
{
    return setUsageOfShadowForSpecificPort(port, (bool)false);
}

//******************************************************************************//
static xme_status_t
setUsageOfShadowGlobal(bool activation) {
    register uint32_t i;
    xme_status_t status;

    for (i = 1U; i < (uint32_t) xme_core_dataHandler_dataHandlerInternals.dataPacketId; ++i) {
        status = setUsageOfShadowForSpecificPort(i, activation);
        XME_CHECK(XME_STATUS_SUCCESS == status, status);
    }
    return XME_STATUS_SUCCESS;
}

static xme_status_t
setUsageOfShadowForSpecificPort(xme_core_dataManager_dataPacketId_t port, bool activation){
    xme_core_dataHandler_dataStructure_t * const internalPort =
                    (xme_core_dataHandler_dataStructure_t *) xme_core_dataHandler_dataHandlerInternals.dataHandlerPorts[port];
    XME_CHECK((NULL != internalPort), XME_STATUS_INTERNAL_ERROR);

    internalPort->shadowEntry.useShadowEntry = activation;
    return XME_STATUS_SUCCESS;
}
