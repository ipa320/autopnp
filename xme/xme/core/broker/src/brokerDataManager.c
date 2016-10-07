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
 * $Id: brokerDataManager.c 7739 2014-03-10 13:53:19Z ruiz $
 */

/**
 * \file
 *         Broker interface for Data Manager.
 */

/**
 * \addtogroup core_broker
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/

#include "xme/core/broker/include/broker.h"
#include "xme/core/broker/include/brokerInternalMethods.h"
#include "xme/core/broker/include/brokerDataManagerInterface.h"

#include "xme/core/dataHandler/include/dataHandler.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
// Documented in brokerInternalTypes.h
extern 
xme_core_broker_functionDescriptionTable_t functionDescriptions;

// Documented in brokerInternalTypes.h
extern 
transferDataPacketTable_t transferDataPacketTable;

// Documented in brokerInternalTypes.h
extern
xme_core_transferDataCallback_t xme_core_transferDataCallback;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_broker_dataAvailabilityChange
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
)
{
    //check if output data packet is a member of the source data packets 
    //in the transfer data packet table. 
    xme_status_t status;
    bool found;

    XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_broker_dataAvailabilityChange(%i, %i)\n", dataPacketId, size);

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, XME_STATUS_INVALID_PARAMETER);

    if (xme_core_broker_isInputDataPacket(dataPacketId)) 
    {
        XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_broker_updateDataReadiness(%i, %i)\n", dataPacketId, size);
        return xme_core_broker_updateDataReadiness(dataPacketId, size); // change the input data readyness
    }

    // if it is not registered in the broker, unlock in the data handler and return a NOT_FOUND error. 
    if (!xme_core_broker_isOutputDataPacket(dataPacketId))
    {
        //XME_LOG(XME_LOG_ERROR, "Broker: xme_core_broker_isOutputDataPacket(%i) not successful!\n", dataPacketId);
        //(void) xme_core_dataHandler_completeReadOperation(dataPacketId);
        return XME_STATUS_NOT_FOUND;
    }
    else if (!xme_core_broker_isOutputDataPacket(dataPacketId) && size == 0)
    {
        // FIXME: This branch is never executed (see if statement above)

        // this branch is created to avoid calls that are called from completeReadOperation for non input or output data packets. 
        return XME_STATUS_NOT_FOUND;
    }

    // avoid recursive calls. 
    if (xme_core_broker_isSourceDataPacketLocked(dataPacketId))
    {
        XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_broker_unlockSourceDataPacket(%i)\n", dataPacketId);
        return xme_core_broker_unlockSourceDataPacket(dataPacketId);
    }
        
    if (size > 0) // only check transfers for positive data availability
    {
        xme_status_t transferStatus = XME_STATUS_SUCCESS;
        bool transferError = false;
        found = false;

        XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_dataHandler_startReadOperation(%i)\n", dataPacketId);
        XME_CHECK(XME_STATUS_SUCCESS == xme_core_dataHandler_startReadOperation(dataPacketId), XME_STATUS_INTERNAL_ERROR);

        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            transferDataPacketTable, 
            xme_core_broker_transferDataPacketItem_t, 
            transferItem);
        {
            if(transferItem->srcDataPacketId == dataPacketId)
            {
                found = true; // activate this flag
                // perform the notification to the data handler
                if (xme_core_transferDataCallback == NULL) {
                    status = xme_core_dataHandler_transferData(
                        transferItem->srcDataPacketId, 
                        transferItem->destDataPacketId);
                }
                else
                {
                    status = xme_core_transferDataCallback(
                        transferItem->srcDataPacketId, 
                        transferItem->destDataPacketId);
                }

                if (status != XME_STATUS_SUCCESS)
                {
                    XME_LOG(XME_LOG_DEBUG, 
                            "Trasfer between %d [src] and %d [dst] provided the following error\n", 
                            transferItem->srcDataPacketId, 
                            transferItem->destDataPacketId);
                    transferError = true;
                    transferStatus = status;
                }

                // lock the source data packet
                transferItem->srcLocked = true;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

        XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_dataHandler_completeReadOperation(%i)\n", dataPacketId);
        status = xme_core_dataHandler_completeReadOperation(dataPacketId);

        // in case of transfer error, or no entries, notify this fact. 
        if (transferError)
        {
            return transferStatus;
        } 

        if (!found)
        {
            return XME_STATUS_NOT_FOUND;
        }

        return status;
    }
    else
    {
        // data is no longer available, so check all related input ports to deactivate them. 
        found = false;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            transferDataPacketTable, 
            xme_core_broker_transferDataPacketItem_t, 
            transferItem);
        {
            // for each entry where the srcDataPacket appears, remove availability in all dst. 
            if(transferItem->srcDataPacketId == dataPacketId)
            {
                (void) xme_core_broker_dataAvailabilityChange(transferItem->destDataPacketId, 0);
                found = true;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            
        if (!found)
        {
            return XME_STATUS_NOT_FOUND;
        }
        else
        {
            return XME_STATUS_SUCCESS;
        }
    }
}

xme_status_t
xme_core_broker_isFunctionReady
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionItem
    );
    {
        if (functionItem->componentId == componentId &&
            functionItem->functionId == functionId &&
            functionItem->functionVariantId == functionVariantId)
        {
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                functionItem->parameters,
                xme_core_broker_functionDataPacket_t,
                functionParameter);
            {
                bool ready = true;

                if (functionParameter->usage == XME_CORE_BROKER_DATAPACKET_USAGE_REQUIRED)
                {
                    ready = xme_core_broker_isDataPacketAvailable(functionParameter->dataPacketId);
                }

                // in case of finding a single false, the function is not available
                if (!ready)
                {
                    return XME_STATUS_NOT_FOUND;
                }
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

/**
 * @}
 */

