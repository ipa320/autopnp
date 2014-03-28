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
 * $Id: broker.c 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Broker.
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
#include "xme/core/broker/include/brokerPnpManagerInterface.h"

#include "xme/core/log.h"
#include "xme/hal/include/mem.h"


/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/

/**
 * \var functionDescriptions
 *
 * \brief Table associating functions and parameters.
 */
xme_core_broker_functionDescriptionTable_t functionDescriptions;

/**
 * \var dataPacketSubscribers
 *
 * \brief Table associating data packets and associated subscriber functions.
 */
xme_core_broker_dataSubscriberTable_t dataPacketSubscribers;

/**
 * \var transferDataPacketTable
 * 
 * \brief Establish relationships between source and destination data packets for transfer function. 
 */
transferDataPacketTable_t transferDataPacketTable;

/**
 * \var xme_core_transferDataCallback
 * \brief Provides a callback for data transfer.
 */
xme_core_transferDataCallback_t xme_core_transferDataCallback;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_broker_init (void* params)
{
    XME_UNUSED_PARAMETER(params);

    // new functionality storing data packets and functions
    XME_HAL_TABLE_INIT(dataPacketSubscribers);
    XME_HAL_TABLE_INIT(functionDescriptions);

    xme_core_transferDataCallback = NULL;

    XME_HAL_SINGLYLINKEDLIST_INIT(transferDataPacketTable);

    return XME_STATUS_SUCCESS;
}

void
xme_core_broker_fini (void)
{
    xme_hal_singlyLinkedList_fini(&transferDataPacketTable);
    
    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataSubscriberItem
    );
    {
        xme_hal_singlyLinkedList_fini(&dataSubscriberItem->subscribers);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(dataPacketSubscribers);

    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionDescriptionItem
    );
    {
        xme_hal_singlyLinkedList_fini(&functionDescriptionItem->parameters);
    }
    XME_HAL_TABLE_ITERATE_END();
    XME_HAL_TABLE_FINI(functionDescriptions);
}

xme_status_t
xme_core_broker_addDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    xme_core_broker_transferDataPacketItem_t* newTransferDataPacketItem;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != srcDataPacketId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dstDataPacketId, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if (transferDataPacketItem->srcDataPacketId == srcDataPacketId && 
            transferDataPacketItem->destDataPacketId == dstDataPacketId)
        {
            transferDataPacketItem->referenceCount++;
            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    {
        newTransferDataPacketItem = (xme_core_broker_transferDataPacketItem_t*) xme_hal_mem_alloc (sizeof(xme_core_broker_transferDataPacketItem_t));
        
        XME_CHECK(NULL != newTransferDataPacketItem, XME_STATUS_OUT_OF_RESOURCES);
        newTransferDataPacketItem->srcDataPacketId = srcDataPacketId;
        newTransferDataPacketItem->destDataPacketId = dstDataPacketId;
        newTransferDataPacketItem->referenceCount = 1;
        newTransferDataPacketItem->srcLocked = false;

        return xme_hal_singlyLinkedList_addItem(&transferDataPacketTable, newTransferDataPacketItem);
    }
}

xme_status_t
xme_core_broker_removeDataPacketTransferEntry
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != srcDataPacketId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dstDataPacketId, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if (transferDataPacketItem->srcDataPacketId == srcDataPacketId && 
            transferDataPacketItem->destDataPacketId == dstDataPacketId)
        {
            if (1 == transferDataPacketItem->referenceCount)
            {
                XME_CHECK(0 < xme_hal_singlyLinkedList_removeItem(&transferDataPacketTable, transferDataPacketItem, (bool)false), XME_STATUS_INTERNAL_ERROR);
            }
            else
            {
                transferDataPacketItem->referenceCount--;
            }
            return XME_STATUS_SUCCESS;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_NOT_FOUND;
}

uint16_t
xme_core_broker_dataPacketTransferEntryCount
(
    xme_core_dataManager_dataPacketId_t srcDataPacketId,
    xme_core_dataManager_dataPacketId_t dstDataPacketId
)
{
    if (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID == srcDataPacketId &&
           XME_CORE_DATAMANAGER_DATAPACKETID_INVALID == dstDataPacketId)
    {
        return 0;
    }
    else if (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != srcDataPacketId &&
                 XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dstDataPacketId)
    {
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            transferDataPacketTable, 
            xme_core_broker_transferDataPacketItem_t, 
            transferDataPacketItem);
        {
            if (transferDataPacketItem->srcDataPacketId == srcDataPacketId && 
                    transferDataPacketItem->destDataPacketId == dstDataPacketId)
            {
                return transferDataPacketItem->referenceCount;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
        return 0;
    }
    else if (XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != srcDataPacketId)
    {
        uint16_t totalCount = 0;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            transferDataPacketTable, 
            xme_core_broker_transferDataPacketItem_t, 
            transferDataPacketItem);
        {
            if (transferDataPacketItem->srcDataPacketId == srcDataPacketId)
            {
                totalCount += transferDataPacketItem->referenceCount;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
        return totalCount;
    }
    else
    {
        uint16_t totalCount = 0;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            transferDataPacketTable, 
            xme_core_broker_transferDataPacketItem_t, 
            transferDataPacketItem);
        {
            if (transferDataPacketItem->destDataPacketId == dstDataPacketId)
            {
                totalCount += transferDataPacketItem->referenceCount;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
        return totalCount;
    }
}

xme_status_t
xme_core_broker_setTransferCallback
(
    xme_core_transferDataCallback_t transferCallback
)
{
    XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_broker_setTransferCallback()\n");
    XME_CHECK(transferCallback != NULL, XME_STATUS_INVALID_PARAMETER);

    xme_core_transferDataCallback = transferCallback;
    return XME_STATUS_SUCCESS;
}

/////////////////////////////////////////////////////////////////////////////

xme_status_t
xme_core_broker_addDataPacketToFunctionItem
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_broker_functionDescriptionItem_t* functionItem,
    bool mandatory
)
{
    bool found;

    found = false;
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        functionItem->parameters,
        xme_core_broker_functionDataPacket_t,
        registeredDataPacket);

        if (dataPacketId == registeredDataPacket->dataPacketId)
        {
            found = true;
        }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(!found)
    {
        // create an entry for the new data packet
        xme_core_broker_functionDataPacket_t* functionParameter = (xme_core_broker_functionDataPacket_t *) xme_hal_mem_alloc(sizeof(xme_core_broker_functionDataPacket_t));
        functionParameter->dataPacketId = dataPacketId;

        if (mandatory)
        {
            functionParameter->usage = XME_CORE_BROKER_DATAPACKET_USAGE_REQUIRED;
        }
        else
        {
            functionParameter->usage = XME_CORE_BROKER_DATAPACKET_USAGE_OPTIONAL;
        }
        return xme_hal_singlyLinkedList_addItem(&functionItem->parameters, functionParameter);
    }
    else
    {
        return XME_STATUS_ALREADY_EXIST;
    }
}


bool
xme_core_broker_isDataPacketRegistered
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, false);

    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataSubscriberItem
    );
    {
        if (dataSubscriberItem->dataPacketId == dataPacketId)
        {
            return true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return false;
}

bool
xme_core_broker_isDataPacketAvailable
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataPacketItem
    );
    {
        if (dataPacketItem->dataPacketId == dataPacketId)
        {
            if (dataPacketItem->readiness == XME_CORE_BROKER_DATAPACKET_READINESS_READY)
            {
                return true;
            }
            else if (dataPacketItem->readiness == XME_CORE_BROKER_DATAPACKET_READINESS_UNAVAILABLE)
            {
                return false;
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return false; // in other case
}

xme_status_t
xme_core_broker_updateDataReadiness
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    uint8_t size
)
{
    XME_LOG(XME_LOG_DEBUG, "Broker: xme_core_broker_updateDataReadiness(%i, %i)\n", dataPacketId, size);

    // iterate over the data packet subscribers table
    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataSubscriberItem
    );
    {
        if (dataSubscriberItem->dataPacketId == dataPacketId)
        {
            if (size > 0)
            {
                dataSubscriberItem->readiness = XME_CORE_BROKER_DATAPACKET_READINESS_READY;
            }
            else
            {
                dataSubscriberItem->readiness = XME_CORE_BROKER_DATAPACKET_READINESS_UNAVAILABLE;
            }
        }

    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

bool
xme_core_broker_isOutputDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if(transferDataPacketItem->srcDataPacketId == dataPacketId)
        {
            return true;
        }
        else if (transferDataPacketItem->destDataPacketId == dataPacketId)
        {
            return false;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return false;
}

bool
xme_core_broker_isInputDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if(transferDataPacketItem->destDataPacketId == dataPacketId)
        {
            return true;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // we perform an additional check: if data packet is
    // registered as input parameter for a function,
    // we can ensure that it is an input data packet.
    if (xme_core_broker_isDataPacketRegistered(dataPacketId))
    {
        return true;
    }

    return false;
}

bool 
xme_core_broker_isSourceDataPacketLocked
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if(transferDataPacketItem->srcDataPacketId == dataPacketId)
        {
            return transferDataPacketItem->srcLocked;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return false;
}

xme_status_t 
xme_core_broker_unlockSourceDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        transferDataPacketTable, 
        xme_core_broker_transferDataPacketItem_t, 
        transferDataPacketItem);
    {
        if(transferDataPacketItem->srcDataPacketId == dataPacketId)
        {
            transferDataPacketItem->srcLocked = false;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */

