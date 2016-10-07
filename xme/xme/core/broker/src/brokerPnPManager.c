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
 * $Id: brokerPnPManager.c 7829 2014-03-14 10:29:33Z ruiz $
 */

/**
 * \file
 *         Broker interface for Plug and Play Manager.
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
#include "xme/core/broker/include/brokerPnpManagerInterface.h"

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
xme_core_broker_dataSubscriberTable_t dataPacketSubscribers;

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_core_broker_registerFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    xme_hal_table_rowHandle_t row;
    xme_core_broker_functionDescriptionItem_t* functionItem;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);

    // update function data needs
    if (!xme_core_broker_isFunctionVariantRegistered(componentId, functionId, functionVariantId))
    {
        // create a new item representing the function
        row = XME_HAL_TABLE_ADD_ITEM(functionDescriptions); // add a new element
        functionItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionDescriptions, row);
           XME_CHECK (NULL != functionItem, XME_STATUS_INVALID_HANDLE);
        functionItem->componentId = componentId;
        functionItem->functionId = functionId;
        functionItem->functionVariantId = functionVariantId;
        XME_HAL_SINGLYLINKEDLIST_INIT(functionItem->parameters);
        return XME_STATUS_SUCCESS;
    }
    else // function is already registered
    {
        return XME_STATUS_ALREADY_EXIST;
    }
}

xme_status_t
xme_core_broker_addDataPacketToFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    bool mandatory
)
{
    bool existingDataPacket;
    xme_core_broker_dataSubscriberItem_t* dataSubscriber;
    xme_core_broker_dataFunction_t* dataFunction;
    xme_hal_table_rowHandle_t row;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, XME_STATUS_INVALID_PARAMETER);

    // update function descriptions with specified data packet
    if (!xme_core_broker_isFunctionVariantRegistered(componentId, functionId, functionVariantId)) 
    {
        xme_core_broker_functionDescriptionItem_t* functionItem;

        row = XME_HAL_TABLE_ADD_ITEM(functionDescriptions); // add a new element
        functionItem = XME_HAL_TABLE_ITEM_FROM_HANDLE(functionDescriptions, row);
        XME_CHECK (NULL != functionItem, XME_STATUS_INVALID_HANDLE);
        functionItem->componentId = componentId;
        functionItem->functionId = functionId;
        functionItem->functionVariantId = functionVariantId;
        XME_HAL_SINGLYLINKEDLIST_INIT(functionItem->parameters);

        XME_CHECK(
            XME_STATUS_SUCCESS == xme_core_broker_addDataPacketToFunctionItem(dataPacketId, functionItem, mandatory), 
            XME_STATUS_INTERNAL_ERROR
        );
    }
    else
    {
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
                // We have found the functionItem. We add this data packet to the function item. 
                XME_CHECK(
                    XME_STATUS_SUCCESS == xme_core_broker_addDataPacketToFunctionItem(dataPacketId, functionItem, mandatory), 
                    XME_STATUS_ALREADY_EXIST
                );
            }
        }
        XME_HAL_TABLE_ITERATE_END();
    }

    // 2. update data packet table with this new information
    existingDataPacket = false;

    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        subscriberItem
    );
    {
        if (subscriberItem->dataPacketId == dataPacketId)
        {
            existingDataPacket = true;
            // the data packet is already registered. add the function to this item.
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                subscriberItem->subscribers,
                xme_core_broker_dataFunction_t,
                functionItem);
            {
                if (functionItem->componentId == componentId &&
                    functionItem->functionId == functionId &&
                    functionItem->functionVariantId == functionVariantId)
                {
                    // the function is already registered in data packets table
                    return XME_STATUS_ALREADY_EXIST;
                }
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

            // if not found any function matching the criteria, create a new item
            dataFunction = (xme_core_broker_dataFunction_t*) xme_hal_mem_alloc (sizeof(xme_core_broker_dataFunction_t));
            dataFunction->componentId = componentId;
            dataFunction->functionId = functionId;
            dataFunction->functionVariantId = functionVariantId;

            XME_CHECK(
                XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(
                &subscriberItem->subscribers,
                dataFunction), 
                XME_STATUS_INTERNAL_ERROR
            );
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    if (!existingDataPacket)
    { 
        // create a new entry in the data packet table
        row = XME_HAL_TABLE_ADD_ITEM(dataPacketSubscribers); // add a new element
        dataSubscriber = (xme_core_broker_dataSubscriberItem_t*) XME_HAL_TABLE_ITEM_FROM_HANDLE(dataPacketSubscribers, row);
           XME_CHECK (NULL != dataSubscriber, XME_STATUS_INVALID_HANDLE);
        dataSubscriber->dataPacketId = dataPacketId;
        dataSubscriber->readiness = XME_CORE_BROKER_DATAPACKET_READINESS_UNAVAILABLE;
        XME_HAL_SINGLYLINKEDLIST_INIT(dataSubscriber->subscribers);

        // add now the new function
        dataFunction = (xme_core_broker_dataFunction_t*) xme_hal_mem_alloc (sizeof(xme_core_broker_dataFunction_t));
        dataFunction->componentId = componentId;
        dataFunction->functionId = functionId;
        dataFunction->functionVariantId = functionVariantId;

        XME_CHECK(
            XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(
            &dataSubscriber->subscribers,
            dataFunction), 
            XME_STATUS_INTERNAL_ERROR
        );
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacketFromFunction
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId
)
{
    bool removed = false;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, XME_STATUS_INVALID_PARAMETER);

    // 1. check function description table
    // lookup for the function in function descriptions table
    if (xme_core_broker_isFunctionVariantRegistered(componentId, functionId, functionVariantId))
    {
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
                xme_core_broker_functionDataPacket_t* functionDataPacketToRemove = NULL;
                
                XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                    functionItem->parameters,
                    xme_core_broker_functionDataPacket_t,
                    functionDataPacket);
                {
                    if (functionDataPacket->dataPacketId == dataPacketId)
                    {
                        functionDataPacketToRemove = functionDataPacket;
                    }
                }
                XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
                
                XME_CHECK(NULL != functionDataPacketToRemove, XME_STATUS_NOT_FOUND);
                
                XME_CHECK(0 < xme_hal_singlyLinkedList_removeItem(
                    &functionItem->parameters,
                    functionDataPacketToRemove,
                    (bool) false),
                    XME_STATUS_INTERNAL_ERROR);
            }
        }
        XME_HAL_TABLE_ITERATE_END();
    }

    // 2. check data packet table
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
            xme_core_broker_dataFunction_t* dataFunctionToRemove = NULL;
            
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                dataSubscriberItem->subscribers,
                xme_core_broker_dataFunction_t,
                functionItem);
            {
                if (functionItem->componentId == componentId &&
                    functionItem->functionId == functionId &&
                    functionItem->functionVariantId == functionVariantId)
                {
                    dataFunctionToRemove = functionItem;
                    removed = true;
                }
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            
            if (dataFunctionToRemove != NULL)
            {
                XME_CHECK(0 < xme_hal_singlyLinkedList_removeItem(
                    &dataSubscriberItem->subscribers,
                    dataFunctionToRemove,
                    (bool) false),
                    XME_STATUS_INTERNAL_ERROR);

                // if it is the last item for that data, remove it from the datapacket table
                if (xme_hal_singlyLinkedList_getItemCount(&dataSubscriberItem->subscribers) == 0)
                {
                    XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_TABLE_REMOVE_ITEM(dataPacketSubscribers, handle), XME_STATUS_INTERNAL_ERROR);
                }
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    XME_CHECK(removed, XME_STATUS_NOT_FOUND);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeDataPacket
(
    xme_core_dataManager_dataPacketId_t dataPacketId
)
{
    bool removed = false;

    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, XME_STATUS_INVALID_PARAMETER);

    // 1. remove data packet information from data packet table
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
            XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_TABLE_REMOVE_ITEM(
                dataPacketSubscribers,
                handle), XME_STATUS_INTERNAL_ERROR);
            removed = true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // 2. remove the associated parameters from the function table
    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionItem
    );
    {
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            functionItem->parameters,
            xme_core_broker_functionDataPacket_t,
            dataPacketItem);
        {
            if (dataPacketItem->dataPacketId == dataPacketId)
            {
                XME_CHECK(0 != xme_hal_singlyLinkedList_removeItem(
                    &functionItem->parameters,
                    dataPacketItem,
                    (bool) false), 
                    XME_STATUS_INTERNAL_ERROR);
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }
    XME_HAL_TABLE_ITERATE_END();

    XME_CHECK(removed, XME_STATUS_NOT_FOUND);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeFunctionVariant
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{
    // fixme: looks like code duplication to previous fkt.
    bool removed = false;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);

    // 1. remove function description from functions table
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
            XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_TABLE_REMOVE_ITEM(
                functionDescriptions,
                handle), XME_STATUS_INTERNAL_ERROR);
            removed = true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // 2. remove the data packet associated to this function
    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataSubscriberItem
    );
    {
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            dataSubscriberItem->subscribers,
            xme_core_broker_dataFunction_t,
            functionItem);
        {
            if (functionItem->componentId == componentId &&
                functionItem->functionId == functionId &&
                functionItem->functionVariantId == functionVariantId)
            {
                XME_CHECK(0 != xme_hal_singlyLinkedList_removeItem(
                    &dataSubscriberItem->subscribers,
                    functionItem,
                    (bool) false), 
                    XME_STATUS_INTERNAL_ERROR);
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
    }
    XME_HAL_TABLE_ITERATE_END();

    if (!removed)
    {
        return XME_STATUS_NOT_FOUND;
    }

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_removeFunction
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    // fixme: looks like a code duplication too. maybe extract this into an own static fkt.
    bool removed = false; 

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);

    // 1. remove function description from functions table
    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionItem
    );
    {
        if (functionItem->componentId == componentId &&
            functionItem->functionId == functionId)
        {
            XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_TABLE_REMOVE_ITEM(
                functionDescriptions,
                handle), XME_STATUS_INTERNAL_ERROR);
            removed = true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    // 2. remove the data packet associated to this function
    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        dataSubscriberItem
    );
    {
        bool functionRemoved = false;
        XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            dataSubscriberItem->subscribers,
            xme_core_broker_dataFunction_t,
            functionItem);
        {
            if (functionItem->componentId == componentId &&
                functionItem->functionId == functionId)
            {
                XME_CHECK(0 != xme_hal_singlyLinkedList_removeItem(
                    &dataSubscriberItem->subscribers,
                    functionItem,
                    false), 
                    XME_STATUS_INTERNAL_ERROR);
                functionRemoved = true;
            }
        }
        XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
        
        XME_CHECK(functionRemoved, XME_STATUS_NOT_FOUND);
    }
    XME_HAL_TABLE_ITERATE_END();

    if (!removed)
    {
        return XME_STATUS_NOT_FOUND;
    }

    return XME_STATUS_SUCCESS;
}

bool
xme_core_broker_isFunctionRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId
)
{
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, false);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, false);

    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionItem
    );
    {
        if (functionItem->componentId == componentId &&
            functionItem->functionId == functionId)
        {
            return true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return false;
}

bool
xme_core_broker_isFunctionVariantRegistered
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    const xme_core_component_functionVariantId_t functionVariantId
)
{
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, false);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, false);

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
            return true;
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return false;
}

xme_status_t
xme_core_broker_getFunctionDataPackets
(
    xme_core_component_t componentId,
    xme_core_component_functionId_t functionId,
    xme_core_component_functionVariantId_t functionVariantId,
    xme_hal_linkedList_descriptor_t* dataPacketsList
)
{
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != dataPacketsList, XME_STATUS_INVALID_PARAMETER);


    if (!xme_core_broker_isFunctionVariantRegistered(componentId, functionId, functionVariantId))
    {
        return XME_STATUS_NOT_FOUND;
    }

    XME_HAL_TABLE_ITERATE_BEGIN(
        functionDescriptions,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_functionDescriptionItem_t,
        functionItem
    );
    {
        if (functionItem->componentId == componentId &&
            functionItem->functionId == functionId
            && functionItem->functionVariantId == functionVariantId)
        {
            if (xme_hal_singlyLinkedList_getItemCount(&(functionItem->parameters)) > 0)
            {
                XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                    functionItem->parameters,
                    xme_core_broker_functionDataPacket_t,
                    parameterItem);
                {
                    // add to the data packets table output parameter
                    XME_CHECK(
                        XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(dataPacketsList, &parameterItem->dataPacketId), 
                        XME_STATUS_INTERNAL_ERROR);
                }
                XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            }
            else
            {
                return XME_STATUS_NOT_FOUND;
            }
        }
    }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_broker_getDataPacketFunctions
(
    xme_core_dataManager_dataPacketId_t dataPacketId,
    xme_hal_linkedList_descriptor_t* functionsList
)
{
    XME_CHECK(XME_CORE_DATAMANAGER_DATAPACKETID_INVALID != dataPacketId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(NULL != functionsList, XME_STATUS_INVALID_PARAMETER);

    if (!xme_core_broker_isDataPacketRegistered(dataPacketId))
    {
        return XME_STATUS_NOT_FOUND;
    }

    XME_HAL_TABLE_ITERATE_BEGIN(
        dataPacketSubscribers,
        xme_hal_table_rowHandle_t,
        handle,
        xme_core_broker_dataSubscriberItem_t,
        subscriberItem
    );
        if (subscriberItem->dataPacketId == dataPacketId)
        {
            if (xme_hal_singlyLinkedList_getItemCount(&(subscriberItem->subscribers))> 0)
            {
                XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                    subscriberItem->subscribers,
                    xme_core_broker_dataFunction_t,
                    functionItem);
                {
                    // add to the data packets table output parameter
                    XME_CHECK(
                        XME_STATUS_SUCCESS == xme_hal_singlyLinkedList_addItem(functionsList, &functionItem->functionId),
                        XME_STATUS_INTERNAL_ERROR);
                }
                XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            }
            else
            {
                return XME_STATUS_NOT_FOUND;
            }
        }
    XME_HAL_TABLE_ITERATE_END();

    return XME_STATUS_SUCCESS;
}

/**
 * @}
 */

