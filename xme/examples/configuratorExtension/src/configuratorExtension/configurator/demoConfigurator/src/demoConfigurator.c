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
 * $Id: demoConfigurator.c 7766 2014-03-11 14:59:57Z wiesmueller $
 */

/**
 * \file
 *         Sample logical route configurator.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "configuratorExtension/configurator/demoConfigurator/include/demoConfigurator.h"

#include "configuratorExtension/topic/dictionary.h"
#include "configuratorExtension/topic/dictionaryData.h"

#include "xme/core/log.h"
#include "xme/core/manifestRepository/include/manifestRepository.h"
#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
/**
 * \brief Maximum number of items in route queue. This is used in
 *        overUpperConnectionBound to sort the routes before removal.
 */
#define ROUTE_QUEUE_MAX_SIZE 100

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \brief Simple map of edgeIDs to quality values.
 *
 * \details Index in edgeIds array matches index in qualityValues array.
 */
typedef struct
{
    xme_hal_graph_edgeId_t edgeIds[ROUTE_QUEUE_MAX_SIZE]; ///< Array of edgeIDs.
    configuratorExtension_attribute_quality_t qualityValues[ROUTE_QUEUE_MAX_SIZE]; ///< Array of qualityValues.
} edgeIdToQualityMap_t;

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
/**
 * \brief Sorts items by quality value.
 *
 * \details Items are expected to be of type xme_hal_graph_edgeId_t.
 *
 * \see xme_hal_linkedList_insertionCallback_t
 *
 * \param[in] item Item to be inserted into the linked list.
 * \param[in] currentItem Pointer to the existing item in the linked list
 *            currently being examined (starting with head).
 * \param[in] userData Pointer to edgeIdToQualityMap_t.
 *
 * \retval 0 quality is equal.
 * \retval 1 quality of inserted edge is higher.
 * \retval -1 quality of inserted edge is lower.
 */
int
routeQueueInsertionCallback
(
    const void* const item,
    const void* const currentItem,
    const void* userData
);

/**
 * \brief Called when connectionCount < lowerConnectionBound of the given input port.
 *        Will remove all incoming routes routes (which will prevent addition of that component IF THIS IS THE ONLY PORT of the component).
 *        Note: We do not remove the component here when it is already running.
 *
 * \param[in] logicalRouteGraph The logical route graph.
 * \param[in] logicalRouteGraph Vertex ID of the input port in the graph.
 * \param[in] vertexData Vertex data of the input port in the graph.
 */
void
belowLowerConnectionBound
(
    xme_hal_graph_graph_t* const logicalRouteGraph,
    xme_hal_graph_vertexId_t vertexId,
    const xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData
);

/**
 * \brief Called when connectionCount > upperConnectionBound of the given input port.
 *        Will remove incoming routes until the connectionCount matches the upperConnectionBound.
 *        Routes with low 'quality' attribute value will be removed first.
 *        Routes without this attributes are treated as having a quality of zero (the lowest quality).
 *
 * \param[in] logicalRouteGraph The logical route graph.
 * \param[in] logicalRouteGraph Vertex ID of the input port in the graph.
 * \param[in] vertexData Vertex data of the input port in the graph.
 * \param[in] upperConnectionBound Upper connection bound of the input port.
 * \param[in] connectionCount Current connection count of the input port.
 */
void
overUpperConnectionBound
(
    xme_hal_graph_graph_t* const logicalRouteGraph,
    xme_hal_graph_vertexId_t vertexId,
    const xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_component_connectionBound_t upperConnectionBound,
    uint16_t connectionCount
);

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
int
routeQueueInsertionCallback
(
    const void* const item,
    const void* const currentItem,
    const void* userData
)
{
    xme_hal_graph_edgeId_t* edgeId = (xme_hal_graph_edgeId_t*)item;
    xme_hal_graph_edgeId_t* currentEdgeId = (xme_hal_graph_edgeId_t*)currentItem;
    configuratorExtension_attribute_quality_t quality = 0U;
    bool qualitySet = false;
    configuratorExtension_attribute_quality_t currentQuality = 0U;
    bool currentQualitySet = false;
    edgeIdToQualityMap_t* edgeIdToQualityMap = (edgeIdToQualityMap_t*)userData;
    uint8_t i;

    for (i = 0; i < ROUTE_QUEUE_MAX_SIZE; i++)
    {
        if (edgeIdToQualityMap->edgeIds[i] == *edgeId)
        {
            quality = edgeIdToQualityMap->qualityValues[i];
            qualitySet = true;
        }
        else if (edgeIdToQualityMap->edgeIds[i] == *currentEdgeId)
        {
            currentQuality = edgeIdToQualityMap->qualityValues[i];
            currentQualitySet = true;
        }
    }

    XME_LOG_IF
    (
        !qualitySet || !currentQualitySet,
        XME_LOG_WARNING, "Sorting failed due erroneous edge to quality map!\n"
    );

    if (quality < currentQuality)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

void
configuratorExtension_configurator_demoConfigurator_callback
(
    xme_hal_graph_graph_t* const logicalRouteGraph
)
{
    xme_status_t status;

    // Iterate over all ports in the logical route graph
    xme_hal_graph_initVertexIterator(logicalRouteGraph);
    while (xme_hal_graph_hasNextVertex(logicalRouteGraph))
    {
        xme_hal_graph_vertexId_t vertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* vertexData;

        vertexId = xme_hal_graph_nextVertex(logicalRouteGraph);
        xme_hal_graph_getVertexData(logicalRouteGraph, vertexId, (void**)&vertexData);

        // If vertex is component port
        if (XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vertexData->vertexType)
        {
            xme_core_componentManifest_t componentManifest;
            size_t constraintLen;

            // Print component constraint string if non-empty
            status = xme_core_manifestRepository_findComponentManifest(vertexData->vertexData.componentPortVertex.componentType, &componentManifest);
            XME_ASSERT_NORVAL(XME_STATUS_SUCCESS == status);
            constraintLen = xme_hal_safeString_strnlen(componentManifest.constraint, sizeof(componentManifest.constraint));
            if (0 < constraintLen)
            {
                XME_LOG(XME_LOG_NOTE,
                    "[demoConfigurator] Component type %d constraint string '%s'\n",
                    vertexData->vertexData.componentPortVertex.componentType, componentManifest.constraint);
            }

            // If vertex represents input port
            if(XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION == vertexData->vertexData.componentPortVertex.portType ||
               XME_CORE_COMPONENT_PORTTYPE_RR_REQUEST_HANDLER == vertexData->vertexData.componentPortVertex.portType ||
               XME_CORE_COMPONENT_PORTTYPE_RR_RESPONSE_HANDLER == vertexData->vertexData.componentPortVertex.portType)
            {
                xme_core_component_connectionBound_t lowerConnectionBound; // Lower connection bound of the current input port
                xme_core_component_connectionBound_t upperConnectionBound; // Upper connection bound of the current input port
                uint16_t connectionCount = 0; // Will count the number of incoming routes to this input port

                lowerConnectionBound = vertexData->vertexData.componentPortVertex.lowerConnectionBound;
                upperConnectionBound = vertexData->vertexData.componentPortVertex.upperConnectionBound;

                XME_ASSERT_NORVAL(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID != lowerConnectionBound);
                XME_ASSERT_NORVAL(XME_CORE_COMPONENT_CONNECTIONBOUND_INVALID != upperConnectionBound);

                // Iterate over logical routes and determine the number of incoming routes
                xme_hal_graph_initIncomingEdgeIterator(logicalRouteGraph, vertexId);
                while (xme_hal_graph_hasNextIncomingEdge(logicalRouteGraph, vertexId))
                {
                    xme_hal_graph_nextIncomingEdge(logicalRouteGraph, vertexId);

                    connectionCount++;
                }
                xme_hal_graph_finiIncomingEdgeIterator(logicalRouteGraph, vertexId);

                if (connectionCount < lowerConnectionBound)
                {
                    belowLowerConnectionBound(logicalRouteGraph, vertexId, vertexData);
                }
                else if (upperConnectionBound != (xme_core_component_connectionBound_t)XME_CORE_COMPONENT_CONNECTIONBOUND_UNBOUNDED &&
                         connectionCount > upperConnectionBound)
                {
                    overUpperConnectionBound(logicalRouteGraph, vertexId, vertexData, upperConnectionBound, connectionCount);
                }
            }
        }
    }
    xme_hal_graph_finiVertexIterator(logicalRouteGraph);
}

void
belowLowerConnectionBound
(
    xme_hal_graph_graph_t* const logicalRouteGraph,
    xme_hal_graph_vertexId_t vertexId,
    const xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData
)
{
    XME_LOG
    (
        XME_LOG_NOTE,
        "[demoConfigurator] Minimum connection bound not satisfied on compID %d, compType %d, port %d. Removing all routes.\n",
        vertexData->vertexData.componentPortVertex.componentId,
        vertexData->vertexData.componentPortVertex.componentType,
        vertexData->vertexData.componentPortVertex.portIndex
    );

    // Remove all routes to this port
    xme_hal_graph_initIncomingEdgeIterator(logicalRouteGraph, vertexId);
    while (xme_hal_graph_hasNextIncomingEdge(logicalRouteGraph, vertexId))
    {
        xme_hal_graph_edgeId_t edgeId;

        edgeId = xme_hal_graph_nextIncomingEdge(logicalRouteGraph, vertexId);

        xme_core_pnp_configExt_removeLink(edgeId);
    }
    xme_hal_graph_finiIncomingEdgeIterator(logicalRouteGraph, vertexId);
}

void
overUpperConnectionBound
(
    xme_hal_graph_graph_t* const logicalRouteGraph,
    xme_hal_graph_vertexId_t vertexId,
    const xme_core_pnp_dataLinkGraph_vertexData_t* const vertexData,
    xme_core_component_connectionBound_t upperConnectionBound,
    uint16_t connectionCount
)
{
    xme_hal_singlyLinkedList_t(ROUTE_QUEUE_MAX_SIZE) queue; // Sorted queue of all incoming routes. Sorted by 'quality' attribute value. Lower quality first.
    edgeIdToQualityMap_t edgeIdToQualityMap;
    uint8_t i = 0;
    xme_hal_linkedList_index_t queueIndex;

    XME_LOG
    (
        XME_LOG_NOTE,
        "[demoConfigurator] Maximum connection bound exceeded on compID %d, compType %d, port %d.\n",
        vertexData->vertexData.componentPortVertex.componentId,
        vertexData->vertexData.componentPortVertex.componentType,
        vertexData->vertexData.componentPortVertex.portIndex
    );

    XME_HAL_SINGLYLINKEDLIST_INIT(queue);

    // Sort routes by quality attribute
    xme_hal_graph_initIncomingEdgeIterator(logicalRouteGraph, vertexId);
    while (i < ROUTE_QUEUE_MAX_SIZE && xme_hal_graph_hasNextIncomingEdge(logicalRouteGraph, vertexId))
    {
        xme_status_t status = XME_STATUS_INTERNAL_ERROR;
        xme_hal_graph_vertexId_t srcVertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* srcVertexData;
        xme_core_componentManifest_t componentManifest;

        edgeIdToQualityMap.edgeIds[i] = xme_hal_graph_nextIncomingEdge(logicalRouteGraph, vertexId);

        srcVertexId = xme_hal_graph_getSourceVertex(logicalRouteGraph, edgeIdToQualityMap.edgeIds[i]);

        xme_hal_graph_getVertexData(logicalRouteGraph, srcVertexId, (void**)&srcVertexData);

        status = xme_core_manifestRepository_findComponentManifest
        (
            srcVertexData->vertexData.componentPortVertex.componentType,
            &componentManifest
        );
        if (status != XME_STATUS_SUCCESS)
        {
            edgeIdToQualityMap.qualityValues[i] = 0;
        }
        else
        {
            xme_core_directory_attributeSetHandle_t attrSetHandle =
                    componentManifest.portManifests[srcVertexData->vertexData.componentPortVertex.portIndex].attrSet;
                    
            status = xme_core_directory_attribute_getAttributeValue
            (
                attrSetHandle,
                (xme_core_attribute_key_t)CONFIGURATOREXTENSION_ATTRIBUTE_QUALITY,
                &edgeIdToQualityMap.qualityValues[i],
                sizeof(configuratorExtension_attribute_quality_t),
                NULL
            );

            if (status != XME_STATUS_SUCCESS)
            {
                edgeIdToQualityMap.qualityValues[i] = 0;
            }
        }

        status = xme_hal_singlyLinkedList_addItemOrdered(
            &queue, &edgeIdToQualityMap.edgeIds[i], routeQueueInsertionCallback, &edgeIdToQualityMap
        );
        if (XME_STATUS_SUCCESS != status)
        {
            XME_LOG(XME_LOG_ERROR, "[demoConfigurator] Error adding route to queue.\n");
        }

        i++;
    }
    xme_hal_graph_finiIncomingEdgeIterator(logicalRouteGraph, vertexId);

    // Iterate over queue and remove routes (beginning with lowest quality) until upper connection bound is satisfied
    queueIndex = xme_hal_singlyLinkedList_getItemCount(&queue);
    for (i = 0U; i < queueIndex && connectionCount > upperConnectionBound; i++)
    {
        xme_hal_graph_edgeId_t* edgeId = (xme_hal_graph_edgeId_t*) xme_hal_singlyLinkedList_itemFromIndex(&queue, i);
        xme_hal_graph_vertexId_t srcVertexId;
        xme_core_pnp_dataLinkGraph_vertexData_t* srcVertexData;
        configuratorExtension_attribute_quality_t quality = 0U;
        bool qualitySet = false;
        
        xme_core_pnp_configExt_removeLink(*edgeId);
        connectionCount--;

        srcVertexId = xme_hal_graph_getSourceVertex(logicalRouteGraph, *edgeId);

        xme_hal_graph_getVertexData(logicalRouteGraph, srcVertexId, (void**)&srcVertexData);

        // Get quality for log message
        {
            uint8_t k = 0;
            for (k = 0; k < ROUTE_QUEUE_MAX_SIZE; k++)
            {
                if (edgeIdToQualityMap.edgeIds[k] == *edgeId)
                {
                    quality = edgeIdToQualityMap.qualityValues[k];
                    qualitySet = true;
                    break;
                }
            }
        }

        XME_ASSERT_NORVAL(qualitySet);

        XME_LOG
        (
            XME_LOG_NOTE,
            "[demoConfigurator] Removed route from compID %d, compType %d, port %d, quality %d.\n",
            srcVertexData->vertexData.componentPortVertex.componentId,
            srcVertexData->vertexData.componentPortVertex.componentType,
            srcVertexData->vertexData.componentPortVertex.portIndex,
            quality
        );
    }
}
