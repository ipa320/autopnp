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
 * $Id: logicalRouteManager.c 5208 2013-09-27 12:13:27Z ruiz $
 */

/**
 * \file
 *         Logical Route Manager.
 */

/**
 * \addtogroup core_pnp_lrm
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/plugAndPlay/include/logicalRouteManager.h"

#include "xme/core/plugAndPlay/include/dataLinkGraph.h"

#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/mem.h"

/******************************************************************************/
/***   Type Definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_pnp_lrm_state_t
 *
 * \brief Represents the internal state of the logical route manager. 
 * \details This data is used internally to store the corresponding routes, publications
 *          and subscriptions. Additionally, it is stored the maximum transaction id and
 *          the maximum channel id. 
 */
typedef struct 
{
    bool initialized; ///< determines if the state is initialized. 
    // TODO: Move static size to Options.cmake
    xme_hal_singlyLinkedList_t(10) graphs; ///< a linked list of state subgraphs. 
    xme_core_transactionId_t maxTransactionId; ///< the maximum assigned transaction id. 
    xme_core_channelId_t maxChannelId; ///< the maximum assigned channel id. 
} xme_core_pnp_lrm_state_t;

/**
 * \struct xme_core_pnp_lrm_stateSubGraph_t
 *
 * \brief This subgraph stores all internally generated graphs.
 * \details This includes, at least the topic id, publications, subscriptions,
 *          generated routes, and associated transaction id. 
 */
typedef struct
{
    xme_core_topic_t topicId; ///< the topic id. 
    xme_hal_linkedList_descriptor_t publications; ///< the linked list containing the set of publications. 
    xme_hal_linkedList_descriptor_t subscriptions; ///< the linked list containing the set of subscriptions. 
    xme_core_pnp_lrm_logicalRoutes_t routes; ///< the generated routes. 
    xme_core_transactionId_t transactionId; ///< the associated transaction id. 
} xme_core_pnp_lrm_stateSubGraph_t;


/******************************************************************************/
/***   Static Variables                                                     ***/
/******************************************************************************/

/**
 * \brief The static logial route manager state. 
 */
static xme_core_pnp_lrm_state_t xme_core_pnp_lrm_state;


/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief Creates a new state subgraph. 
 *
 * \param[out] subGraph the generated empty subgraph. 
 *
 * \retval XME_STATUS_SUCCESS if the subgraph is successfully generated. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if the subgraph cannot be generated in the 
 *         logical route manager.
 */
xme_status_t 
xme_core_pnp_lrm_createSubGraph
(
    xme_core_pnp_lrm_stateSubGraph_t** subGraph
);

/**
 * \brief Frees resources from a state subgraph. 
 *
 * \param subGraph the state subgraph to be freed. 
 *
 * \retval XME_STATUS_SUCCESS if subgraph resources has been sucessfully freed. 
 * \retval XME_STATUS_OUT_OF_RESOURCES if the subgraph cannot be freed in the 
 *         logical route manager.
 */
xme_status_t 
xme_core_pnp_lrm_removeSubGraph
(
    xme_core_pnp_lrm_stateSubGraph_t** subGraph
);

/**
 * \brief Boolean function that determine if a given announcement already exists. 
 *
 * \param[in] announcementList the list of internal announcements already published. 
 * \param[in] announcement the announcement to compare. 
 *
 * \retval true if the announcement is already part of the announcement list. 
 * \retval false if the announcement does not exist on the announcement list.
 */
bool xme_core_pnp_lrm_announcementExists
(
    xme_hal_linkedList_descriptor_t announcementList, 
    xme_core_pnp_lrm_announcement_t* announcement
);

/**
 * \brief A comparison function to compare two announcements.
 *
 * \param announcement1 the first announcement to compare. 
 * \param announcement2 the second announcement to compare. 
 *
 * \return an integer value stating if the first announcement is
 *         lower, equal or greater than the second announcement. 
 */
int 
xme_core_pnp_lrm_compareAnnouncement
(
    xme_core_pnp_lrm_announcement_t* announcement1, 
    xme_core_pnp_lrm_announcement_t* announcement2
);

/**
 * \brief Adds a component port vertex to a given graph. 
 *
 * \param graph The graph in which is inserted the component port vertex. 
 * \param componentPortVertex the component port vertex to insert in the graph. 
 *
 * \retval XME_STATUS_SUCCESS if the component port vertex has been sucessfully added
 *         to the graph. 
 * \retval XME_STATUS_INVALID_PARAMETER if input parameters are not valid.
 * \retval XME_STATUS_OUT_OF_RESOURCES if cannot add the component port vertex to the
 *         graph due to lack of resources. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot generate a valid component port vertex
 *         to the graph. 
 */
xme_status_t 
xme_core_pnp_lrm_addComponentPortVertex
(
    xme_hal_graph_graph_t* graph, 
    xme_core_pnp_dataLinkGraph_vertexData_t** componentPortVertex
);

/**
 * \brief Removes a component port vertex from the given graph. 
 *
 * \param graph the graph. 
 * \param vertexId the vertex identifier to remove. 
 *
 * \retval XME_STATUS_SUCCESS if the vertex has been removed from the graph. 
 * \retval XME_STATUS_INVALID_PARAMETER if input parameters are invalid. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot remove the given vertex from the graph. 
 */
xme_status_t 
xme_core_pnp_lrm_removeComponentPortVertex
(
    xme_hal_graph_graph_t* graph, 
    xme_hal_graph_vertexId_t vertexId
);

/**
 * \brief Function that implements the comparison of two vertices
 *        in a given graph. 
 * \note Comparison affects only at component port level. 
 *
 * \param vertexData1 the first vertex data to compare. 
 * \param vertexData2 the second vertex data to compare. 
 *
 * \return an int value stating that the first vertex data is
 *         lower, equal or greater than the second vertex data. 
 */
int
xme_core_pnp_lrm_vertexCompareCallbackSameComponentPort
(
    void* vertexData1,
    void* vertexData2
);

/**
 * \brief Function that implements the comparison of two edges
 *        in a given graph. 
 * \note Comparison affects only to established logical routes. 
 *
 * \param edgeData1 the first edge data to compare. 
 * \param edgeData2 the second edge data to compare. 
 *
 * \return an int value stating that the first edge data is
 *         lower, equal or greater than the second edge data. 
 */
int
xme_core_pnp_lrm_edgeCompareCallbackLogicalRouteNotEstablished
(
    void* edgeData1,
    void* edgeData2
);

/**
 * \brief Obtains the first edge data where source and
 *        sink are the same as provided input parameters. 
 *
 * \param[in] sourceVertexData the source vertex data. 
 * \param[in] sinkVertexData the sink vertex data. 
 *
 * \return the edge data of a data link graph containing
 *         the first occurrence of the edge data associating
 *         source and sink vertex data. If there is no edges linking
 *         source and sink vertices, NULL value is returned instead. 
 */
xme_core_pnp_dataLinkGraph_edgeData_t*
xme_core_pnp_lrm_findEstablishedLogicalRoute
(
    xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertexData,
    xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData
);

/**
 * \brief Adds a logical route edge to the input graph.
 *
 * \param[in] graph the incoming graph to add the logical route edge. 
 * \param[out] logicalRouteEdge the newly generated logical route edge. 
 * \param[in] sourceVertexId the vertex identifier of source component port vertex.
 * \param[in] sinkVertexId the vertex identifier of sink component port vertex.
 *
 * \retval XME_STATUS_SUCCESS if logical route edge has been added to the graph. 
 * \retval XME_STATUS_INVALID_PARAMETER if input parameters are not valid.
 * \retval XME_STATUS_OUT_OF_RESOURCES if cannot add the edge to the
 *         graph due to lack of resources. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot generate a valid logical route edge
 *         to the graph. 
 */
xme_status_t
xme_core_pnp_lrm_addLogicalRouteEdge
(
    xme_hal_graph_graph_t* graph,
    xme_core_pnp_dataLinkGraph_edgeData_t** logicalRouteEdge,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId
);

/**
 * \brief Removes a logical route edge from the given graph. 
 *
 * \param graph the graph. 
 * \param edgeId the edge identifier to remove. 
 *
 * \retval XME_STATUS_SUCCESS if the edge has been removed from the graph. 
 * \retval XME_STATUS_INVALID_PARAMETER if input parameters are invalid. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot remove the given edge from the graph. 
 */
xme_status_t 
xme_core_pnp_lrm_removeLogicalRouteEdge
(
    xme_hal_graph_graph_t* graph, 
    xme_hal_graph_edgeId_t edgeId
);

/**
 * \brief Function that implements the comparison of two edges
 *        in a given graph. 
 * \note Comparison affects only at logical routes. 
 *
 * \param edgeData1 the first edge data to compare. 
 * \param edgeData2 the second edge data to compare. 
 *
 * \return an int value stating that the first edge data is
 *         lower, equal or greater than the second edge data. 
 */
int 
xme_core_pnp_lrm_compareLogicalRouteEdge
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData1, 
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData2
);

/**
 * \brief Function that implements the comparison of two vertices
 *        in a given graph. 
 * \note Comparison affects only at component port vertex level. 
 *
 * \param vertexData1 the first vertex data to compare. 
 * \param vertexData2 the second vertex data to compare. 
 * \param portType the port type to compare. 
 *
 * \return an int value stating that the first vertex data is
 *         lower, equal or greater than the second vertex data. 
 */
int 
xme_core_pnp_lrm_compareComponentPortVertex
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData1, 
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData2,
    xme_core_component_portType_t portType
);

/**
 * \brief Checks if two announcement matches.
 * \details Two announcement matches when source announcement publishes
 *          some topic (with associated attributes) and the sink announcement
 *          is waiting for exactly that matching publication. 
 * \param sourceAnnouncement the source announcement.
 * \param sinkAnnouncement the sink announcement. 
 *
 * \retval true if source announcement matchs to sink announcement. 
 * \retval false if source announcement does not match to sink announcement. 
 */
bool xme_core_pnp_lrm_matchAnnouncement
(
    xme_core_pnp_lrm_announcement_t* sourceAnnouncement, 
    xme_core_pnp_lrm_announcement_t* sinkAnnouncement
);

/**
 * \brief Matches announcement attributes. 
 * \details Attributes are metadata associated to the announcement. 
 *          If all attributes in the publication matches the filter in 
 *          subscription for the same key in attribute, the return 
 *          value is true. In other case, the value is false. 
 * \note Attribute list on subscription side can be empty. In this case,
 *       the result is true. 
 * \param sourceAnnouncement the publication announcement, in which is
 *        contained the attribute definitions. 
 * \param sinkAnnouncement the subscription announcement, in which is 
 *        defined the attribute filters. 
 *
 * \retval true if the source announcement attribute definitions match
 *         with the sink announcement attribute filters. 
 * \retval false if there is at least one non-matching attribute
 *         between source announcement attribute definitions and
 *         sink announcement attribute filters.
 */
bool
xme_core_pnp_lrm_matchAttributes
(
    xme_core_pnp_lrm_announcement_t* sourceAnnouncement, 
    xme_core_pnp_lrm_announcement_t* sinkAnnouncement
);

//// FIXME: Export? Do we need that?
//int xme_core_pnp_lrm_compare_topic(xme_core_topic_t* topic1, xme_core_topic_t* topic2);
//
//// FIXME: Export?
//int xme_core_pnp_lrm_compare_channelId(xme_core_channelId_t* channelId1, xme_core_channelId_t* channelId2);
//
//// FIXME: Export?
//int xme_core_pnp_lrm_compare_component(xme_core_component_t* component1, xme_core_component_t* component2);
//
//// FIXME: Export?
//int xme_core_pnp_lrm_compare_containerVertexData(xme_core_pnp_lrm_announcement_t* announcement1, xme_core_pnp_lrm_announcement_t* announcement2);
//
//// FIXME: Export?
//int xme_core_pnp_lrm_compare_logicalRouteEdgeData(xme_core_pnp_lrm_announcement_t* announcement1, xme_core_pnp_lrm_announcement_t* announcement2);


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

xme_status_t
xme_core_pnp_lrm_init(
    void *params
)
{
    XME_UNUSED_PARAMETER(params);

    // Forget previous configuration (required for independence of test cases)
    // TODO: Solve in a clean way. If a test intiailizes the LRM, then it should also finalize it!
    //       As such, the correct statement here is: XME_ASSERT(!xme_core_pnp_lrm_state.initialized);
    if (xme_core_pnp_lrm_state.initialized)
    {
        xme_core_pnp_lrm_fini();
    }

    XME_HAL_SINGLYLINKEDLIST_INIT(xme_core_pnp_lrm_state.graphs);

    xme_core_pnp_lrm_state.initialized = true;

    return XME_STATUS_SUCCESS;
}

void
xme_core_pnp_lrm_fini(void)
{
    XME_ASSERT_NORVAL(xme_core_pnp_lrm_state.initialized);

    // Forget previous configuration and free memory (required for independence of test cases)
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        xme_core_pnp_lrm_state.graphs, 
        xme_core_pnp_lrm_stateSubGraph_t, 
        stateSubgraph
    );
    {
        (void) xme_core_pnp_lrm_removeSubGraph(&stateSubgraph);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_HAL_SINGLYLINKEDLIST_FINI(xme_core_pnp_lrm_state.graphs);


    xme_core_pnp_lrm_state.maxTransactionId = XME_CORE_INVALID_TRANSACTION_ID;
    xme_core_pnp_lrm_state.maxChannelId = XME_CORE_INVALID_CHANNEL_ID;

    xme_core_pnp_lrm_state.initialized = false;
}

/*
 * Maybe, we need an additional parameter attrList for specifying required
 * attributes of a response (only required for request ports)
 */
xme_status_t
xme_core_pnp_lrm_announcePort
(
    xme_core_node_nodeId_t nodeId,
    xme_core_component_t componentId,
    xme_core_component_portType_t portType,
    uint16_t portIndex,
    xme_core_topic_t topicId,
    xme_core_directory_attributeSetHandle_t attrSet,
    xme_core_transactionId_t transactionId
)
{
    xme_core_pnp_lrm_announcement_t* newAnnouncement;
    xme_core_pnp_lrm_stateSubGraph_t* subGraph;

    XME_CHECK(XME_CORE_NODE_INVALID_NODE_ID != nodeId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_COMPONENT_PORTTYPE_INVALID != portType, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(XME_CORE_TOPIC_INVALID_TOPIC != topicId, XME_STATUS_INVALID_PARAMETER);
    
    subGraph = NULL;

    // Search for the subgraph with the same topicId.
    // TODO: Add attribute support capability as soon as it is scheduled.
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        xme_core_pnp_lrm_state.graphs, 
        xme_core_pnp_lrm_stateSubGraph_t, 
        stateSubgraph);
    {
        if (stateSubgraph->topicId == topicId)
        {
            subGraph = stateSubgraph;
            break;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // If it does not exists, create a new subgraph and add it to the internal state. 
    if (subGraph == NULL)
    {
        XME_CHECK(xme_core_pnp_lrm_createSubGraph(&subGraph) == XME_STATUS_SUCCESS, XME_STATUS_INTERNAL_ERROR);

        subGraph->topicId = topicId;
        subGraph->transactionId = transactionId;
    }

    XME_ASSERT(subGraph != NULL);


    /* //FIXME:Assumptions: 
     * - All announcements will be kept forever unless they are explicitly rolled back
     * - If an announcement already exists from an earlier transaction, the older version will be used. (Ok?)
     */ 
    newAnnouncement = (xme_core_pnp_lrm_announcement_t*)xme_hal_mem_alloc(sizeof(xme_core_pnp_lrm_announcement_t));
        
    XME_CHECK(newAnnouncement != NULL, XME_STATUS_OUT_OF_RESOURCES);

    newAnnouncement->nodeId = nodeId; 
    newAnnouncement->componentId = componentId;
    newAnnouncement->portType = portType;
    newAnnouncement->topicId = topicId;
    newAnnouncement->portIndex = portIndex;
    newAnnouncement->attrSet = attrSet;
    newAnnouncement->transactionId = transactionId;

    // Find a similar announcement in publications and subscriptions list. 
    if (portType == XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION)
    {
        XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(subGraph->publications, newAnnouncement), XME_STATUS_INTERNAL_ERROR);
    }
    else if (portType == XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION)
    {
        XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(subGraph->subscriptions, newAnnouncement), XME_STATUS_INTERNAL_ERROR);
    }

    subGraph->transactionId = transactionId;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_lrm_replaceContainerId
(
    xme_core_node_nodeId_t oldNodeId,
    xme_core_container_t oldContainerId,
    xme_core_node_nodeId_t newNodeId,
    xme_core_container_t newContainerId,
    xme_core_transactionId_t transactionId
)
{
    XME_UNUSED_PARAMETER(oldNodeId);
    XME_UNUSED_PARAMETER(oldContainerId);
    XME_UNUSED_PARAMETER(newNodeId);
    XME_UNUSED_PARAMETER(newContainerId);
    XME_UNUSED_PARAMETER(transactionId);

    //TODO: Find all affected channels and announcements and modify them

    return XME_STATUS_SUCCESS;
}

/*
 * This function will not do anything because channelIds will be globally unique
 */
xme_status_t
xme_core_pnp_lrm_discardChannelId
(
    xme_core_channelId_t channelId
)
{
    XME_UNUSED_PARAMETER(channelId);

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_lrm_generateChannelId
(
    xme_core_transactionId_t transactionId,
    xme_core_channelId_t* outChannelId
)
{
    /*
    The support of both channelId and transactionId is related with commit and rollback functions (currently developed at dataHandler level).

    The channel id will be generated associated to a given logical route. In case that the route is not finally established, the corresponding channelId generation is rolled back. Just for that unique purpose, and for maintaining traceability of operations in Chromosome, we should maintain transactionId as part of the current LRM interface and implementation, even if we do not use it at all.
    */
    XME_UNUSED_PARAMETER(transactionId);

    XME_CHECK(NULL != outChannelId, XME_STATUS_INVALID_PARAMETER);

    xme_core_pnp_lrm_state.maxChannelId = (xme_core_channelId_t)(xme_core_pnp_lrm_state.maxChannelId + 1);

    *outChannelId = xme_core_pnp_lrm_state.maxChannelId;

    //TODO: Define the valididy of a channel ID. (per transaction? globally?)
    //TODO: Check for maximum Value
    return (XME_STATUS_SUCCESS);
}

xme_status_t
xme_core_pnp_lrm_getLogicalRoutes
(
    xme_core_transactionId_t transactionId,
    xme_core_pnp_lrm_logicalRoutes_t* outRouteCandidates
)
{
    xme_hal_linkedList_descriptor_t publicationVertices;
    xme_hal_linkedList_descriptor_t subscriptionVertices;
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexPtr;
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexToUse;
    xme_core_pnp_dataLinkGraph_edgeData_t* edgePtr;
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeToUse;
    xme_core_channelId_t channelId;
    xme_hal_graph_vertexId_t sourceVertexId;
    xme_hal_graph_vertexId_t sinkVertexId;

    XME_CHECK(NULL != outRouteCandidates, XME_STATUS_INVALID_PARAMETER);

    // check for validity of t'Id: XME_CHECK(transactionId == ...
    XME_CHECK(xme_hal_graph_init((xme_core_pnp_lrm_logicalRoutes_t*)outRouteCandidates) == XME_STATUS_SUCCESS, XME_STATUS_INTERNAL_ERROR);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_pnp_lrm_state.graphs, xme_core_pnp_lrm_stateSubGraph_t, subGraph);
    {
        if (subGraph->transactionId == transactionId)
        {
            XME_HAL_SINGLYLINKEDLIST_INIT(publicationVertices);
            XME_HAL_SINGLYLINKEDLIST_INIT(subscriptionVertices);
            
            /*
             * How it works:
             * 
             * 1. Update local route Graph elements
             * 2. Update local route status
             * 3. COPY routes to outRouteCandidates
             */

            // 1. Update graph elements

            // 1a. subscriptions
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN
            (
                subGraph->subscriptions, 
                xme_core_pnp_lrm_announcement_t, 
                subscription
            );
            {
                xme_status_t status;
                vertexToUse = NULL;

                status = xme_hal_graph_initVertexIterator(&subGraph->routes);
                XME_ASSERT(XME_STATUS_SUCCESS == status);
                while(xme_hal_graph_hasNextVertex(&subGraph->routes))
                {        
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(&subGraph->routes, xme_hal_graph_nextVertex(&subGraph->routes), (void**)&vertexPtr), XME_STATUS_INTERNAL_ERROR);

                    if (vertexPtr->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT &&
                        vertexPtr->topicId == subscription->topicId &&
                        vertexPtr->vertexData.componentPortVertex.componentId == subscription->componentId &&
                        vertexPtr->nodeId == subscription->nodeId &&
                        vertexPtr->vertexData.componentPortVertex.portIndex == subscription->portIndex)
                    {
                        vertexToUse = vertexPtr;
                    }

                }
                status = xme_hal_graph_finiVertexIterator(&subGraph->routes);
                XME_ASSERT(XME_STATUS_SUCCESS == status);

                if (vertexToUse == NULL) {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addComponentPortVertex(&subGraph->routes, &vertexToUse), XME_STATUS_INTERNAL_ERROR);

                    vertexToUse->nodeId = subscription->nodeId;
                    vertexToUse->topicId = subscription->topicId;
                    vertexToUse->vertexData.componentPortVertex.portIndex = subscription->portIndex;
                    vertexToUse->vertexData.componentPortVertex.componentId = subscription->componentId;
                    vertexToUse->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
                    vertexToUse->vertexData.componentPortVertex.announcement = subscription;
                }

                XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(subscriptionVertices, vertexToUse), XME_STATUS_INTERNAL_ERROR);

                // 3. clone and add to outRouteCandidates
                vertexPtr = NULL;
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addComponentPortVertex(outRouteCandidates, &vertexPtr), XME_STATUS_INTERNAL_ERROR);

                XME_CHECK(vertexPtr != NULL, XME_STATUS_OUT_OF_RESOURCES);

                vertexPtr->nodeId = vertexToUse->nodeId;
                vertexPtr->topicId = vertexToUse->topicId;
                vertexPtr->vertexData.componentPortVertex.componentId = vertexToUse->vertexData.componentPortVertex.componentId;
                vertexPtr->vertexData.componentPortVertex.portType = vertexToUse->vertexData.componentPortVertex.portType;
                vertexPtr->vertexData.componentPortVertex.portIndex = vertexToUse->vertexData.componentPortVertex.portIndex;
                vertexPtr->vertexData.componentPortVertex.announcement = vertexToUse->vertexData.componentPortVertex.announcement;

            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();


            // 1b. publications
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(subGraph->publications, xme_core_pnp_lrm_announcement_t, publication);
            {
                xme_status_t status;
                vertexToUse = NULL;

                status = xme_hal_graph_initVertexIterator(&subGraph->routes);
                XME_ASSERT(XME_STATUS_SUCCESS == status);
                while(xme_hal_graph_hasNextVertex(&subGraph->routes))
                {        
                    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(&subGraph->routes, xme_hal_graph_nextVertex(&subGraph->routes), (void**)&vertexPtr), XME_STATUS_INTERNAL_ERROR);

                    if (vertexPtr->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT &&
                        vertexPtr->topicId == publication->topicId &&
                        vertexPtr->vertexData.componentPortVertex.componentId == publication->componentId &&
                        vertexPtr->nodeId == publication->nodeId &&
                         vertexPtr->vertexData.componentPortVertex.portIndex == publication->portIndex)
                    {
                        vertexToUse = vertexPtr;
                    }

                }
                status = xme_hal_graph_finiVertexIterator(&subGraph->routes);
                XME_ASSERT(XME_STATUS_SUCCESS == status);

                if (vertexToUse == NULL) {
                    XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addComponentPortVertex(&subGraph->routes, &vertexToUse), XME_STATUS_INTERNAL_ERROR);

                    vertexToUse->nodeId = publication->nodeId;
                    vertexToUse->topicId = publication->topicId;
                    vertexToUse->vertexData.componentPortVertex.portIndex = publication->portIndex;
                    vertexToUse->vertexData.componentPortVertex.componentId = publication->componentId;
                    vertexToUse->vertexData.componentPortVertex.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION;
                    vertexToUse->vertexData.componentPortVertex.announcement = publication;
                }

                XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(publicationVertices, vertexToUse), XME_STATUS_INTERNAL_ERROR);

                // 3. clone and add to outRouteCandidates
                vertexPtr = NULL;
                XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addComponentPortVertex(outRouteCandidates, &vertexPtr), XME_STATUS_INTERNAL_ERROR);

                XME_CHECK(vertexPtr != NULL, XME_STATUS_OUT_OF_RESOURCES);

                vertexPtr->nodeId = vertexToUse->nodeId;
                vertexPtr->topicId = vertexToUse->topicId;
                vertexPtr->vertexData.componentPortVertex.componentId = vertexToUse->vertexData.componentPortVertex.componentId;
                vertexPtr->vertexData.componentPortVertex.portType = vertexToUse->vertexData.componentPortVertex.portType;
                vertexPtr->vertexData.componentPortVertex.portIndex = vertexToUse->vertexData.componentPortVertex.portIndex;
                vertexPtr->vertexData.componentPortVertex.announcement = vertexToUse->vertexData.componentPortVertex.announcement;
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();


            // 2. Update edges
            XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                subscriptionVertices, 
                xme_core_pnp_dataLinkGraph_vertexData_t, 
                subscriptionVertex
            );
            {
                XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
                    publicationVertices, 
                    xme_core_pnp_dataLinkGraph_vertexData_t, 
                    publicationVertex
                );
                {
                    if (xme_core_pnp_lrm_matchAnnouncement(
                        publicationVertex->vertexData.componentPortVertex.announcement, 
                        subscriptionVertex->vertexData.componentPortVertex.announcement))
                    {
                        xme_status_t status;

                        /*
                         * check if route already exists in local route Graph. 
                         * steps:
                         * 1. reverse lookup all incoming edges
                         * 2. search each edge for source that matches publication
                         * 3. add missing edges
                         */
                        edgeToUse = NULL;

                        status = xme_hal_graph_initIncomingEdgeIterator(&subGraph->routes, subscriptionVertex->vertexId);
                        XME_ASSERT(XME_STATUS_SUCCESS == status);
                        while (xme_hal_graph_hasNextIncomingEdge(&subGraph->routes, subscriptionVertex->vertexId))
                        {
                            xme_hal_graph_edgeId_t edgeId;
                            xme_hal_graph_vertexId_t vertexId;

                            /*xme_hal_graph_getEdgeData(&subGraph->routes, xme_hal_graph_nextIncomingEdge(&subGraph->routes, subscriptionVertex->vertexId), &edgePtr);*/

                            edgeId = xme_hal_graph_nextIncomingEdge(&subGraph->routes, subscriptionVertex->vertexId);

                            XME_CHECK(edgeId != XME_HAL_GRAPH_INVALID_EDGE_ID, XME_STATUS_INTERNAL_ERROR);

                            vertexId = xme_hal_graph_getSourceVertex(&subGraph->routes, edgeId);

                            XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INTERNAL_ERROR);

                            if (publicationVertex->vertexId == vertexId)
                            {
                                XME_CHECK(xme_hal_graph_getEdgeData(&subGraph->routes, edgeId, (void**)&edgeToUse) == XME_STATUS_SUCCESS, XME_STATUS_INTERNAL_ERROR);
                            }

                        }
                        status = xme_hal_graph_finiIncomingEdgeIterator(&subGraph->routes, subscriptionVertex->vertexId);
                        XME_ASSERT(XME_STATUS_SUCCESS == status);

                        if (edgeToUse == NULL)
                        {
                            XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addLogicalRouteEdge(&subGraph->routes, &edgeToUse, publicationVertex->vertexId, subscriptionVertex->vertexId), XME_STATUS_INTERNAL_ERROR);

                            XME_CHECK(edgeToUse != NULL, XME_STATUS_OUT_OF_RESOURCES);

                            XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_generateChannelId(publicationVertex->vertexData.componentPortVertex.announcement->transactionId, &channelId), XME_STATUS_INTERNAL_ERROR);

                            edgeToUse->topicId = publicationVertex->topicId;
                            edgeToUse->edgeData.logicalRouteEdge.channelId = channelId;
                        }


                        // 3. clone and add to outRouteCandidates

                        // find source vertex
                        sourceVertexId = XME_HAL_GRAPH_INVALID_VERTEX_ID;
                        sinkVertexId = XME_HAL_GRAPH_INVALID_VERTEX_ID;
                        vertexPtr = NULL;

                        status = xme_hal_graph_initVertexIterator(outRouteCandidates);
                        XME_ASSERT(XME_STATUS_SUCCESS == status);
                        while (xme_hal_graph_hasNextVertex(outRouteCandidates))
                        {
                            xme_hal_graph_vertexId_t vertexId;

                            vertexId = xme_hal_graph_nextVertex(outRouteCandidates);

                            XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INTERNAL_ERROR);

                            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getVertexData(outRouteCandidates, vertexId, (void**)&vertexPtr), XME_STATUS_INTERNAL_ERROR);

                            if (xme_core_pnp_lrm_compareComponentPortVertex(subscriptionVertex, vertexPtr, XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION) == 0)
                            {
                                sinkVertexId = vertexId;
                            }
                            else if (xme_core_pnp_lrm_compareComponentPortVertex(publicationVertex, vertexPtr, XME_CORE_COMPONENT_PORTTYPE_DCC_PUBLICATION) == 0)
                            {
                                sourceVertexId = vertexId;
                            }
                        }
                        status = xme_hal_graph_finiVertexIterator(outRouteCandidates);
                        XME_ASSERT(XME_STATUS_SUCCESS == status);

                        XME_CHECK(sinkVertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID && sourceVertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INTERNAL_ERROR);

                        {
                            xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertexData = NULL;
                            xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData = NULL;

                            status = xme_hal_graph_getVertexData(outRouteCandidates, sourceVertexId, (void**) &sourceVertexData);
                            XME_ASSERT(XME_STATUS_SUCCESS == status);

                            status = xme_hal_graph_getVertexData(outRouteCandidates, sinkVertexId, (void**) &sinkVertexData);
                            XME_ASSERT(XME_STATUS_SUCCESS == status);

                            edgePtr = xme_core_pnp_lrm_findEstablishedLogicalRoute(sourceVertexData, sinkVertexData);

                            // If edgePtr is non-NULL, the logical route edge between the given
                            // source and sink vertex has already been established. In this case,
                            // xme_core_pnp_lrm_addLogicalRouteEdge() will
                            // not allocate new edge data for it.
                            if (NULL == edgePtr)
                            {
                                continue; // This is already an established logical route. 
                            }
                            
                            XME_CHECK(XME_STATUS_SUCCESS == xme_core_pnp_lrm_addLogicalRouteEdge(outRouteCandidates, &edgePtr, sourceVertexId, sinkVertexId), XME_STATUS_INTERNAL_ERROR);
                        }

                        XME_CHECK(edgePtr != NULL, XME_STATUS_OUT_OF_RESOURCES);

                        edgePtr->topicId = edgeToUse->topicId;
                        edgePtr->edgeData.logicalRouteEdge.channelId = edgeToUse->edgeData.logicalRouteEdge.channelId;
                    }
                }
                XME_HAL_SINGLYLINKEDLIST_ITERATE_END();
            }
            XME_HAL_SINGLYLINKEDLIST_ITERATE_END();



            // clean up
            XME_HAL_SINGLYLINKEDLIST_FINI(publicationVertices); // FIXME: just clearing the list instead might be more efficient
            XME_HAL_SINGLYLINKEDLIST_FINI(subscriptionVertices);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();


    /*
    What is part of the candidate list?

        Current idea:
        - All possible Routes for a topic that is affected by port announcements.
        - setLogicalRoute checks for duplicates automatically
        - But: somewhere there should be a flag denoting if there are already routes established.
        - -> Idea: Flag should be added to a route candidate 
    */

    // vertex: xme_core_pnp_lrm_vertexData_t
    // edge: xme_core_pnp_lrm_edgeData_t

    /*
        xme_hal_graph_edgeId_t
        xme_hal_graph_addEdge
        (
            xme_hal_graph_graph_t *graph,
            xme_hal_graph_vertexId_t srcVertexId,
            xme_hal_graph_vertexId_t dstVertexId,
            void* edgeData
        );

        Assign edgeId to edgeData manually after adding to graph
    */



    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_lrm_setLogicalRoute
(
    xme_hal_graph_edgeId_t edge,
    xme_core_channelId_t channelId,
    xme_core_transactionId_t transactionId
)
{
    xme_hal_graph_vertexId_t vertexId;
    xme_hal_graph_edgeId_t edgeId;
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData;

    XME_UNUSED_PARAMETER(edge);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_pnp_lrm_state.graphs, xme_core_pnp_lrm_stateSubGraph_t, subGraph);
    {
        if (subGraph->transactionId == transactionId)
        {
            xme_status_t status = xme_hal_graph_initVertexIterator(&subGraph->routes);
            XME_ASSERT(XME_STATUS_SUCCESS == status);
            {
                while (xme_hal_graph_hasNextVertex(&subGraph->routes))
                {
                    vertexId = xme_hal_graph_nextVertex(&subGraph->routes);

                    status = xme_hal_graph_initOutgoingEdgeIterator(&subGraph->routes, vertexId);
                    XME_ASSERT(XME_STATUS_SUCCESS == status);
                    {
                        while (xme_hal_graph_hasNextOutgoingEdge(&subGraph->routes, vertexId))
                        {
                            edgeId = xme_hal_graph_nextOutgoingEdge(&subGraph->routes, vertexId);

                            XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(&subGraph->routes, edgeId, (void**) &edgeData), XME_STATUS_INTERNAL_ERROR);

                            if (edgeData->edgeType == XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE &&
                                edgeData->edgeData.logicalRouteEdge.channelId == channelId)
                            {
                                xme_status_t result = XME_STATUS_SUCCESS;

                                // Committing the same edge twice is an error
                                if (edgeData->edgeData.logicalRouteEdge.established)
                                {
                                    result = XME_STATUS_INTERNAL_ERROR;
                                }
                                else
                                {
                                    edgeData->edgeData.logicalRouteEdge.established = true;
                                }

                                status = xme_hal_graph_finiOutgoingEdgeIterator(&subGraph->routes, vertexId);
                                XME_ASSERT(XME_STATUS_SUCCESS == status);

                                status = xme_hal_graph_finiVertexIterator(&subGraph->routes);
                                XME_ASSERT(XME_STATUS_SUCCESS == status);

                                return result;
                            }
                        }
                    }
                    status = xme_hal_graph_finiOutgoingEdgeIterator(&subGraph->routes, vertexId);
                    XME_ASSERT(XME_STATUS_SUCCESS == status);
                }
            }
            status = xme_hal_graph_finiVertexIterator(&subGraph->routes);
            XME_ASSERT(XME_STATUS_SUCCESS == status);
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return XME_STATUS_INVALID_PARAMETER;
}

/////////////////////////////////////////////////////////////////////////////

xme_status_t 
xme_core_pnp_lrm_createSubGraph
(
    xme_core_pnp_lrm_stateSubGraph_t** subGraph
)
{
    XME_CHECK(subGraph != NULL, XME_STATUS_INVALID_PARAMETER);

    *subGraph = (xme_core_pnp_lrm_stateSubGraph_t*)xme_hal_mem_alloc(sizeof(xme_core_pnp_lrm_stateSubGraph_t));

    XME_CHECK(*subGraph != NULL, XME_STATUS_OUT_OF_RESOURCES);

    XME_CHECK(XME_STATUS_SUCCESS == XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(xme_core_pnp_lrm_state.graphs, *subGraph), XME_STATUS_INTERNAL_ERROR);

    XME_HAL_SINGLYLINKEDLIST_INIT((*subGraph)->publications);
    XME_HAL_SINGLYLINKEDLIST_INIT((*subGraph)->subscriptions);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_init(&(*subGraph)->routes), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_pnp_lrm_removeSubGraph
(
    xme_core_pnp_lrm_stateSubGraph_t** subGraph
)
{
    XME_CHECK(subGraph != NULL, XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN((*subGraph)->publications, xme_core_pnp_lrm_announcement_t, elem);
    {
        xme_hal_mem_free(elem);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN((*subGraph)->subscriptions, xme_core_pnp_lrm_announcement_t, elem);
    {
        xme_hal_mem_free(elem);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_clear(&(*subGraph)->routes), XME_STATUS_INTERNAL_ERROR);

    XME_HAL_SINGLYLINKEDLIST_FINI((*subGraph)->publications);
    XME_HAL_SINGLYLINKEDLIST_FINI((*subGraph)->subscriptions);
    XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_fini(&(*subGraph)->routes), XME_STATUS_INTERNAL_ERROR);

    xme_hal_mem_free(*subGraph);
    XME_CHECK(0 != XME_HAL_SINGLYLINKEDLIST_REMOVE_ITEM(xme_core_pnp_lrm_state.graphs,(*subGraph), false), XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_pnp_lrm_addComponentPortVertex
(
    xme_hal_graph_graph_t* graph, 
    xme_core_pnp_dataLinkGraph_vertexData_t** componentPortVertex
)
{
    xme_hal_graph_vertexId_t newVertex;
    
    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);

    *componentPortVertex = (xme_core_pnp_dataLinkGraph_vertexData_t*)xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_vertexData_t));

    XME_CHECK(*componentPortVertex != NULL, XME_STATUS_OUT_OF_RESOURCES);

    newVertex = xme_hal_graph_addVertex(graph, *componentPortVertex);

    XME_CHECK(newVertex != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INTERNAL_ERROR);

    (*componentPortVertex)->vertexId = newVertex;
    (*componentPortVertex)->vertexType = XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT;

    return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_pnp_lrm_removeComponentPortVertex
(
    xme_hal_graph_graph_t* graph, 
    xme_hal_graph_vertexId_t vertexId
)
{
    //xme_core_pnp_lrm_vertexData_t* vertexToRemove;

    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(vertexId != XME_HAL_GRAPH_INVALID_VERTEX_ID, XME_STATUS_INVALID_PARAMETER);
    // FIXME: The following check is not working
    //XME_CHECK(xme_hal_graph_getVertexData(graph, vertexId, &vertexToRemove) == XME_STATUS_SUCCESS, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(xme_hal_graph_removeVertex(graph, vertexId) == XME_STATUS_SUCCESS, XME_STATUS_INTERNAL_ERROR);
    //xme_hal_mem_free(vertexToRemove);

    //XME_CHECK(vertexToRemove == NULL, XME_STATUS_INTERNAL_ERROR);

    // FIXME: Remove all related edges as well?

    return xme_hal_graph_removeVertex(graph, vertexId);
}

int
xme_core_pnp_lrm_vertexCompareCallbackSameComponentPort
(
    void* vertexData1,
    void* vertexData2
)
{
    xme_core_pnp_dataLinkGraph_vertexData_t* vd1 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData1;
    xme_core_pnp_dataLinkGraph_vertexData_t* vd2 = (xme_core_pnp_dataLinkGraph_vertexData_t*) vertexData2;

    // This callback only handles component port vertices, everything else does not match
    XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vd1->vertexType, -1);
    XME_CHECK(XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT == vd2->vertexType, -1);

    return (//vd1->vertexType == vd2->vertexType && <-- not needed because it is stopped in previous check. 
        vd1->nodeId == vd2->nodeId &&
        vd1->topicId == vd2->topicId &&
        vd1->vertexData.componentPortVertex.portType == vd2->vertexData.componentPortVertex.portType &&
        vd1->vertexData.componentPortVertex.componentId == vd2->vertexData.componentPortVertex.componentId) ?
        0 : 1;
}

int
xme_core_pnp_lrm_edgeCompareCallbackLogicalRouteNotEstablished
(
    void* edgeData1,
    void* edgeData2
)
{
    // FIXME: This function does not compare two edge data, but only one of them. 
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData;

    XME_ASSERT_RVAL(NULL == edgeData2, -1);

    edgeData = (xme_core_pnp_dataLinkGraph_edgeData_t*) edgeData1;

    return (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE == edgeData->edgeType &&
        !edgeData->edgeData.logicalRouteEdge.established) ? 0 : -1;
}

xme_core_pnp_dataLinkGraph_edgeData_t*
xme_core_pnp_lrm_findEstablishedLogicalRoute
(
    xme_core_pnp_dataLinkGraph_vertexData_t* sourceVertexData,
    xme_core_pnp_dataLinkGraph_vertexData_t* sinkVertexData
)
{
    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(xme_core_pnp_lrm_state.graphs, xme_core_pnp_lrm_stateSubGraph_t, subGraph);
    {
        xme_hal_graph_vertexCompareCallback_t vertexCompareCallback;
        xme_hal_graph_edgeCompareCallback_t edgeCompareCallback;
        xme_hal_graph_vertexId_t sourceVertexId;
        xme_hal_graph_vertexId_t sinkVertexId;
        xme_hal_graph_edgeId_t edgeId;
        xme_core_pnp_dataLinkGraph_edgeData_t* edgeData;

        vertexCompareCallback = xme_hal_graph_getVertexCompareCallback(&subGraph->routes);
        edgeCompareCallback = xme_hal_graph_getEdgeCompareCallback(&subGraph->routes);

        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setVertexCompareCallback(&subGraph->routes, xme_core_pnp_lrm_vertexCompareCallbackSameComponentPort), NULL);
        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setEdgeCompareCallback(&subGraph->routes, xme_core_pnp_lrm_edgeCompareCallbackLogicalRouteNotEstablished), NULL);

        sourceVertexId = xme_hal_graph_getNextVertexWithDataComparison(&subGraph->routes, sourceVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
        if (XME_HAL_GRAPH_INVALID_VERTEX_ID != sourceVertexId)
        {
            sinkVertexId = xme_hal_graph_getNextVertexWithDataComparison(&subGraph->routes, sinkVertexData, XME_HAL_GRAPH_INVALID_VERTEX_ID);
            if (XME_HAL_GRAPH_INVALID_VERTEX_ID != sinkVertexId)
            {
                //edgeId = XME_HAL_GRAPH_INVALID_EDGE_ID; <-- not needed, because it is never read. 
                do
                {
                    edgeId = xme_hal_graph_getNextEdgeBetweenWithDataComparison(&subGraph->routes, sourceVertexId, sinkVertexId, NULL, XME_HAL_GRAPH_INVALID_EDGE_ID);
                    if (edgeId != XME_HAL_GRAPH_INVALID_EDGE_ID)
                    {
                        edgeData = NULL;
                        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_getEdgeData(&subGraph->routes, edgeId, (void**) &edgeData), NULL);
                        XME_ASSERT_RVAL(NULL != edgeData, NULL);

                        if (XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE != edgeData->edgeType)
                        {
                            continue;
                        }

                        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setVertexCompareCallback(&subGraph->routes, vertexCompareCallback), NULL);
                        XME_CHECK(XME_STATUS_SUCCESS == xme_hal_graph_setEdgeCompareCallback(&subGraph->routes, edgeCompareCallback), NULL);

                        // Found a matching edge
                        return edgeData;
                    }
                }
                while (XME_HAL_GRAPH_INVALID_EDGE_ID != edgeId);
            }
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    // No match found
    return NULL;
}

xme_status_t
xme_core_pnp_lrm_addLogicalRouteEdge
(
    xme_hal_graph_graph_t* graph,
    xme_core_pnp_dataLinkGraph_edgeData_t** logicalRouteEdge,
    xme_hal_graph_vertexId_t sourceVertexId,
    xme_hal_graph_vertexId_t sinkVertexId
)
{
    xme_hal_graph_edgeId_t newEdge;

    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_ASSERT(NULL != logicalRouteEdge);

    // FIXME: redundant?
    XME_CHECK(xme_hal_graph_getVertexData(graph, sourceVertexId, NULL) == XME_STATUS_SUCCESS, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(xme_hal_graph_getVertexData(graph, sinkVertexId, NULL) == XME_STATUS_SUCCESS, XME_STATUS_INVALID_PARAMETER);

    if (NULL == *logicalRouteEdge)
    {
        *logicalRouteEdge = (xme_core_pnp_dataLinkGraph_edgeData_t*)xme_hal_mem_alloc(sizeof(xme_core_pnp_dataLinkGraph_edgeData_t));
        XME_CHECK(*logicalRouteEdge != NULL, XME_STATUS_OUT_OF_RESOURCES);
    }

    XME_ASSERT(NULL != *logicalRouteEdge);

    newEdge = xme_hal_graph_addEdge(graph, sourceVertexId, sinkVertexId, *logicalRouteEdge);

    XME_CHECK_REC(newEdge != XME_HAL_GRAPH_INVALID_EDGE_ID, XME_STATUS_INTERNAL_ERROR, xme_hal_mem_free(*logicalRouteEdge));

    (*logicalRouteEdge)->edgeId = newEdge;
    (*logicalRouteEdge)->edgeType = XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE;

    return XME_STATUS_SUCCESS;
}

xme_status_t 
xme_core_pnp_lrm_removeLogicalRouteEdge(xme_hal_graph_graph_t* graph, xme_hal_graph_edgeId_t edgeId)
{
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeToRemove;
    
    XME_CHECK(graph != NULL, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(edgeId != XME_HAL_GRAPH_INVALID_EDGE_ID, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(xme_hal_graph_getEdgeData(graph, edgeId, (void**)&edgeToRemove) == XME_STATUS_SUCCESS, XME_STATUS_INVALID_PARAMETER);
    XME_CHECK(xme_hal_graph_getEdgeData(graph, edgeId, (void**)&edgeToRemove) == XME_STATUS_SUCCESS, XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(xme_hal_graph_removeEdge(graph, edgeId) == XME_STATUS_SUCCESS, XME_STATUS_INTERNAL_ERROR);

    xme_hal_mem_free(edgeToRemove);


    XME_CHECK(edgeToRemove == NULL, XME_STATUS_INTERNAL_ERROR);

    return XME_STATUS_SUCCESS;
}

int 
xme_core_pnp_lrm_compareLogicalRouteEdge
(
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData1, 
    xme_core_pnp_dataLinkGraph_edgeData_t* edgeData2
)
{
    XME_CHECK(edgeData1 != NULL, 1);
    XME_CHECK(edgeData2 != NULL, 1);
    //No checks as they would hide null reference exceptions

    if (edgeData1->edgeType == edgeData2->edgeType &&
        edgeData1->edgeType == XME_CORE_PNP_DATALINKGRAPH_EDGETYPE_LOGICALROUTE  && // separate comparison
        edgeData1->topicId == edgeData2->topicId &&
        edgeData1->edgeData.logicalRouteEdge.channelId == edgeData2->edgeData.logicalRouteEdge.channelId)
        return 0;

    return 1;
}

int 
xme_core_pnp_lrm_compareComponentPortVertex
(
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData1, 
    xme_core_pnp_dataLinkGraph_vertexData_t* vertexData2,
    xme_core_component_portType_t portType
)
{
    XME_CHECK(vertexData1 != NULL, 1);
    XME_CHECK(vertexData2 != NULL, 1);
    //No checks as they would hide null reference exceptions

    if (vertexData1->vertexType == vertexData2->vertexType &&
        vertexData1->vertexType == XME_CORE_PNP_DATALINKGRAPH_VERTEXTYPE_COMPONENT_PORT &&
        vertexData1->topicId == vertexData2->topicId &&
        vertexData1->nodeId == vertexData2->nodeId &&
        vertexData1->vertexData.componentPortVertex.componentId == vertexData2->vertexData.componentPortVertex.componentId &&
        vertexData1->vertexData.componentPortVertex.portType == vertexData2->vertexData.componentPortVertex.portType &&
        vertexData1->vertexData.componentPortVertex.portIndex == vertexData2->vertexData.componentPortVertex.portIndex &&
        vertexData1->vertexData.componentPortVertex.portType == portType)
        return 0;

    return 1;
}

bool 
xme_core_pnp_lrm_announcementExists
(
    xme_hal_linkedList_descriptor_t announcementList, 
    xme_core_pnp_lrm_announcement_t* announcement 
)
{
    bool result = false;

    // Equality of announcements is based on all properties except transactionId
    // and topic id, because input announcementList is already of the same
    // topic id.

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
        announcementList, 
        xme_core_pnp_lrm_announcement_t, 
        announcementItem
    );
    {
        if (announcementItem->nodeId == announcement->nodeId &&
            announcementItem->componentId == announcement->componentId &&
            announcementItem->portType == announcement->portType)
        {
            result = true;
            break;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    return result;
}

bool 
xme_core_pnp_lrm_matchAnnouncement
(
    xme_core_pnp_lrm_announcement_t* sourceAnnouncement, 
    xme_core_pnp_lrm_announcement_t* sinkAnnouncement
)
{

    XME_CHECK(sourceAnnouncement != NULL, false);
    XME_CHECK(sinkAnnouncement != NULL, false);

    return sourceAnnouncement->topicId == sinkAnnouncement->topicId &&
        xme_core_pnp_lrm_matchAttributes(sourceAnnouncement, sinkAnnouncement);
}

bool
xme_core_pnp_lrm_matchAttributes
(
    xme_core_pnp_lrm_announcement_t* sourceAnnouncement,
    xme_core_pnp_lrm_announcement_t* sinkAnnouncement
)
{
    xme_status_t status;
    int matches = false;

    status = xme_core_directory_attribute_isFilterMatchingDefinition
    (
        sinkAnnouncement->attrSet,
        sourceAnnouncement->attrSet,
        &matches
    );
    XME_ASSERT(XME_STATUS_SUCCESS == status);

    return (bool) matches;
}

/**
 * @}
 */
