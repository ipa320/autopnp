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
 * $Id: componentManager.c 3467 2013-05-23 13:48:45Z ruiz $
 */

/**
 * \file
 *         Component Manager.
 */

/**
 * \addtogroup core_compMan
 * @{
 *
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/nodeManager/include/componentManager.h"
#include "xme/core/nodeManager/include/binaryManager.h"
#include "xme/core/directory/include/plugAndPlayManager.h"
#include "xme/core/xme_api.h"
#include "xme/core/componentContext.h"
#include "xme/core/componentList.h"

#include "xme/hal/include/linkedList.h"
#include "xme/hal/include/mem.h"
#if !defined(XME_CPU_ARCH_NIOSII)
#include "xme/hal/include/fileio.h"
#include "xme/hal/include/safeString.h"
#endif /* !defined(XME_CPU_ARCH_NIOSII) */

#include "xme/core/executionManager/include/executionManagerWrapperInterface.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#define XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_COMPONENT_NAME 0 ///< the component name index.
#define XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_SHARED_LIBRARY_PATH 1 ///< the library path index.
#define XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_AUTO_START 2 ///< the autostart index.
#define XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_START_ON_NETWORK_CONNECTION 3 ///< the start on network connection index. 
#define XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_MEMBERS_END 4 ///< the members end index. 

/******************************************************************************/
/***   Global variables                                                     ***/
/******************************************************************************/

/**
 * \brief  Next free component identifier.
 */
static xme_core_component_t xme_core_nodeManager_componentManager_nextComponentId = (xme_core_component_t)1U;

/**
 * The current transaction-ID of the Component-Manager.
 * Notice: The Component-Manager can momentarily only handle one transaction at a time. 
 */

// XXX: VR: I do not see any reason not to move the following lines here from componentList
// xme_core_componentDescriptor_t* xme_core_nodeManager_componentManager_componentDescriptorsPtr;
// xme_core_component_t xme_core_nodeManager_componentManager_initialNumberOfComponents;
xme_core_transactionId_t xme_core_nodeManager_componentManager_curTransactionId = XME_CORE_INVALID_TRANSACTION_ID; ///< the current transaction id. 
xme_core_transactionId_t xme_core_nodeManager_componentManager_lastTransactionId = XME_CORE_INVALID_TRANSACTION_ID; ///< the last transaction id. 

#ifdef PRE_IL1_SUPPORT
XME_API_INIT()
#endif // #ifdef PRE_IL1_SUPPORT
XME_API_IL1_INIT()

#ifdef PRE_IL1_SUPPORT
typedef void (*initFunction_t) (xme_api_t*, xme_core_componentDescriptor_t*, uint16_t*, uint16_t*);
#endif // #ifdef PRE_IL1_SUPPORT
typedef void (*initFunction_il1_t) (xme_api_il1_t*, xme_core_componentDescriptor_t*, uint16_t*, uint16_t*);
typedef void (*componentCallbacks_t) (void*); //void* points to the Component_configStruct

/**
 * A double linked list containing the component descriptors (type: xme_core_componentDescriptor_t) of plugged in components.
 *
 * This linked list will substitute the static array 'xme_core_nodeManager_componentManager_componentDescriptorsPtr[]' which is not capable to be extended during runtime.
 * TODO(KB): What is a good maximal number of the linked-list entries???, see ticket #2031
 */
xme_hal_doublyLinkedList_t(100) xme_core_nodeManager_componentManager_componentDescriptorLinkedList;

//Maximal number of Shared Libraries that can be loaded at a time.
//TODO(KB): change this to be a linked list, therewidth there is no max limit of libraries that can be loaded, see ticket #2031
//Calling Shared Library a Container
xme_core_nodeManager_componentManager_containerDescriptor_t xme_core_nodeManager_componentManager_containerDescriptors[XME_CORE_COMPONENTMANAGER_MAX_CONTAINERS];

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

#if !defined(XME_CPU_ARCH_NIOSII)

static int get_token(char* buffer, int buffer_size, char* token, FILE** fp);
static xme_status_t xme_core_nodeManager_componentManager_readConfigurationFile(void); 

#define TOKEN_COMMA ','
#define TOKEN_NEWLINE '\n'

static int get_token(char* buffer, int buffer_size, char* token, FILE** fp)
{
	static int read_size = 0;
	static int parser_head = 0;
	int token_length = 0;

	while(1) {
		if(read_size - parser_head <= 0) {
			read_size = xme_hal_fileio_fread(buffer, sizeof(char), buffer_size, *fp);
			if(read_size == 0)
			{	
				token_length = 0;
				token[token_length] = EOF;
				read_size = 0;
				parser_head = 0;
				break;
			}
			parser_head = 0;
		}

		// Handling Line End Character (Linux, Mac, Windows)
		if(token_length!=0 && (buffer[parser_head] == ',' || buffer[parser_head] == '\n' || buffer[parser_head] == '\r')) {
			token[token_length++] = '\0';
			//parser_head++;
			// Handling Windows Line End Character "\r\n"
			/*if(buffer[parser_head] == '\n')
			{
				parser_head++;
			}*/
			break;
		}
		else if(token_length==0 && buffer[parser_head] == ',') {
			parser_head++;
			token[token_length++] = TOKEN_COMMA;
			token[token_length++] = '\0';
			break;
		}
		else if(token_length==0 && (buffer[parser_head] == '\n' || buffer[parser_head] == '\r')) {
			parser_head++;
			if(buffer[parser_head] == '\n')
			{
				parser_head++;
			}
			token[token_length++] = TOKEN_NEWLINE;
			token[token_length++] = '\0';
			break;
		}
		else
		{
			token[token_length++] = buffer[parser_head];
		}
		parser_head++;
	}
	
	return token_length;
}

/**
 * \brief The list of entries (lines) of the component-managers configuration file
 */
xme_core_nodeManager_componentManager_configurationFile_entry_t* xme_core_nodeManager_componentManager_configurationFile_entries_list = NULL;

static xme_status_t
xme_core_nodeManager_componentManager_readConfigurationFile(void)
{
	// Dummy implementation of reading from manifest file to initialize Linked List of components
	xme_hal_fileio_fileHandle_t fp;
	xme_core_nodeManager_componentManager_configurationFile_entry_t* temp_container = NULL;
	char* token;
	int token_length = 0;
	char* buffer;
	int member_index = 0;
	int temp_index = 0;

#if defined(_WIN32)
	fp = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_1.txt", XME_HAL_FILEIO_MODE_READONLY);
#elif defined(__GNUC__)
	fp = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_2.txt", XME_HAL_FILEIO_MODE_READONLY);
	//fp = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_3_malformed.txt", XME_HAL_FILEIO_MODE_READONLY);
#endif

	//Error Handling in case file is not found, or cannot be opened.	
	XME_CHECK_MSG
	(
		fp != NULL,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Error in Opening Configuration File\n"
	);
	
	token = (char*)xme_hal_mem_alloc(sizeof(char)*256);
	buffer = (char*)xme_hal_mem_alloc(sizeof(char)*16);
	token_length = get_token(buffer, 16, token, &fp);
		
	while(token[0] != EOF)
	{
		//TODO: add error handling for case the file is malformed, e.g. a line contains less or more than 4 elements! see issue #2031
		//use 'xme_fileio_manifest_csv_test_file_3_malformed.txt' to test correct handling of malformed files
		switch(member_index)
		{
			case XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_COMPONENT_NAME: 
				temp_container = (xme_core_nodeManager_componentManager_configurationFile_entry_t*) xme_hal_mem_alloc(sizeof(xme_core_nodeManager_componentManager_configurationFile_entry_t));
				xme_hal_mem_set(temp_container, 0, sizeof(xme_core_nodeManager_componentManager_configurationFile_entry_t));
				xme_hal_safeString_strncpy(temp_container->component_name, token, token_length);
				get_token(buffer, 16, token, &fp);
				XME_CHECK_MSG
				(
					TOKEN_COMMA == token[0],
					XME_STATUS_INTERNAL_ERROR,
					XME_LOG_ERROR,
					"Malformed Configuration File\n"
				);
				break;
			case XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_SHARED_LIBRARY_PATH: 
				xme_hal_safeString_strncpy(temp_container->shared_library_path, token, token_length);
				get_token(buffer, 16, token, &fp);
				XME_CHECK_MSG
				(
					TOKEN_COMMA == token[0],
					XME_STATUS_INTERNAL_ERROR,
					XME_LOG_ERROR,
					"Malformed Configuration File\n"
				);
				break;
			case XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_AUTO_START: 
				temp_container->auto_start = (atoi(token) != 0);
				get_token(buffer, 16, token, &fp);
				XME_CHECK_MSG
				(
					TOKEN_COMMA == token[0],
					XME_STATUS_INTERNAL_ERROR,
					XME_LOG_ERROR,
					"Malformed Configuration File\n"
				);
				break;
			case XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_START_ON_NETWORK_CONNECTION: 
				temp_container->start_on_network_connection = (atoi(token) != 0);
				get_token(buffer, 16, token, &fp);
				XME_CHECK_MSG
				(
					TOKEN_NEWLINE == token[0],
					XME_STATUS_INTERNAL_ERROR,
					XME_LOG_ERROR,
					"Malformed Configuration File\n"
				);
				break;
			default:
				break;
		}
		member_index++;
		if(member_index == XME_NODEMANAGER_COMPONENTMANAGER_CONTAINER_INFO_MEMBERS_END)
		{
			temp_container->next = xme_core_nodeManager_componentManager_configurationFile_entries_list;
			xme_core_nodeManager_componentManager_configurationFile_entries_list = temp_container;
			temp_container = NULL;
			member_index=0;
		}
		token_length = get_token(buffer, 16, token, &fp);
	}

	xme_hal_mem_free(buffer);
	xme_hal_mem_free(token);
	
	//TODO : Remove Later. Printing values read from file.
	temp_index=0;
	temp_container = xme_core_nodeManager_componentManager_configurationFile_entries_list;
	while(temp_container != NULL)
	{
		XME_LOG(XME_LOG_NOTE, "Component No. = %d\n", temp_index);
		XME_LOG(XME_LOG_NOTE, "Component Name = %s\n", temp_container->component_name);
		XME_LOG(XME_LOG_NOTE, "Shared Library Path = %s\n", temp_container->shared_library_path);
		XME_LOG(XME_LOG_NOTE, "Auto Start = %d\n", temp_container->auto_start);
		XME_LOG(XME_LOG_NOTE, "Start on Network Connection = %d\n", temp_container->start_on_network_connection);
		XME_LOG(XME_LOG_NOTE, "--------------------------------------\n");
		temp_index++;
		temp_container = temp_container->next;
	}

	return XME_STATUS_SUCCESS;
}

#endif /* !defined(XME_CPU_ARCH_NIOSII) */

xme_status_t
xme_core_nodeManager_componentManager_init(void)
{
	//XME_ASSERT(XME_HAL_TLS_INVALID_TLS_HANDLE == xme_core_resourceManager_config.currentComponentIdTlsHandle);

	// Initialize configuration structure
	XME_HAL_DOUBLYLINKEDLIST_INIT(xme_core_nodeManager_componentManager_componentDescriptorLinkedList);

#if !defined(XME_CPU_ARCH_NIOSII)
	XME_CHECK_MSG
	(
		(XME_STATUS_SUCCESS == xme_core_nodeManager_componentManager_readConfigurationFile()),
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Loading configuration file of Component-Manager failed!\n"
	);
#endif /* !defined(XME_CPU_ARCH_NIOSII) */

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_fini(void)
{
	// Destroy software components (reverse because the core components should be destroyed last)
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t, 
		cd
	);
	{
		if (cd->destroy)
		{
			// ...call the destruction routine and destroy the information structure for the component

			XME_COMPONENT_CONTEXT
			(
				cd->componentId,
				{
					cd->destroy(cd->config);
				}
			);
		}
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
	return XME_STATUS_SUCCESS;
}

static xme_status_t
xme_core_nodeManager_componentManager_generateTransactionId
(
	xme_core_transactionId_t* outTransactionId
)
{
	*outTransactionId = XME_CORE_INVALID_TRANSACTION_ID;

	//Abort if another PnP-transaction is in progress currently
	XME_CHECK_MSG
	(
		XME_CORE_INVALID_TRANSACTION_ID == xme_core_nodeManager_componentManager_curTransactionId,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_WARNING,
		"The request to the Component-Manager cannot be handled because currently another transaction is ongoing!\n"
	);

	//find next transaction id
	xme_core_nodeManager_componentManager_curTransactionId = xme_core_nodeManager_componentManager_lastTransactionId + (xme_core_transactionId_t)1;

	//handle overflow
	if(xme_core_nodeManager_componentManager_curTransactionId == XME_CORE_MAX_TRANSACTION_ID){
		xme_core_nodeManager_componentManager_curTransactionId = (xme_core_transactionId_t)1;
	}
	//Abort if the produced transactionId is invalid
	if(xme_core_nodeManager_componentManager_curTransactionId == XME_CORE_INVALID_TRANSACTION_ID){
		XME_LOG
		(
			XME_LOG_ERROR,
			"Component-Manager failed to find a valid transactionID, aborting!\n"
		);
		return XME_STATUS_INTERNAL_ERROR;
	}
	
	*outTransactionId = xme_core_nodeManager_componentManager_curTransactionId;
	return XME_STATUS_SUCCESS;
}

static xme_status_t
xme_core_nodeManager_componentManager_confirmTransaction
(
	xme_core_transactionId_t transactionId
)
{
	XME_CHECK_MSG
	(
		(transactionId == xme_core_nodeManager_componentManager_curTransactionId),
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Component-Manager cannot confirm transaction, because the id doesn't correspond to the currently active transaction!\n"
	);
	xme_core_nodeManager_componentManager_lastTransactionId = xme_core_nodeManager_componentManager_curTransactionId;
	xme_core_nodeManager_componentManager_curTransactionId = XME_CORE_INVALID_TRANSACTION_ID;
	//TODO(KB): implement function, see ticket #2031
	return XME_STATUS_SUCCESS;
}

/*
static xme_status_t
xme_core_nodeManager_componentManager_rollbackTransaction
(
	xme_core_transactionId_t transactionId
)
{
	XME_UNUSED_PARAMETER(transactionId);
	//TODO(KB): implement function, see ticket #2031
	return XME_STATUS_UNSUPPORTED;
}
*/

xme_status_t
xme_core_nodeManager_componentManager_loginSuccess(void)
{
	//TODO(KB): implement announcement of all components on this new node to the PnP-Manager, see ticket #2031
	XME_CHECK_MSG
	(
		(XME_STATUS_SUCCESS == xme_core_nodeManager_componentManager_calculatePreinstalledComponents()),
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Determination of preinstalled components failed in Component-Manager!\n"
	);

	//for each preinstalled component
	//TODO: call xme_core_directory_pnpManager_addContainer, see ticket #2031
	{
		//TODO(KB): this has to be substituted by RPC in distributed case for IL3,  see ticket #2024
		//xme_core_directory_pnpManager_addContainer(containerManifest, componentManifests, nodeId, containerId, componentIds, 2);
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_calculatePreinstalledComponents(void)
{
	//TODO(KB): implement determination of preinstalled components on this node, see ticket #2031
	//TODO(KB): Question: Search only for components compiled into XME or also for components inside SharedLibraries or both? see ticket #2031
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_activateDeployment
(
	xme_core_transactionId_t transactionId,
	xme_core_channelId_t channelId
)
{
	//definitions
	bool transactionSuccess;
	XME_UNUSED_PARAMETER(transactionId);
	XME_UNUSED_PARAMETER(channelId);

	//initializations
	transactionSuccess = true;

	//TODO: implement functionality, see ticket #2031
	
	/*
	//ask RM to activate the Resource Partition
	if(XME_STATUS_SUCCESS != activateResourcePartition(transactionId))
	{
		transactionSuccess = false;
	}


	//ask EMC to activate the schedule
	if(XME_STATUS_SUCCESS != activateSchedule(transactionId))
	{
		transactionSuccess = false;
	}
	*/

	if(transactionSuccess) {
		return XME_STATUS_SUCCESS;
	}
	else {
		return XME_STATUS_INTERNAL_ERROR;
	}
}

xme_status_t
xme_core_nodeManager_componentManager_feasibleDeployment
(
	xme_core_directory_containerManifest_t containerManifest,
	xme_core_component_t componentId,
	xme_core_directory_componentSchedulingManifest_t componentSchedulingInformation,
	xme_core_metric_t* outMetric,
	bool* outIsBinaryPresent,
	xme_core_transactionId_t* outTransactionId
)
{
	//definitions
	bool transactionSuccess;
	xme_core_metric_t metric;
	bool binaryPresent;

	//initializations
	transactionSuccess = true;
	metric = XME_CORE_INVALID_METRIC;
	binaryPresent = false;

	XME_UNUSED_PARAMETER(componentId);
	XME_UNUSED_PARAMETER(componentSchedulingInformation);

	*outTransactionId = XME_CORE_INVALID_TRANSACTION_ID;
	xme_core_nodeManager_componentManager_generateTransactionId(outTransactionId);
	
	XME_CHECK_MSG
	(
		(	XME_CORE_INVALID_TRANSACTION_ID != *outTransactionId)
		&&(	XME_CORE_MAX_TRANSACTION_ID != *outTransactionId),
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_WARNING,
		"xme_core_nodeManager_componentManager_feasibleDeployment aborted, another transaction is ongoing!\n"
	);
	

	//ask Binary Manager, if the binary can be stored on the node
	if(XME_STATUS_SUCCESS != xme_core_nodeManager_binaryManager_binaryFeasible(
		containerManifest.binarySize,
		(xme_core_nodeManager_binaryId_t) 5555, //TODO: who is responsible to find a free binaryId?, see ticket #2031
		*outTransactionId,
		&metric,
		&binaryPresent))
	{
		transactionSuccess = false;
	}

	//ask Resource Manager, 
	//TODO: implement "resourcePartitionFeasible", see ticket #2044

	//ask Execution Manager Configurator
	//TODO: implement "scheduleFeasible", see ticket #2026, #2027
	
	*outMetric = (xme_core_metric_t) 100;
	*outIsBinaryPresent = true;


	if(transactionSuccess) {
		xme_core_nodeManager_componentManager_confirmTransaction(xme_core_nodeManager_componentManager_curTransactionId);
		return XME_STATUS_SUCCESS;
	}
	else {
		return XME_STATUS_INTERNAL_ERROR;
	}
}

xme_status_t
xme_core_nodeManager_componentManager_installBinary
(
	xme_core_transactionId_t transactionId
)
{
	XME_UNUSED_PARAMETER(transactionId);

	//TODO(KB): implement function, see ticket #2031

	//ask BM to store the binary
	//return xme_core_nodeManager_binaryManager_storeBinary(transactionId);
	return XME_STATUS_SUCCESS;
}


/*** Code from former ResourceManager ************************************************************/

xme_status_t
xme_core_nodeManager_componentManager_createComponent(xme_core_componentDescriptor_t* componentDescriptor)
{
	// If this item is in use...
	if (componentDescriptor->create)
	{
		// ...create an information structure for the component and call the creation routine

		// Check whether a new component can actually be added
		// TODO: On error, clean up components initialized properly so far! See ticket #758
		XME_CHECK(xme_core_nodeManager_componentManager_nextComponentId < XME_MAX_SYSTEM_VALUE, XME_STATUS_OUT_OF_RESOURCES);
		componentDescriptor->componentId = xme_core_nodeManager_componentManager_nextComponentId++;

		XME_LOG
		(
			XME_LOG_VERBOSE,
			"Creating software component \"%s\" (#%d)\n",
			componentDescriptor->componentName,
			componentDescriptor->componentId
		);

		//Add the componentDescriptor to the dynamically exdentable linked list 'xme_core_nodeManager_componentManager_componentDescriptorLinkedList'
		XME_HAL_DOUBLYLINKEDLIST_ADD_ITEM(xme_core_nodeManager_componentManager_componentDescriptorLinkedList, (void*)componentDescriptor);

		// Switch to the component context
		XME_COMPONENT_CONTEXT
		(
			componentDescriptor->componentId,
			{
				// Create the component
				// TODO: On error, clean up components initialized properly so far! See ticket #758
				XME_CHECK_MSG
				(
					XME_STATUS_SUCCESS == componentDescriptor->create(componentDescriptor->config),
					XME_STATUS_OUT_OF_RESOURCES,
					XME_LOG_FATAL,
					"Component #%d (\"%s\") failed to initialize!\n",
					componentDescriptor->componentId,
					componentDescriptor->componentName
				);
			}
		);
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_destroyComponent(xme_core_component_t componentId)
{
	xme_core_componentDescriptor_t* foundComponent = xme_core_nodeManager_componentManager_getComponentHandle(componentId);
	if(foundComponent==NULL)
	{
		return XME_STATUS_NOT_FOUND;
	}

	// If this item is in use...
	if (foundComponent->destroy)
	{
		XME_LOG
		(
			XME_LOG_VERBOSE,
			"Destroying software component \"%s\" (#%d)\n",
			foundComponent->componentName,
			foundComponent->componentId
		);

		XME_HAL_DOUBLYLINKEDLIST_REMOVE_ITEM(xme_core_nodeManager_componentManager_componentDescriptorLinkedList, (void*)foundComponent, false);

		// Switch to the component context
		XME_COMPONENT_CONTEXT
		(
			foundComponent->componentId,
			{
				// Create the component
				// TODO: On error, clean up components initialized properly so far! See ticket #758
				XME_CHECK_MSG
				(
					XME_STATUS_SUCCESS == foundComponent->destroy(foundComponent->config),
					XME_STATUS_OUT_OF_RESOURCES,
					XME_LOG_FATAL,
					"Component #%d (\"%s\") failed to destroy!\n",
					foundComponent->componentId,
					foundComponent->componentName
				);
			}
		);
	}
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_activateComponents(void)
{
	// Activate software components
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t, 
		cd
	);
	{
		// If this item is in use...
		if (cd->activate)
		{
			xme_core_nodeManager_componentManager_activateComponent(cd->componentId);
		}
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();

	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_activateComponent(xme_core_component_t componentId)
{
	xme_core_componentDescriptor_t* foundComponent = xme_core_nodeManager_componentManager_getComponentHandle(componentId);
	if(foundComponent==NULL)
	{
		return XME_STATUS_NOT_FOUND;
	}

	XME_COMPONENT_CONTEXT
	(
		foundComponent->componentId,
		{
			// ...activate the respective component
			XME_CHECK_MSG
			(
				XME_STATUS_SUCCESS == foundComponent->activate(foundComponent->config),
				XME_STATUS_INTERNAL_ERROR,
				XME_LOG_FATAL,
				"Component #%d failed to activate!\n",
				foundComponent->componentId
			);
		}
	);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_deactivateComponent(xme_core_component_t componentId)
{
	xme_core_componentDescriptor_t* foundComponent = xme_core_nodeManager_componentManager_getComponentHandle(componentId);
	if(foundComponent==NULL)
	{
		return XME_STATUS_NOT_FOUND;
	}

	XME_COMPONENT_CONTEXT
	(
		foundComponent->componentId,
		{
			// ...activate the respective component
			XME_CHECK_MSG
			(
				XME_STATUS_SUCCESS == foundComponent->deactivate(foundComponent->config),
				XME_STATUS_INTERNAL_ERROR,
				XME_LOG_FATAL,
				"Component #%d (\"%s\") failed to deactivate!\n",
				foundComponent->componentId,
				foundComponent->componentName
			);
		}
	);
	return XME_STATUS_SUCCESS;
}

xme_status_t
xme_core_nodeManager_componentManager_deactivateComponents(void)
{
	// Deactivate software components (reverse because the core components should be stopped last)
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t,
		cd
	);
	{
		// If this item is in use...
		if (cd->deactivate)
		{
			XME_COMPONENT_CONTEXT
			(
				cd->componentId,
				{
					// ...deactivate the respective component
					cd->deactivate(cd->config);
				}
			);
		}
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_REVERSE_END();
	return XME_STATUS_SUCCESS;
}

//utility function returning the name of the component with the given ID 
const char*
xme_core_nodeManager_componentManager_getComponentName
(
	xme_core_component_t componentId
)
{
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t, 
		cd
	);
	{
		if (cd->componentId == componentId)
		{
			return cd->componentName;
		}
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
	// When no component with given id has been found return NULL
	return NULL;
}

void 
xme_core_nodeManager_componentManager_printAllComponentNames(void)
{
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t, 
		cd
	);
	{
		XME_LOG
		(
			XME_LOG_NOTE,
			"%i: %s\n",
			cd->componentId,
			cd->componentName
		);
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
	XME_LOG
	(
		XME_LOG_NOTE,
		"#Components: %i\n",  
		xme_core_nodeManager_componentManager_getComponentCount()
	);
	XME_LOG(XME_LOG_NOTE,"---\n");
}

//utility function returning a handle to the component with the given ID 
xme_core_componentDescriptor_t*
xme_core_nodeManager_componentManager_getComponentHandle
(
	xme_core_component_t componentId
)
{
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_BEGIN
	(
		xme_core_nodeManager_componentManager_componentDescriptorLinkedList, 
		xme_core_componentDescriptor_t, 
		cd
	);
	{
		if(cd->componentId == componentId) 
		{
			return cd;
		}
	}
	XME_HAL_DOUBLYLINKEDLIST_ITERATE_END();
	// When no component with given id has been found return NULL
	XME_LOG
	(
		XME_LOG_WARNING,
		"Warning, ComponentId #%d not found!\n",
		componentId
	);
	return NULL;
}

//utility function returning the number of components
uint16_t 
xme_core_nodeManager_componentManager_getComponentCount(void)
{
	return XME_HAL_DOUBLYLINKEDLIST_ITEM_COUNT(xme_core_nodeManager_componentManager_componentDescriptorLinkedList);
}


/*** Platform specific-code ********************************************************************/

#ifdef PRE_IL1_SUPPORT

/**
 * \brief Function loads a new dynamic library (comprising 1 or more components) 
 * and integrates the contained components into the system.
 */
xme_status_t
xme_core_nodeManager_componentManager_integrateSharedLibrary(char *sharedLib_path)
{

#if defined(WIN32) || defined(__GNUC__) //if Windows or Linux

	initFunction_t	initFunction = NULL;
	//TODO(KB): whats a good size for temporal array componentDescriptors? (storing the descriptors of the newly plugged in components), see ticket #2031
	xme_core_componentDescriptor_t tempComponentDescriptorArray[10];
	//find a free Container for the shared library file that should be loaded
	int freeContainerId = -1;

	xme_core_nodeManager_componentManager_printAllComponentNames();

	//for finding first free slot in array		
	for(freeContainerId = 0; freeContainerId<XME_CORE_COMPONENTMANAGER_MAX_CONTAINERS; freeContainerId++)
	{
		if(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle == NULL)
		{
			break;
		}
	}
	
	XME_CHECK_MSG
	(
		-1 != freeContainerId,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Cannot load a shared library (maximal number of loadable shared libraries reached)\n"
	);

	// Load the shared library file
	xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle = XME_COMPONENT_LOAD_LIB(sharedLib_path);

	if (xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle != NULL)
	{

		initFunction = (initFunction_t) XME_COMPONENT_GET_SYM(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle, "xme_application_initialize");
		if ( initFunction != NULL ) 
		{
			int j = 0;
			uint16_t numberOfNewComponents = 0;
			uint16_t plugin_used_xme_core_api_version = 0;
						
			//call the init function of the DLL and get the callback handles of the components inside the DLL
			(*initFunction)(&xme_api, tempComponentDescriptorArray, &numberOfNewComponents, &plugin_used_xme_core_api_version);

			XME_CHECK_MSG
			(
				XME_CORE_API_VERSION == plugin_used_xme_core_api_version,
				XME_STATUS_INTERNAL_ERROR,
				XME_LOG_ERROR,
				"Component Manager cannot integrate a shared library, because it's developed against the wrong XME_CORE_API_VERSION!\n"
			);
	
			XME_LOG
			(
				XME_LOG_NOTE, "#Number of old Components: %i\n", xme_core_nodeManager_componentManager_getComponentCount()
			);
			XME_LOG
			(
				XME_LOG_NOTE, "#Number of new Components: %i\n", numberOfNewComponents
			);
			//for all new components
			for(j=0; j<numberOfNewComponents; j++)
			{
				/* Create all required data-structures for the new component, during this:
				* - add the config-struct to the linked component-list of component descriptors
				* - call the 'create' callback function of the new component
				*/
	
				//create memory space for a new componentDescriptor and copy the componentDescriptor of the sharedLibrary into this, because tempComponentDescriptorArray is only on the stack and vanishes later
				xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j] = (xme_core_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_componentDescriptor_t));
				xme_hal_mem_copy(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j], &tempComponentDescriptorArray[j], sizeof(xme_core_componentDescriptor_t));
				
				xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = j+1;

				/* Start the new components */
				XME_LOG
				(
					XME_LOG_NOTE,
					"Starting component '%s'\n",  
					xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]->componentName
				);
	
				//create resources of new component
				xme_core_nodeManager_componentManager_createComponent(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]);
				//start the new component
				//TODO(KB): this has probably to be done not here but later when the new configuration is activated, see ticket #2031
				xme_core_nodeManager_componentManager_activateComponent(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]->componentId);
			}
		}
		else 
		{
			XME_LOG
			(
				XME_LOG_ERROR,
				"Failed to call a shared library function (during GetProcAddress/dlsym)\n"
			);
	
			XME_COMPONENT_FREE_LIB(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle);
			xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = 0;
			return XME_STATUS_NOT_FOUND;
		}
	}
	else 
	{
		XME_LOG
		(
			XME_LOG_ERROR,
			"Failed to load a shared library (during LoadLibraryA/dlopen)\n"
		);
		XME_COMPONENT_FREE_LIB(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle);
		xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = 0;
		return XME_STATUS_NOT_FOUND;
	}

	//xme_core_nodeManager_componentManager_printAllComponentNames();

	//TODO(KB): free the library later while plug-off the DLL or shutdown the system (don't free it here), see ticket #2031
	//FreeLibrary(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId]);
	return XME_STATUS_SUCCESS;

#else //other OS than Windows or Linux

	XME_UNUSED_PARAMETER(sharedLib_path);
	XME_LOG
	(
		XME_LOG_ERROR,
		"xme_core_nodeManager_componentManager_integrateSharedLibrary not implemented on this platform\n"
	);
	return XME_STATUS_UNSUPPORTED;

#endif // defined(WIN32) || defined(__GNUC__)
}

#endif // #ifdef PRE_IL1_SUPPORT


/**
 * \brief Function loads a new dynamic IL1 library (comprising 1 or more components) 
 * and integrates the contained components into the system.
 */
xme_status_t
xme_core_nodeManager_componentManager_integrateSharedLibrary_il1(char *sharedLib_path)
{

#if defined(WIN32) || defined(__GNUC__) //if Windows or Linux

	initFunction_il1_t	initFunction = NULL;
	//TODO(KB): whats a good size for temporal array componentDescriptors? (storing the descriptors of the newly plugged in components), see ticket #2031
	xme_core_componentDescriptor_t tempComponentDescriptorArray[10];
	//find a free Container for the shared library file that should be loaded
	int freeContainerId = -1;

	xme_core_nodeManager_componentManager_printAllComponentNames();

	//for finding first free slot in array		
	for(freeContainerId = 0; freeContainerId<XME_CORE_COMPONENTMANAGER_MAX_CONTAINERS; freeContainerId++)
	{
		if(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle == NULL)
		{
			break;
		}
	}
	
	XME_CHECK_MSG
	(
		-1 != freeContainerId,
		XME_STATUS_INTERNAL_ERROR,
		XME_LOG_ERROR,
		"Cannot load a shared library (maximal number of loadable shared libraries reached)\n"
	);

	// Load the shared library file
	xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle = XME_COMPONENT_LOAD_LIB(sharedLib_path);

	if (xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle != NULL)
	{

		initFunction = (initFunction_il1_t) XME_COMPONENT_GET_SYM(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle, "xme_application_initialize");
		if ( initFunction != NULL ) 
		{
			//int j = 0;
			uint16_t numberOfNewComponents = 0;
			uint16_t plugin_used_xme_core_api_version = 0;
						
			//call the init function of the DLL and get the callback handles of the components inside the DLL
			(*initFunction)(&xme_api_il1, tempComponentDescriptorArray, &numberOfNewComponents, &plugin_used_xme_core_api_version);

			XME_CHECK_MSG
			(
				XME_CORE_API_VERSION == plugin_used_xme_core_api_version,
				XME_STATUS_INTERNAL_ERROR,
				XME_LOG_ERROR,
				"Component Manager cannot integrate a shared library, because it's developed against the wrong XME_CORE_API_VERSION!\n"
			);
	
			XME_LOG
			(
				XME_LOG_NOTE, "#Number of old Components: %i\n", xme_core_nodeManager_componentManager_getComponentCount()
			);
			XME_LOG
			(
				XME_LOG_NOTE, "#Number of new Components: %i\n", numberOfNewComponents
			);
#if 0
			//for all new components
			for(j=0; j<numberOfNewComponents; j++)
			{
				/* Create all required data-structures for the new component, during this:
				* - add the config-struct to the linked component-list of component descriptors
				* - call the 'create' callback function of the new component
				*/
	
				//create memory space for a new componentDescriptor and copy the componentDescriptor of the sharedLibrary into this, because tempComponentDescriptorArray is only on the stack and vanishes later
				xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j] = (xme_core_componentDescriptor_t*) xme_hal_mem_alloc(sizeof(xme_core_componentDescriptor_t));
				xme_hal_mem_copy(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j], &tempComponentDescriptorArray[j], sizeof(xme_core_componentDescriptor_t));
				
				xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = j+1;

				/* Start the new components */
				XME_LOG
				(
					XME_LOG_NOTE,
					"Starting component '%s'\n",  
					xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]->componentName
				);
	
				//create resources of new component
				xme_core_nodeManager_componentManager_createComponent(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]);
				//start the new component
				//TODO(KB): this has probably to be done not here but later when the new configuration is activated, see ticket #2031
				xme_core_nodeManager_componentManager_activateComponent(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].refComponentDescriptors[j]->componentId);
			}
#endif
		}
		else 
		{
			XME_LOG
			(
				XME_LOG_ERROR,
				"Failed to call a shared library function (during GetProcAddress/dlsym)\n"
			);
	
			XME_COMPONENT_FREE_LIB(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].handle);
			xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = 0;
			return XME_STATUS_NOT_FOUND;
		}
	}
	else 
	{
		XME_LOG
		(
			XME_LOG_ERROR,
			"Failed to load a shared library (during LoadLibraryA/dlopen)\n"
		);
		
		xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId].numberOfComponents = 0;
		return XME_STATUS_NOT_FOUND;
	}

	//xme_core_nodeManager_componentManager_printAllComponentNames();

	//TODO(KB): free the library later while plug-off the DLL or shutdown the system (don't free it here), see ticket #2031
	//FreeLibrary(xme_core_nodeManager_componentManager_containerDescriptors[freeContainerId]);
	return XME_STATUS_SUCCESS;

#else //other OS than Windows or Linux

	XME_UNUSED_PARAMETER(sharedLib_path);
	XME_LOG
	(
		XME_LOG_ERROR,
		"xme_core_nodeManager_componentManager_integrateSharedLibrary_il1 not implemented on this platform\n"
	);
	return XME_STATUS_UNSUPPORTED;

#endif // defined(WIN32) || defined(__GNUC__)
}




/*********************************************************************************************/

/*
 * \brief This function "unplugs" a container
 * 
 * \param indexContainerDescriptors is the index in the ContainerDescriptors Array
 */
void 
xme_core_nodeManager_componentManager_fakePnpUnplugTrigger
(
	int indexContainerDescriptors
)
{
	int i = 0;
	for(i=0; i < xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].numberOfComponents; i++)
	{
		xme_core_componentDescriptor_t* cd = xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].refComponentDescriptors[i];
		
		XME_LOG
		(
			XME_LOG_NOTE, "#Removing Component '%s' with ID: %i\n", xme_core_nodeManager_componentManager_getComponentName(cd->componentId), cd->componentId
		);
		
		xme_core_nodeManager_componentManager_deactivateComponent(cd->componentId);
		xme_core_nodeManager_componentManager_destroyComponent(cd->componentId);
	}

	XME_LOG
	(
		XME_LOG_NOTE, "#Freeing Library, with indexContainerDescriptor %i\n", indexContainerDescriptors
	);
	
	//unplug the container (DLL/SO)
	XME_COMPONENT_FREE_LIB(xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].handle);
	//reset the containerDescriptor Array
	xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].handle = NULL;
	for(i=0; i<xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].numberOfComponents; i++)
	{
		//resetting refComponentDescriptors
		xme_hal_mem_free(xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].refComponentDescriptors[i]);
		xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].refComponentDescriptors[i] = NULL;
	}
	xme_core_nodeManager_componentManager_containerDescriptors[indexContainerDescriptors].numberOfComponents = 0;
	

	xme_core_nodeManager_componentManager_printAllComponentNames();
	XME_LOG
	(
		XME_LOG_NOTE, "#Unplug finished\n"
	);
}

/**
 * @}
 */
