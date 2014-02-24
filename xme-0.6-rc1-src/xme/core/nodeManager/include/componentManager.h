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
 * $Id: componentManager.h 4905 2013-09-02 13:46:15Z ruiz $
 */

/**
 * \file
 *         Component Manager.
 */

/**
 * \defgroup core_compMan Component Manager group
 * @{
 *
 * \brief The objective of the broker group is to act as a mediatior between
 *        the Data Handler and the Execution Manager.
 *
 * \details Tasks of the ComponentManager:
 *          - Activation of advanced components when preconditions are satisfied, possible preconditions:
 *              -# Core is active (useful for local applications)
 *              -# Node is connected to network
 *          - Activation means analyzing the manifest of the container, its components and functions, checking the feasibility and allocation of resources
 *          - Sends buffered publications and subscriptions (used by local applications) to directory as soon as node is connected to network
 *          - Receives manifest (plus potential additional information, e.g. with respect to execution frequency) from Plug&Play manager and checks feasibility to install a container/component on the node
 *          - Sends schedule feasibility requests to the scheduler for each function contained in the to-be-installed component. The order of the requests depends on the priority of the function (first all required functions, then the optional functions ordered according to their influence on the quality level)
 *          - Receive binary from Plug&Play manager/app store? and install the container
 *          - Activates components upon request from directory(?)
 *          - Checks feasibility of new wirings for existing and running components.
 * \note Open question:
 *       - how do we initially setup the local channels (by a local directory, by preconfiguration, by the component manager)
 * \note Design decision:
 *       - As the component manager has to check the availability of resources in advance before installing a component, the resource manager should also trigger the allocation and then handover the resources. Rationale: we hereby avoid that the interface definition is kept both in the manifest and the function code 
 */

#ifndef XME_CORE_NODEMANAGER_COMPONENTMANAGER_H
#define XME_CORE_NODEMANAGER_COMPONENTMANAGER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/manifestTypes.h"
#include "xme/core/coreTypes.h"

//#include "xme/core/directory/plugAndPlayManager.h"

#include <stdbool.h>
/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

/*** Windows-Code *************************************************************/
#if defined(_WIN32) //if Windows
    // exclude rarely-used Windows header files
    #define WIN32_LEAN_AND_MEAN
    // Windows Header Files:
    #include <windows.h>
    #include <strsafe.h>

    #define XME_COMPONENT_DYLIB_HANDLE HINSTANCE ///< DLL_LIB_HANDLE
    #define XME_COMPONENT_FUNC_DECL __declspec(dllexport) ///< DLL_FUNC_DECL
    #define XME_COMPONENT_LOAD_LIB(param_lib_name) LoadLibraryA(param_lib_name) ///< DLL_LIB_LOAD
    #define XME_COMPONENT_GET_SYM(param_lib_handle, param_symbol_name) GetProcAddress(param_lib_handle, param_symbol_name) ///< DLL_GET_SYM
    #define XME_COMPONENT_FREE_LIB(param_lib_name) FreeLibrary(param_lib_name) ///< DLL_LIB_FREE
/*** Linux-Code ***************************************************************/
#elif defined(__GNUC__) && defined(XME_CPU_ARCH_NIOSII) //if Linux
    /* null implementation for Altera NiosII CPU architecture (NiosII defined in xme_opt.h) */
    #define XME_COMPONENT_DYLIB_HANDLE void* ///< DLL_LIB_HANDLE
    #define XME_COMPONENT_FUNC_DECL ///< DLL_FUNC_DECL
    #define XME_COMPONENT_LOAD_LIB(param_lib_name) (0l) ///< DLL_LIB_LOAD -- do {} while(0)
    #define XME_COMPONENT_GET_SYM(param_lib_handle, param_symbol_name) (0l) ///< DLL_GET_SYM -- do {} while(0)
    #define XME_COMPONENT_FREE_LIB(param_lib_name) (0l)///< DLL_LIB_FREE -- do {} while(0)
#else
    /* definitions for other GNU platforms, e.g. Linux */
#include <dlfcn.h>
    #define XME_COMPONENT_DYLIB_HANDLE void* ///< DLL_LIB_HANDLE
    #define XME_COMPONENT_FUNC_DECL ///< DLL_FUNC_DECL
    #define XME_COMPONENT_LOAD_LIB(param_lib_name) dlopen(param_lib_name, RTLD_LAZY) ///< DLL_LIB_LOAD
    #define XME_COMPONENT_GET_SYM(param_lib_handle, param_symbol_name) dlsym(param_lib_handle, param_symbol_name) ///< DLL_GET_SYM
    #define XME_COMPONENT_FREE_LIB(param_lib_name) dlclose(param_lib_name) ///< DLL_LIB_FREE
#endif

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

#define XME_CORE_COMPONENTMANAGER_MAX_COMPONENTS_PER_CONTAINER 10 ///< max components per container. 

/**
 * \struct xme_core_nodeManager_componentManager_containerDescriptor_t
 *
 * \brief the component manager container descriptor
 */
typedef struct
{
    XME_COMPONENT_DYLIB_HANDLE handle; ///< the component handle. 
    int numberOfComponents; ///< the number of components. 
    xme_core_componentDescriptor_t* refComponentDescriptors[XME_CORE_COMPONENTMANAGER_MAX_COMPONENTS_PER_CONTAINER]; ///< the reference component descriptors. 
} 
xme_core_nodeManager_componentManager_containerDescriptor_t;

#define XME_CORE_COMPONENTMANAGER_MAX_CONTAINERS 10 ///< defines the max number of container of the component manager. 
extern xme_core_nodeManager_componentManager_containerDescriptor_t xme_core_nodeManager_componentManager_containerDescriptors[XME_CORE_COMPONENTMANAGER_MAX_CONTAINERS]; ///< contains the container descriptors. 

/**
 * \struct xme_core_nodeManager_componentManager_configurationFile_entry_t
 *
 * \brief the component manager configuration file entry
 */
typedef struct 
{
    char component_name[32]; ///< the component name.
    char shared_library_path[256]; ///< the component shared library path. 
    bool auto_start; ///< if the component starts automatically.
    bool start_on_network_connection; ///< if the component start when there is network connection. 
    struct xme_core_nodeManager_componentManager_configurationFile_entry_t* next; ///< the next configuration entry.
} 
xme_core_nodeManager_componentManager_configurationFile_entry_t;

/******************************************************************************/
/***   Static variables                                                     ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

/**
 * \brief  Initializes the component manager.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the component manager has been successfully
 *            initialized.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if the component manager got an error during initialization.
 */
xme_status_t
xme_core_nodeManager_componentManager_init(void);

/**
 * \brief  Shuts down the component manager.
 * \return Returns one of the following status codes:
 *          - XME_CORE_STATUS_SUCCESS if the shutdown has been successfully done.
 *          - XME_CORE_STATUS_INTERNAL_ERROR if an error occurred.
 */
xme_status_t
xme_core_nodeManager_componentManager_fini(void);

/**
 * \brief  Announcement that this node has been successfully logged in the network.
 *
 * This function is called by the LoginClient exclusively.
 *
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_loginSuccess(void);

/**
 * \brief  Determine the components that have been preinstalled on this node
 *
 * This function is called after this node was successfully logged into the net.
 * After this function determined the preinstalled components on this node, these components are announced at the central PnP-Manager
 *
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_calculatePreinstalledComponents(void);


/*** Configuration Interface ***/
//TODO: Parameter 'channelId' ist nicht im Component Manager Klassendiagramm angegeben, wird aber im Sequenzdiagramm 'Directory-View' zweimal �bergeben (ganz unten) (ticket #2116)

/**
 * \brief This call activates a function by integrating it into the deployment
 *
 * \param transactionId The id of the current transaction
 * \param channelId the channel id. 
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_activateDeployment
(
    xme_core_transactionId_t transactionId,
    xme_core_channelId_t channelId
);

/**
 * \brief Determines if the deployment is feasible. 
 *
 * \param containerManifest the component manifest. 
 * \param componentId the component identifier. 
 * \param componentSchedulingInformation the alpha curve. 
 * \param outMetric the output parameter indicating the metric. 
 * \param outIsBinaryPresent the output parameter inticating if the binary is present. 
 * \param outTransactionId the output parameter indicating the transaction identifier. 
 *
 * \return the return status code. 
 */
xme_status_t
xme_core_nodeManager_componentManager_feasibleDeployment
(
    xme_core_directory_containerManifest_t containerManifest,
    xme_core_component_t componentId,
    xme_core_directory_componentSchedulingManifest_t componentSchedulingInformation,
    xme_core_metric_t* outMetric,
    bool* outIsBinaryPresent,
    xme_core_transactionId_t* outTransactionId
);

/**
 * \brief Installs a binary. 
 *
 * \param transactionId The transaction id. 
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_installBinary
(
    xme_core_transactionId_t transactionId
);

/**
 * \brief Integrates a shared library. 
 *
 * \param sharedLib_path The shared library path pointer. 
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_integrateSharedLibrary
(
    char *sharedLib_path
);

/**
 * \brief Integrates a shared library for Integration Level 1. 
 *
 * \param sharedLib_path The shared library path pointer. 
 * \return status code
 */
xme_status_t
xme_core_nodeManager_componentManager_integrateSharedLibrary_il1
(
    char *sharedLib_path
);

//TODO(KB): just for testing, remove later! see ticket #2024
/**
 * \brief A fake pnp trigger. 
 *
 * \param sharedLibraryPath The shared library path pointer. 
 * \param fakeId a fake identifier.
 * \return status code
 */
void
xme_core_nodeManager_componentManager_fakePnpTrigger
(
    char* sharedLibraryPath,
    int fakeId
);

/**
 * \brief This function "unplugs" a container
 *
 * \param indexContainerDescriptors is the index in the ContainerDescriptors Array
 *
 * \return status code.
 */
void
xme_core_nodeManager_componentManager_fakePnpUnplugTrigger
(
    int indexContainerDescriptors
);

/*** Code from former ResourceManager ************************************************************/


/**
 * \brief Activate all components. 
 *
 * \return status code.
 */
xme_status_t
xme_core_nodeManager_componentManager_activateComponents(void);

/**
 * \brief Activates the given component. 
 *
 * \param componentId the component id to activate.
 * 
 * \return status code.
 */
xme_status_t
xme_core_nodeManager_componentManager_activateComponent
(
    xme_core_component_t componentId
);

/**
 * \brief Deactivate all components. 
 *
 * \return status code.
 */
xme_status_t
xme_core_nodeManager_componentManager_deactivateComponents(void);

/**
 * \brief Deactivates the given component. 
 *
 * \param componentId the component id to activate.
 * 
 * \return status code.
 */
xme_status_t
xme_core_nodeManager_componentManager_deactivateComponent
(
    xme_core_component_t componentId
);

/**
 * \brief  Initializes a new component.
 *
 * \param  componentDescriptor The descriptor of the component under creation (containing the pointers to the callback functions and the config struct as well as the component name)
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_SUCCESS if the component has been properly created.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if creation failed.
 */
xme_status_t
xme_core_nodeManager_componentManager_createComponent
(
    xme_core_componentDescriptor_t* componentDescriptor
);

/**
 * \brief Destroyes a Component.
 *
 * \param componentId of the component to be destroyed.
 *
 * \return Returns one of the following status codes:
 *          - XME_CORE_SUCCESS if the component has been properly destroyed.
 *          - XME_CORE_STATUS_OUT_OF_RESOURCES if destroy failed.
 */
xme_status_t
xme_core_nodeManager_componentManager_destroyComponent
(
    xme_core_component_t componentId
);

/**
 * \brief  Get handle of component with given id.
 *
 * \param  componentId Locally valid component id.
 *
 * \return Pointer to component or NULL when no component with
 *         given id exists.
 */
xme_core_componentDescriptor_t*
xme_core_nodeManager_componentManager_getComponentHandle
(
    xme_core_component_t componentId
);

/**
 * \brief  Get number of components inside the linked list 'xme_core_nodeManager_componentManager_componentDescriptorLinkedList'
 *
 * \return Number of components
 */
uint16_t
xme_core_nodeManager_componentManager_getComponentCount(void);

/**
 * \brief  Get name of component with given id.
 *
 * \param  componentId Locally valid component id.
 *
 * \return String with name of component or NULL when no component with
 *         given id exists.
 */
const char*
xme_core_nodeManager_componentManager_getComponentName
(
    xme_core_component_t componentId
);

/**
 * \brief  Prints a list of the names of all active components.
 */
void
xme_core_nodeManager_componentManager_printAllComponentNames(void);

XME_EXTERN_C_END

/**
 * @}
 */

#endif // #ifndef XME_CORE_NODEMANAGER_COMPONENTMANAGER_H

