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
 * $Id: componentRepository.c 4598 2013-08-07 14:28:43Z ruiz $
 */


/**
 * \file
 *         Implementation of Execution Manager component repository.
 */

#define MODULE_ACRONYM "ExecMgrRep: "

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/defines.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"
#include "xme/hal/include/mem.h"
#include "xme/core/log.h"

//******************************************************************************//
//***   Local variables                                                      ***//
//******************************************************************************//
/**
 * \var ComponentRepository
 * \brief Contains all registered component descriptors
 */
struct ComponentRepository_s
{
    xme_hal_singlyLinkedList_t(XME_CORE_EXEC_REPOSITORY_MAX_COMPONENTS) components;
} ComponentRepository;

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//

/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_init( void )
{
    XME_HAL_SINGLYLINKEDLIST_INIT(ComponentRepository.components);
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_fini( void )
{
    /* fixme: refactor: Make cleanup! */
    /**     odo clean up component descriptors and function descriptors at exit
    */
    XME_HAL_SINGLYLINKEDLIST_FINI(ComponentRepository.components);
    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_getComponent
(
    xme_core_component_t componentId,
    xme_core_exec_componentDescriptor_t** component
)
{
    xme_core_exec_componentDescriptor_t* result = NULL;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId,
              XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(ComponentRepository.components,
            xme_core_exec_componentDescriptor_t,
            loopItem);
        if(loopItem->componentId == componentId )
        {
            result = loopItem;
            break;
        }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(NULL == result)
    {
        *component = NULL;
        return XME_STATUS_NOT_FOUND;
    }

    *component = result;
    return XME_STATUS_SUCCESS;

}

/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_getComponentFunction
(
        xme_core_exec_componentDescriptor_t* component,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
)
{
    xme_core_exec_functionDescriptor_t* result = NULL;

    XME_CHECK( NULL != component,
        XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId,
                  XME_STATUS_INVALID_PARAMETER);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            component->functions,
            xme_core_exec_functionDescriptor_t,
            loopItem );
        if(loopItem->functionId == functionId )
        {
            result = loopItem;
            break;
        }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(NULL == result)
    {
        *function = NULL;
        return XME_STATUS_NOT_FOUND;
    }

    *function = result;
    return XME_STATUS_SUCCESS;
}

/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_getFunction(
        xme_core_component_t componentId,
        xme_core_component_functionId_t functionId,
        xme_core_exec_functionDescriptor_t** function
)
{
    xme_core_exec_componentDescriptor_t* componentDesc = NULL;
    xme_core_exec_functionDescriptor_t* result = NULL;

    XME_CHECK(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentId,
                  XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_CORE_COMPONENT_INVALID_FUNCTION_CONTEXT != functionId,
                  XME_STATUS_INVALID_PARAMETER);

    XME_CHECK(XME_STATUS_SUCCESS== xme_core_exec_componentRepository_getComponent(componentId, &componentDesc),
              XME_STATUS_NO_SUCH_VALUE);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(
            componentDesc->functions,
            xme_core_exec_functionDescriptor_t,
            loopItem );
    {
        if(loopItem->functionId == functionId )
        {
                result = loopItem;
                break;
        }
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(NULL == result)
    {
        *function = NULL;
        return XME_STATUS_NOT_FOUND;
    }

    *function = result;
    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
xme_status_t
xme_core_exec_componentRepository_registerComponent
(
    const xme_core_exec_componentDescriptor_t* const componentDescriptor
)
{
    xme_core_exec_componentDescriptor_t* tmpComponent = NULL;
    XME_CHECK_MSG(NULL != componentDescriptor,
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_ERROR,
        MODULE_ACRONYM "registerComponent: NULL pointer parameter!\n");

    XME_CHECK_MSG(XME_CORE_COMPONENT_INVALID_COMPONENT_CONTEXT != componentDescriptor->componentId,
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_ERROR,
        MODULE_ACRONYM "registerComponent: invalid component id passed as a  parameter!\n");

    XME_CHECK_MSG(0 < XME_HAL_SINGLYLINKEDLIST_ITEM_COUNT(componentDescriptor->functions),
        XME_STATUS_INVALID_PARAMETER,
        XME_LOG_ERROR,
        MODULE_ACRONYM "registerComponent: component descriptor [%d] has no functions!\n",
        componentDescriptor->componentId);

    XME_HAL_SINGLYLINKEDLIST_ITERATE_BEGIN(componentDescriptor->functions, xme_core_exec_functionDescriptor_t, functionDesc);
    {
        XME_CHECK_MSG(true == xme_core_exec_isValidFunctionDescriptor(functionDesc),
            XME_STATUS_INVALID_PARAMETER,
            XME_LOG_ERROR,
            MODULE_ACRONYM "function descriptor is not valid!\n",
            componentDescriptor->componentId);
    }
    XME_HAL_SINGLYLINKEDLIST_ITERATE_END();

    if(XME_STATUS_NOT_FOUND == xme_core_exec_componentRepository_getComponent(componentDescriptor->componentId, &tmpComponent))
        return XME_HAL_SINGLYLINKEDLIST_ADD_ITEM(
                ComponentRepository.components,
                (xme_core_exec_componentDescriptor_t*) componentDescriptor);
    else
        return XME_STATUS_ALREADY_EXIST;
}
