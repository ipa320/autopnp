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
 * $Id: loginManagerInternalTypes.h 4374 2013-07-26 08:25:02Z geisinger $
 */

/**
 * \file
 *         Login Manager.
 */

#ifndef XME_CORE_LOGIN_LOGINMANAGERINTERNALTYPES_H
#define XME_CORE_LOGIN_LOGINMANAGERINTERNALTYPES_H

/**
 * \addtogroup core_login_manager
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/component.h"
#include "xme/core/manifestTypes.h"

#include "xme/hal/include/table.h"

/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/

/**
 * \struct xme_core_login_loginManager_componentItem_t
 * \brief Stores the component and component type associated 
 *        to the topic received in the login request. 
 */
typedef struct
{
    xme_core_component_t componentId; ///< the component id. 
    xme_core_componentType_t componentType; ///< the component type.
} xme_core_login_loginManager_componentItem_t;

/**
 * @}
 */

#endif // #ifndef XME_CORE_LOGIN_LOGINMANAGERINTERNALTYPES_H
