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
 * $Id: auditHandler.h 3535 2013-05-28 14:22:17Z camek $
 */

/**
 * \file
 *         Data Handler.
 *
 * \brief The audit function provides monitoring and collection of information
 *        about security-related actions, and subsequent analysis of the information
 *        to review security policies, controls and procedures.
 */

#ifndef XME_CORE_DATAHANDLER_AUDITHANDLER_H_
#define XME_CORE_DATAHANDLER_AUDITHANDLER_H_

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//

#include "xme/core/component.h"

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

/**
 * \enum xme_core_dataHandler_auditElements_t
 * \brief enumeration values for audit elements. 
 */
typedef enum auditElements{
  XME_AUDITELEMENT_INVALID = 0, ///< invalid audit element. 
  XME_AUDITELEMENT_NEWOBJECT_ADDED, ///< new object added. 
  XME_AUDITELEMENT_OBJECT_DELETED, ///< object deleted. 
  XME_AUDITELEMENT_ACCESSRIGHT_DISTRIBUTED, ///< access rights distributed. 
  XME_AUDITELEMENT_ACCESSRIGHT_REVOCED, ///< access rights revoked.
  XME_AUDITELEMENT_CAPABILITY_DISTRIBUTED, ///< capability distributed. 
  XME_AUDITELEMENT_CAPABILITY_REVOCED, ///< capability revoked. 
  XME_AUDITELEMENT_SUBJECT_SECURITY_ATTRIBUTE_CHANGED, ///< the subject of security attribute has changed. 
  XME_AUDITELEMENT_OBJECT_SECURITY_ATTRIBUTE_CHANGED, ///< the object of security attribute has changed. 
  XME_AUDITELEMENT_POLICY_CHECK_PERFORMED, ///< policy check performed. 
  XME_AUDITELEMENT_POLICY_CHECK_BYPASSED, ///< The use of access rights to bypass a policy check
  XME_AUDITELEMENT_USE_IDENTIFICATION_FUNCTION, ///< use of identification function. 
  XME_AUDITELEMENT_USE_AUTHENTIFICATION_FUNCTION, ///< use of authentification function. 
  XME_AUDITELEMENT_USE_IDENTIFICATION_AND_AUTHENTICATION_FUNCTION, ///< use of both authentification and identification function. 
  XME_AUDITELEMENT_SUPERUSER_ACTION_PERFORMED, ///< superuser action performed. 
  XME_AUDITELEMENT_DATA_IMPORTED_FROM_EXTERN, ///< data imported from extern source. 
  XME_AUDITELEMENT_DATA_EXPORTED_TO_EXTERN ///< data exported to extern sink. 
} xme_core_dataHandler_auditElements_t;

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Creates a data handler dump.  
 * \retval XME_STATUS_SUCESS if the data handler is successfully dumped. 
 * \retval XME_STATUS_INTERNAL_ERROR in any other case. 
 */
extern xme_status_t 
xme_core_dataHandler_dataHandlerDump(void);

/**
 * \brief Generates an audit entry.
 * \retval XME_STATUS_SUCESS if the audit entry is generated. 
 * \retval XME_STATUS_INTERNAL_ERROR if audit entry cannot be generated. 
 */
extern xme_status_t 
xme_core_dataHandler_generateAuditEntry(void);

/**
 * \brief Signals an alarm. 
 * \retval XME_STATUS_SUCESS if the alarm is signaled. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot signal an alarm. 
 */
extern xme_status_t 
xme_core_dataHandler_signalAlarm(void);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_AUDITHANDLER_H_ */
