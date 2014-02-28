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
 * $Id: auditHandler.c 3329 2013-05-16 16:14:42Z camek $
 */

/**
 * \file
 *         Audit Handler.
 */

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/log.h"

#include "xme/core/dataHandler/include/dataHandlerInternalTypes.h"
#include "xme/core/dataHandler/include/auditHandler.h"

//******************************************************************************//
//***   Type definitions                                                     ***//
//******************************************************************************//

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//

/**
 * \brief Records an audit for the data handler
 * \retval XME_STATUS_SUCESS if the records is successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot record the audit. 
 */
static xme_status_t
xme_core_dataHandler_recordAudit(void);

/**
 * \brief Collects alarms for the data handler
 * \retval XME_STATUS_SUCESS if alarms are collected successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot collect alarms. 
 */
static xme_status_t
xme_core_dataHandler_collectAlarm(void);

/**
 * \brief Examins alarms for the data handler
 * \retval XME_STATUS_SUCESS if the examination is successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot examin the alarm. 
 */
static xme_status_t
xme_core_dataHandler_examinAlarm(void);

/**
 * \brief Analyses an audit trail for the data handler
 * \retval XME_STATUS_SUCESS if the analysis is successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot analyse the audit. 
 */
static xme_status_t
xme_core_dataHandler_analysAuditTrail(void);

/*xme_status_t
xme_core_dataHandler_archiveAuditTrail(void);
*/

/**
 * \brief Examins audit trails for the data handler
 * \retval XME_STATUS_SUCESS if the examination is successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot examin the audtit trails. 
 */
static xme_status_t
xme_core_dataHandler_examinAuditTrail(void);

/**
 * \brief Collects audit trails for the data handler
 * \retval XME_STATUS_SUCESS if audit trails are collected successfully. 
 * \retval XME_STATUS_INTERNAL_ERROR if cannot collect audit trails. 
 */
static xme_status_t
xme_core_dataHandler_collectAuditTrail(void);

//******************************************************************************//
//***   Implementation                                                       ***//
//******************************************************************************//
xme_status_t
xme_core_dataHandler_generateAuditEntry(void)
{
  xme_status_t status = XME_STATUS_INTERNAL_ERROR;

  status = xme_core_dataHandler_collectAuditTrail();

  return status;
}

xme_status_t
xme_core_dataHandler_signalAlarm(void){
  xme_status_t status = XME_STATUS_INTERNAL_ERROR;

  status = xme_core_dataHandler_collectAlarm();
  status = xme_core_dataHandler_examinAlarm();
  return status;
}

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_collectAlarm(void){
  return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_examinAlarm(void){
  xme_status_t status = XME_STATUS_INTERNAL_ERROR;

  status = xme_core_dataHandler_collectAuditTrail();
  return status;
}

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_analysAuditTrail(void){
  return XME_STATUS_SUCCESS;
}

//******************************************************************************//
/*xme_status_t
xme_core_dataHandler_archiveAuditTrail(){
  retrun XME_STATUS_SUCCESS;
}
*/

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_recordAudit(void){
  return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_examinAuditTrail(void){
  return XME_STATUS_SUCCESS;
}

//******************************************************************************//
static xme_status_t
xme_core_dataHandler_collectAuditTrail(void){
  xme_status_t status = XME_STATUS_INTERNAL_ERROR;

  // first we will get the timestamp
    //Timestamp Absolute time-stamp of when the event occurred.

    // second we will get the current component from the execution manager

    // third we will collect all ports of that component to

    // fourth what was done by the component and why we call this function

    // fifth additional arguments of the operation

    // sixth what was the performed action

  // seventh we will analyze the audit information if it is valid and consistency
  status = xme_core_dataHandler_examinAuditTrail();

  // shall we do that here?
  status = xme_core_dataHandler_analysAuditTrail();

  // eighth we will record the audit info

    status = xme_core_dataHandler_recordAudit();

  return status;
}
