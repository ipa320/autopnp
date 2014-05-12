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
 * $Id: databaseTestProbeManipulations.h 7684 2014-03-05 15:00:06Z ruiz $
 */

/**
 * \file
 *         Database for Test Probe Abstraction.
 *
 * \brief This component is focused only on operation performed by the Test Probe.
 */

#ifndef XME_CORE_DATAHANDLER_DATABASETESTPROBEMANIPULATIONS_H
#define XME_CORE_DATAHANDLER_DATABASETESTPROBEMANIPULATIONS_H

//******************************************************************************//
//***   Includes                                                             ***//
//******************************************************************************//
#include "xme/core/dataHandler/internal/databaseInternalMethods.h"

#include "xme/defines.h"

//******************************************************************************//
//***   Prototypes                                                           ***//
//******************************************************************************//
XME_EXTERN_C_BEGIN

/**
 * \brief Applies manipulations to the provided data store.
 * \details Copies from master database the content of the data store and 
 *          applies remaining manipulations to that data store. 
 *
 * \param[in] dataStore The struct containing the data store.
 *
 * \retval XME_STATUS_SUCCESS If the set of manipulations was applied.
 */
xme_status_t
xme_core_dataHandler_databaseTestProbe_applyManipulations
(
    xme_core_dataHandler_database_dataStore_t* const dataStore
);

XME_EXTERN_C_END

#endif /* XME_CORE_DATAHANDLER_DATABASETESTPROBEMANIPULATIONS_H */

