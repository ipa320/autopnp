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
 * $Id: detectDevicesFunctionWrapper.h 6372 2014-01-17 09:58:50Z kainz $
 */

/**
 * \file
 *         Function wrapper - a generic abstraction for one executable function
 *              scheduled by the execution manager.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

#ifndef DETECTOR_ADV_DETECTOR_DETECTDEVICESFUNCTIONWRAPPER_H
#define DETECTOR_ADV_DETECTOR_DETECTDEVICESFUNCTIONWRAPPER_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/core/executionManager/include/executionManagerDataStructures.h"
#include "xme/core/log.h"

#include "xme/defines.h"

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/
XME_EXTERN_C_BEGIN

/**
 * \brief Start execution of function.
 *
 * \details This will start an infinite loop and execute the wrapped step
 *          function. Before each step call we wait until the execution
 *          managers signals that the function is ready to execute.
 *
 * \param userFunctionDescRaw The xme_core_exec_functionDescriptor_t of the
 *        function that will be executed by this wrapper. Must not be null.
 *        If the given function descriptor has its pointer to the init
 *        function set, then this initialization function will be called
 *        once before the first call to the step function.
 */
void
detector_adv_detector_detectDevicesFunctionWrapper_execute
(
    void* userFunctionDescRaw
);

XME_EXTERN_C_END

#endif // #ifndef DETECTOR_ADV_DETECTOR_DETECTDEVICESFUNCTIONWRAPPER_H
