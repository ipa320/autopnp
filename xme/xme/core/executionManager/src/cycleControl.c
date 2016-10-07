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
 * $Id: cycleControl.c 7459 2014-02-18 10:25:58Z geisinger $
 */

/**
 * \file
 *         Scheduler cycle counter operations.
 */

#define MODULE_ACRONYM "ExecMgr   : "

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/defines.h"
#include "xme/core/executionManager/include/executionManagerComponentRepositoryInterface.h"
#include "xme/core/executionManager/include/executionManagerScheduleManagementInterface.h"
#include "xme/core/executionManager/include/executionManagerIntern.h"

#include "xme/core/dataHandler/include/dataHandlerConfigurator.h"

#include "xme/core/topicData.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/time.h"

#include <inttypes.h>

xme_core_dataManager_dataPacketId_t xme_core_exec_cycleCounter = XME_CORE_DATAMANAGER_DATAPACKETID_INVALID;

xme_status_t
xme_core_exec_scheduler_incrementCycleCounter(void);

/****************************************************************************/
xme_status_t
xme_core_exec_resetDbCycleCounter(void)
{
	if(xme_core_exec_intConfig.useDbCycleCounter)
	{
		xme_core_topic_exec_cycleCounter_t cycleCounterBuf;

		cycleCounterBuf.cycleCounter = 0;
		XME_CHECK(XME_STATUS_SUCCESS ==
				xme_core_dataHandler_writeData(
						1U,
						&cycleCounterBuf,
						sizeof(cycleCounterBuf)),
				XME_STATUS_INTERNAL_ERROR
		);
	}

    return XME_STATUS_SUCCESS;
}

xme_core_dataManager_dataPacketId_t
xme_core_exec_initDbCycleCounter(void)
{
	XME_CHECK(XME_STATUS_SUCCESS ==
		xme_core_dataHandler_createDataPacket
		(
			sizeof(xme_core_topic_exec_cycleCounter_t),
			&xme_core_exec_cycleCounter
		),
		XME_CORE_DATAMANAGER_DATAPACKETID_INVALID
	);

	XME_CHECK(XME_STATUS_SUCCESS ==
		xme_core_dataHandler_setDataPacketPersistent
		(
			xme_core_exec_cycleCounter
		),
		XME_CORE_DATAMANAGER_DATAPACKETID_INVALID
	);

	XME_CHECK(XME_STATUS_SUCCESS == xme_core_dataHandler_configure(), XME_CORE_DATAMANAGER_DATAPACKETID_INVALID);

	return xme_core_exec_cycleCounter;
}

/****************************************************************************/

xme_status_t
xme_core_exec_scheduler_incrementCycleCounter(void)
{
    xme_core_topic_exec_cycleCounter_t cycleCounterBuf;
    unsigned int bytesRead = 0;
    uint32_t cycleCount = xme_core_exec_dispatcher_incCycleCount();

    XME_LOG(XME_LOG_VERBOSE, MODULE_ACRONYM
            "############################### CycleInt  %3"PRIu64" ######################################\n",
            cycleCount);

    if(xme_core_exec_intConfig.useDbCycleCounter)
    {
		XME_CHECK(XME_STATUS_SUCCESS ==
			xme_core_dataHandler_readData(
					1U,
					&cycleCounterBuf,
					sizeof(xme_core_topic_exec_cycleCounter_t), //
					&bytesRead),
			XME_STATUS_INTERNAL_ERROR
		);

		cycleCounterBuf.cycleCounter++;

		XME_LOG(XME_LOG_VERBOSE, MODULE_ACRONYM
				"############################### CycleDB   %3d ######################################\n",
				cycleCounterBuf.cycleCounter);

		XME_CHECK(XME_STATUS_SUCCESS ==
			xme_core_dataHandler_writeData(
					1U,
					&cycleCounterBuf,
					sizeof(xme_core_topic_exec_cycleCounter_t)),
			XME_STATUS_INTERNAL_ERROR
		);
    }

    return XME_STATUS_SUCCESS;
}


/****************************************************************************/
uint32_t
xme_core_exec_scheduler_getCycleCounter(void)
{
	if(xme_core_exec_intConfig.useDbCycleCounter)
	{
		xme_core_topic_exec_cycleCounter_t cycleCounterBuf;
		unsigned int bytesRead = 0;
		xme_status_t status;

		status = xme_core_dataHandler_readData(1U,
				&cycleCounterBuf, sizeof(xme_core_topic_exec_cycleCounter_t), &bytesRead);

		XME_ASSERT(XME_STATUS_SUCCESS == status);
		XME_ASSERT(sizeof(xme_core_topic_exec_cycleCounter_t) == bytesRead);

		return cycleCounterBuf.cycleCounter;
	}
	else
	{
		return xme_core_exec_dispatcher_getCycleCount();
	}
}
