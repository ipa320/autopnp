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
 * $Id: graphviz_arch.cpp 7493 2014-02-19 14:47:04Z kainz $
 */

/**
 * \file
 *         Graphviz abstraction (platform specific part: generic OS based
 *         implementation).
 */

/**
 * \addtogroup hal_graphviz
 * @{
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/include/graphviz.h"

#include <graphviz/gvc.h>
#include <stdint.h>

#include "xme/hal/include/sync.h"

#ifndef WIN32
int fopen_s(FILE **fp, const char *fn, const char *o) {
	*fp = fopen(fn,o);
	return *fp?0:1;
}
#endif


/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
static uint8_t xme_hal_graphviz_initializationCount = 0;
static GVC_t* xme_hal_graphviz_gvc = NULL;
static xme_hal_sync_criticalSectionHandle_t xme_hal_graphviz_mutex = XME_HAL_SYNC_INVALID_CRITICAL_SECTION_HANDLE;


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
xme_hal_graphviz_init(void)
{
	if (0 == xme_hal_graphviz_initializationCount)
	{
		//Init graphviz;
		xme_hal_graphviz_gvc = gvContext();
		xme_hal_graphviz_mutex = xme_hal_sync_createCriticalSection();

		xme_hal_graphviz_initializationCount++;

		return XME_STATUS_SUCCESS;
	}
	else
	{
		xme_hal_graphviz_initializationCount++;

		return XME_STATUS_SUCCESS;
	}
}

xme_status_t
xme_hal_graphviz_generateImage
(
    const char* dotFilePath,
    const char* pngFilePath
)
{
	FILE *fp;
	graph_t *graph;

	fopen_s(&fp, dotFilePath, "r");
	XME_CHECK(fp != NULL, XME_STATUS_INTERNAL_ERROR);

	xme_hal_sync_enterCriticalSection(xme_hal_graphviz_mutex);
	graph = agread_usergets(fp, fgets);
	gvLayout(xme_hal_graphviz_gvc, graph, "dot");
	gvRenderFilename(xme_hal_graphviz_gvc, graph, "png", pngFilePath);
	gvFreeLayout(xme_hal_graphviz_gvc, graph);
	agclose(graph);
	xme_hal_sync_leaveCriticalSection(xme_hal_graphviz_mutex);

	fclose(fp);

	return XME_STATUS_SUCCESS;
}

void
xme_hal_graphviz_fini(void)
{
	if (0 < xme_hal_graphviz_initializationCount)
	{
		xme_hal_graphviz_initializationCount--;

		if (0 == xme_hal_graphviz_initializationCount)
		{
			//Finalize graphviz
			xme_hal_sync_destroyCriticalSection(xme_hal_graphviz_mutex);
			xme_hal_graphviz_gvc = NULL;
		}
	}
}

/**
 * @}
 */
