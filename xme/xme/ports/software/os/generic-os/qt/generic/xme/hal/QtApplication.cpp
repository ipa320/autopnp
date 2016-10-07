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
 * $Id: QtApplication.cpp 6587 2014-02-02 12:27:34Z kainz $
 */
/**
 * \file
 *         Qt application.
 *
 * \author
 *         Gerd Kainz <kainz@fortiss.org>
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "xme/hal/QtApplication.h"

#include <QMetaType>


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
QtApplication::QtApplication(int &argc, char **argv) :
	QApplication(argc, argv)
{
	qRegisterMetaType<xme_hal_qt_callback_t>("xme_hal_qt_callback_t");

	connect(this, SIGNAL(executeSignal(xme_hal_qt_callback_t, void*)), this, SLOT(execute(xme_hal_qt_callback_t, void*)));
}

QtApplication::~QtApplication()
{
}

void QtApplication::triggerExecution(xme_hal_qt_callback_t callback, void* userData)
{
	emit executeSignal(callback, userData);
}

void QtApplication::execute(xme_hal_qt_callback_t callback, void* userData)
{
	callback(userData);
}
