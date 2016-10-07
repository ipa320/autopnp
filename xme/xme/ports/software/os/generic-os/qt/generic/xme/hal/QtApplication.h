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
 * $Id: QtApplication.h 6587 2014-02-02 12:27:34Z kainz $
 */

/**
 * \file
 *         Qt application.
 *
 * \author
 *         Gerd Kainz <kainz@fortiss.org>
 */

#ifndef XME_HAL_QT_QTAPPLICATION_H
#define XME_HAL_QT_QTAPPLICATION_H

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <QtGui/QApplication>

#include "xme/hal/include/qt.h"


/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/
namespace Ui
{
	class QtApplication;
}


/******************************************************************************/
/***   Type definitions                                                     ***/
/******************************************************************************/
/**
 * \typedef xme_hal_qt_callback_t
 *
 * \brief   Callback executed by GUI thread.
 */
typedef void (*xme_hal_qt_callback_t) (void* userData);

/**
 * \typedef QtApplication
 *
 * \brief   Qt application class adding callback mechanism.
 */
class QtApplication : public QApplication
{
	Q_OBJECT

	public:
		explicit QtApplication(int &argc, char **argv);
		~QtApplication();
		void triggerExecution(xme_hal_qt_callback_t callback, void* userData);

	signals:
		void executeSignal(xme_hal_qt_callback_t callback, void* userData);

	public slots:
		void execute(xme_hal_qt_callback_t callback, void* userData);
};

#endif // #ifndef XME_HAL_QT_QTAPPLICATION_H
