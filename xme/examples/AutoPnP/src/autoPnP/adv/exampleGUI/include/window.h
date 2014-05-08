/*
 * Copyright (c) 2011 - 2013, fortiss GmbH.
 * All rights reserved.
 *
 * This file is part of AutoPnP.
 *
 * $Id$
 */

/**
 * \file
 *         Capabilities window.
 *
 * \author
 *         Gerd Kainz <kainz@fortiss.org>
 */

#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>
#include <string>

#include "xme/hal/include/qt.h"

namespace Ui
{
	class Window;
}

class Window : public QMainWindow
{
	Q_OBJECT

	public:
		explicit Window(QWidget *parent = 0);
		~Window();
		void closeEvent (QCloseEvent *event);

	private slots:
		void addComponent();

	private:
		Ui::Window* ui;
};

#endif // #ifndef WINDOW_H
