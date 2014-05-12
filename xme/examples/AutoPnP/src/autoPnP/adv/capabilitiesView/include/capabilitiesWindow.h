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

#ifndef CAPABILITIESWINDOW_H
#define CAPABILITIESWINDOW_H

#include <QMainWindow>
#include <string>

#include "xme/hal/include/qt.h"

namespace Ui
{
	class CapabilitiesWindow;
}

class CapabilitiesWindow : public QMainWindow
{
	Q_OBJECT

	public:
		explicit CapabilitiesWindow(std::string filepath, QWidget *parent = 0);
		~CapabilitiesWindow();
		void closeEvent (QCloseEvent *event);
		void updateCapabilities();

	private:
		Ui::CapabilitiesWindow* ui;
		std::string filepath;
};

#endif // #ifndef CAPABILITIESWINDOW_H
