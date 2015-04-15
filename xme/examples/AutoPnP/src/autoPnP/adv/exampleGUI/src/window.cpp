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
 *         Window.
 *
 * \author
 *         Gerd Kainz <kainz@fortiss.org>
 */

#include "autoPnP/adv/exampleGUI/include/window.h"

#include "xme/core/plugAndPlay/include/plugAndPlayClient.h"

#include "autoPnP/adv/exampleGUI/include/exampleGUIComponentWrapper.h"
#include "autoPnP/topic/autoPnPCommonData.h"

#include "ui_window.h"

#ifndef WIN32
void strcpy_s(char *dst, size_t n, const char *src) {
	strncpy(dst, src,n);
}
#endif

Window::Window(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::Window)
{
	ui->setupUi(this);

	connect(ui->addButton, SIGNAL(clicked()), this, SLOT(addComponent()));
}

Window::~Window()
{
	delete ui;
}

void Window::closeEvent (QCloseEvent *event)
{
	xme_core_pnp_pnpClient_logoutThisNode();
}

void Window::addComponent()
{
	AutoPnP_topic_add_component_t addComponent;
	xme_status_t status;

	if (strcmp(ui->componentName->text().toAscii(), ""))
	{
		strcpy_s(addComponent.parent, sizeof(addComponent.parent), "");
		strcpy_s(addComponent.name, sizeof(addComponent.name), ui->componentName->text().toAscii());
		ui->componentName->setText("");
		addComponent.capabilitiesCount = 0;

		status = autoPnP_adv_exampleGUI_exampleGUIComponentWrapper_writePortAddComponent(&addComponent);
		if (XME_STATUS_SUCCESS != status)
		{
			XME_LOG(XME_LOG_ERROR, "Window::addComponent: Error sending!\n");
		}

		autoPnP_adv_exampleGUI_exampleGUIComponentWrapper_completeWriteOperations();
	}
}
