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

#include "autoPnP/adv/capabilitiesView/include/capabilitiesWindow.h"

#include "xme/core/executionManager/include/executionManager.h" //TODO

#include "ui_capabilitiesWindow.h"
#include <QtWebKit/QWebFrame>

typedef struct {
	Ui::CapabilitiesWindow* ui;
	std::string* filepath;
} executeParameter_t;

void updateCapabilitiesImage(void* userData);

CapabilitiesWindow::CapabilitiesWindow(std::string filepath, QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::CapabilitiesWindow)
{
	ui->setupUi(this);
	this->filepath = filepath;

	updateCapabilities();
}

CapabilitiesWindow::~CapabilitiesWindow()
{
	delete ui;
}

void CapabilitiesWindow::updateCapabilities()
{
	if(!this) return;

	static executeParameter_t parameter = {ui, &filepath};

	xme_hal_qt_triggerExecution(updateCapabilitiesImage, &parameter);
}

void CapabilitiesWindow::closeEvent (QCloseEvent *event)
{
	//TODO there is a bit of trouble here.
	//we will stop at the end of this cycle but what about if the current request is processed at the beginning of next cycle?
	//that is the ack is sent in next cycle, then it will never hit the wire
	xme_core_exec_stop(true);
}

void updateCapabilitiesImage(void* userData)
{
	executeParameter_t* parameter = (executeParameter_t*)userData;

	QPixmap capabilities(750, 550);
	capabilities.fill(Qt::white);
	capabilities.load(QString(parameter->filepath->c_str()));

	parameter->ui->webView->setUrl(QUrl(parameter->filepath->c_str()));
}
