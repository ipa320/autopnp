/*
 * Copyright (c) 2011, fortiss GmbH.
 * All rights reserved.
 *
 * $Id$
 */

/**
 * \file
 *         GUI Header
 *
 * \author
 *         Nadine Keddis <keddis@fortiss.org>
 */

#include <QtGui/QApplication>
#include "mainwindow.h"
#include "wrapperHeader.h"

#include "ros_code.h"
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
#include "headerFile.h"
#ifdef __cplusplus
}
#endif

MainWindow *w = 0;
int window_initialized = 0;

extern "C" int startGui(int argc, char *argv[])
{
	QApplication a(argc, argv);
	w = new MainWindow();
    w->show();

    window_initialized = 1;

    return a.exec();
}

extern "C" void writeTextToGui(char* text)
{
	w->emitWriteTextSignal(text);
}

void writeButtonStateWrapper(int state)
{
	writeButtonState(state);
}

void displayImage(unsigned int width, unsigned int height, unsigned int step, const std::vector<unsigned char>& data)
{
	w->emitDisplayImage(width, height, step, data);
}

extern "C" int initRos(int argc, char *argv[])
{
	while (window_initialized == 0)
		sleep(0.01);
	void (*displayImageFunction)(unsigned int, unsigned int, unsigned int, const std::vector<unsigned char>& data);
	displayImageFunction = displayImage;
	RosInit rosInit(argc, argv, displayImageFunction);
}
