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
#ifdef __cplusplus
extern "C" { 
#endif
	int startGui(int argc, char *argv[]);
	void writeTextToGui(char* text);
	int initRos(int argc, char *argv[]);
#ifdef __cplusplus
}
#endif
void writeButtonStateWrapper(int state);
void terminateApplicationWrapper(void);
