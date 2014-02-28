/*
 * Copyright (c) 2011-2013, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: dateTime.c 2135 2013-01-14 17:56:58Z geisinger $
 */

/**
 * \file
 *         Date/time utility.
 *
 *         Prints the current date and/or time or the modification time of a
 *         given file in a user-defined format.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
void
printUsage(void)
{
	printf(
		"Usage: date-time <format> [ <filename> ]\n"
		"  <format>   is the date/time format according to strftime()\n"
		"  <filename> is the file whose modification time to use\n"
		"             (if omitted, current date/time is printed)\n"
	);
}

void
printVersion(void)
{
	printf("date-time version %d.%d.%d\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

void
toLocalTime
(
	struct tm* _tm,
	const time_t* time
)
{
#ifdef _MSC_VER
	localtime_s(_tm, time);
#else
	struct tm* temp = NULL;
	temp = localtime(time);
	memcpy(_tm, temp, sizeof(struct tm));
#endif
}

int
printCurrentTime
(
	const char* format
)
{
	time_t t;
	struct tm local;
	char buf[256];

	time(&t);
	toLocalTime(&local, &t);

	strftime(buf, 256, format, &local);

	printf("%s\n", buf);

	return 0;
}

int
printFileModificationTime
(
	const char* format,
	const char* filename
)
{
	struct stat st;
	struct tm local;
	char buf[256];

	if (stat(filename, &st))
	{
		return 2;
	}

	toLocalTime(&local, &st.st_mtime);

	strftime(buf, 256, format, &local);

	printf("%s\n", buf);

	return 0;
}

int
main
(
	int argc,
	char* argv[]
)
{
	if (2 == argc)
	{
		if (!strcmp(argv[1], "--help"))
		{
			printUsage();
			return 0;
		}
		else if (!strcmp(argv[1], "--version"))
		{
			printVersion();
			return 0;
		}
		else
		{
			return printCurrentTime(argv[1]);
		}
	}
	else if (3 == argc)
	{
		return printFileModificationTime(argv[1], argv[2]);
	}
	else
	{
		printUsage();
		return 1;
	}
}
