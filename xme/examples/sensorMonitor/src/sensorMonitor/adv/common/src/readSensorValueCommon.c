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
 * $Id: readSensorValueCommon.c 6157 2013-12-18 16:17:39Z geisinger $
 */

/**
 * \file
 *         Source file for common functionality in readSensorValue functions.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "sensorMonitor/adv/common/include/readSensorValueCommon.h"

#include "sensorMonitor/adv/sensorB/include/sensorBComponent.h"

#include "xme/core/log.h"

#include "xme/hal/include/safeString.h"

#ifndef WIN32
#include <mntent.h>
#include <unistd.h>
#include <sys/statvfs.h>
#include <sys/stat.h>
#else // #ifndef WIN32
#include <Windows.h>
#endif  // #ifndef WIN32

/******************************************************************************/
/***   Helper functions                                                     ***/
/******************************************************************************/

void
sensorMonitor_adv_common_fillSensorType
(
    xme_core_component_config_t* const componentConfig
)
{
    sensorMonitor_adv_sensorB_sensorBComponent_config_t* config = (sensorMonitor_adv_sensorB_sensorBComponent_config_t*) componentConfig;

    config->sensorType = "Free space";

    XME_LOG(XME_LOG_ALWAYS, "Sensor to be monitored: %s\n\n", config->sensorType);
}

void
sensorMonitor_adv_common_fillSensorHostName
(
    xme_core_component_config_t* const componentConfig
)
{
    sensorMonitor_adv_sensorB_sensorBComponent_config_t* config = (sensorMonitor_adv_sensorB_sensorBComponent_config_t*) componentConfig;
    
    // FIXME: Move to component initialization!
    gethostname(config->hostName, sizeof(config->hostName));
}

void
sensorMonitor_adv_common_fillSensorTopic
(
    xme_core_component_config_t* const componentConfig
)
{
    sensorMonitor_adv_sensorB_sensorBComponent_config_t* config = (sensorMonitor_adv_sensorB_sensorBComponent_config_t*) componentConfig;

#ifndef WIN32
    // Linux implementation

    int driveCount = 0;
    int selectedDrive = 0;
    FILE* fp;
    struct mntent *ent;

    fp = setmntent("/etc/mtab", "r");
    while ((ent = getmntent(fp)) != NULL)
    {
        if (strcmp(ent->mnt_fsname, "none") == 0 || strcmp(ent->mnt_fsname, "sysfs") == 0 || strcmp(ent->mnt_fsname, "proc") == 0 ||
            strcmp(ent->mnt_fsname, "gvfs-fuse-daemon") == 0 || strcmp(ent->mnt_fsname, "devpts") == 0)
        {
            // Ignore this partition
            continue;
        }

        // List this partition
        XME_LOG(XME_LOG_ALWAYS, "[%d] %s %s\n", driveCount++, ent->mnt_dir, ent->mnt_type);
    }
    endmntent(fp);

    do
    {
        XME_LOG(XME_LOG_ALWAYS, "Which partition do you wish to monitor? [0-%d]: ", driveCount-1);
        if (!scanf("%d", &selectedDrive))
        {
            selectedDrive = -1;
        }
    }
    while (selectedDrive < 0 || selectedDrive> driveCount-1);

    driveCount = 0;
    fp = setmntent("/etc/mtab", "r");
    while ((ent = getmntent(fp)) != NULL)
    {
        if (strcmp(ent->mnt_fsname, "none") == 0 || strcmp(ent->mnt_fsname, "sysfs") == 0 || strcmp(ent->mnt_fsname, "proc") == 0 ||
            strcmp(ent->mnt_fsname, "gvfs-fuse-daemon") == 0 || strcmp(ent->mnt_fsname, "devpts") == 0)
        {
            // Ignore this partition
            continue;
        }

        // Check for matching drive
        if (driveCount++ == selectedDrive)
        {
            xme_hal_safeString_strncpy(config->sensorInstance, ent->mnt_dir, sizeof(config->sensorInstance));
            break;
        }
    }
    endmntent(fp);

#else // #ifndef WIN32
    // Windows implementation

    char szBuffer[1024];
    int driveCount = 0;
    int selectedDrive = 0;
    unsigned int i;
    DWORD length;

    length = GetLogicalDriveStrings(1024, (LPTSTR)szBuffer);
    XME_CHECK_MSG(0 != length, XME_CHECK_RVAL_VOID, XME_LOG_ERROR, "GetLogicalDriveStrings() failed: %d\n", GetLastError());

    XME_LOG(XME_LOG_ALWAYS, "The logical drives of this machine are:\n");
    for (i = 0U; i < length;)
    {
        if (szBuffer[i] != 0)
        {
            UINT type = GetDriveType((LPCSTR)(&szBuffer[i]));
            if (DRIVE_REMOVABLE != type && DRIVE_FIXED != type && DRIVE_RAMDISK != type)
            {
                // Ignore this drive
                i += strlen(&szBuffer[i]) + 1U;
                continue;
            }

            // List this drive
            XME_LOG(XME_LOG_ALWAYS, "[%d] : %s\n", driveCount++, (LPTSTR)(&szBuffer[i]));
            i += strlen(&szBuffer[i]) + 1U;
        }

        if (0 == szBuffer[i+1])
        {
            i++;
        }
    }

    do
    {
        XME_LOG(XME_LOG_ALWAYS, "\nWhich drive do you wish to monitor? [0-%d]: ", driveCount-1);
        scanf_s("%d", &selectedDrive);
    }
    while (selectedDrive < 0 || selectedDrive > driveCount-1);
    XME_LOG(XME_LOG_ALWAYS, "\n");

    driveCount = 0;
    for (i = 0; i < length;)
    {
        if (szBuffer[i] != 0)
        {
            UINT type = GetDriveType((LPCSTR)(&szBuffer[i]));
            if (DRIVE_REMOVABLE != type && DRIVE_FIXED != type && DRIVE_RAMDISK != type)
            {
                // Ignore this drive
                i += strlen(&szBuffer[i]) + 1;
                continue;
            }

            // Check for matching drive
            if(driveCount++ == selectedDrive)
            {
                xme_hal_safeString_strncpy(config->sensorInstance, (LPTSTR)(&szBuffer[i]), sizeof(config->sensorInstance));
                break;
            }
            i += strlen(&szBuffer[i]) + 1;
        }

        if (0 == szBuffer[i+1])
        {
            i++;
        }
    }

#endif // #ifndef WIN32
}

uint64_t
sensorMonitor_adv_common_fillSensorValue
(
    const xme_core_component_config_t* const componentConfig
)
{
    sensorMonitor_adv_sensorB_sensorBComponent_config_t* config = (sensorMonitor_adv_sensorB_sensorBComponent_config_t*) componentConfig;

    uint64_t sensorValue = 0ULL;

#ifndef WIN32
    struct statvfs buf;
    statvfs(config->sensorInstance, &buf);
    sensorValue = ((uint64_t) buf.f_bsize * (uint64_t) buf.f_bfree);
#else
    DWORD dwSectPerClust, dwBytesPerSect, dwFreeClusters, dwTotalClusters;
    GetDiskFreeSpace(config->sensorInstance, &dwSectPerClust,&dwBytesPerSect, &dwFreeClusters, &dwTotalClusters);
    sensorValue = (__int64) dwFreeClusters * dwSectPerClust * dwBytesPerSect;
#endif

    return sensorValue;
}
