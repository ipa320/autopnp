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
 * $Id: base64ioTest.cpp 6459 2014-01-23 16:50:54Z geisinger $
 */

/**
 * \file
 *         File input/output unit test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/defines.h"
#include "xme/core/component.h"
#include "xme/core/dataManagerTypes.h"

#include "xme/hal/include/base64io.h"
#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"
#include "xme/hal/include/fileio.h"



/******************************************************************************/
/***   DataStructures                                                       ***/
/******************************************************************************/

typedef enum
{
    XME_CORE_DATAHANDLER_PORT_CREATING,
    XME_CORE_DATAHANDLER_PORT_ACTIVE,
    XME_CORE_DATAHANDLER_PORT_INACTIVE,
    XME_CORE_DATAHANDLER_PORT_DESTROYING
}
xme_core_dataHandler_port_status_t;

typedef struct
{
    /**
     * This data structure will contain the information regarding the port operation.
     * All this information will be logged to a log file in the base64 encoding.
     * Total size of the string in the encoded format will be 48 bytes, and will
     * decode to 64 bytes in ASCII.
     */
    time_t timestamp;
    xme_core_component_t componentId;
    char componentName[36];
    xme_core_dataManager_dataPacketId_t portId;
    xme_core_component_portType_t portType;
    xme_core_topic_t portTopic;
    xme_core_dataHandler_port_status_t status;
}
xme_core_dataHandler_port_log_t;

#define XME_CORE_DATAHANDLER_PORT_AUDITLOG_FILENAME "xme_portAudit.log"

static void
xme_core_dataHandler_portActivity_record
(
    xme_core_dataHandler_port_log_t * log
)
{
    xme_hal_fileio_fileHandle_t fp;
    char str[65];

    fp = xme_hal_fileio_fopen(XME_CORE_DATAHANDLER_PORT_AUDITLOG_FILENAME, XME_HAL_FILEIO_MODE_WRITEONLY_BINARY);

    xme_hal_safeString_snprintf(str, 15, "%10lu %2d ", log->timestamp, log->componentId); 
    xme_hal_safeString_snprintf(str+14, 34, "%33s", log->componentName);
    xme_hal_safeString_snprintf(str+47, 18," %3d %2d %6d %2d", log->portId, log->portType,
                                                    log->portTopic, log->status);
    str[64] = '\0';
    xme_hal_base64io_fprintf(fp, "%s", str);

    /*
    xme_hal_base64io_fprintf(fp, "%10lu %2d %33s %3d %2d %6d %2d", log->timestamp, log->componentId, 
                                                    log->componentName, log->portId, log->portType,
                                                    log->portTopic, log->status);
*/
    xme_hal_fileio_fclose(fp);
}

static void
xme_core_dataHandler_portActivity_read
(
    xme_core_dataHandler_port_log_t * log
)
{
    xme_hal_fileio_fileHandle_t fp;
    char *str;
    char timestamp_str[12];
    char componentId_str[4];
    char portId_str[5];
    char portType_str[4];
    char portTopic_str[8];
    char status_str[4];

    fp = xme_hal_fileio_fopen(XME_CORE_DATAHANDLER_PORT_AUDITLOG_FILENAME, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    
    str = (char *)xme_hal_mem_alloc(sizeof(char)*66);
    xme_hal_base64io_read_file(fp, &str, 64);
    
    xme_hal_safeString_strncpy(timestamp_str, str, 11);
    timestamp_str[11] = '\0';
    log->timestamp = (time_t) strtoul(timestamp_str, NULL, 10);

    xme_hal_safeString_strncpy(componentId_str, str+11, 3);
    componentId_str[2] = '\0';
    log->componentId = (xme_core_component_t) atoi(componentId_str);

    xme_hal_safeString_strncpy(log->componentName, str+14, 34);
    log->componentName[35] = '\0';

    xme_hal_safeString_strncpy(portId_str, str+48, 4);
    portId_str[3] = '\0';
    log->portId = (xme_core_dataManager_dataPacketId_t) atoi(portId_str);
    
    xme_hal_safeString_strncpy(portType_str, str+52, 3);
    portType_str[2] = '\0';
    log->portType = (xme_core_component_portType_t) atoi(portType_str);

    xme_hal_safeString_strncpy(portTopic_str, str+55, 7);
    portTopic_str[6] = '\0';
    log->portTopic = (xme_core_topic_t) atoi(portTopic_str);

    xme_hal_safeString_strncpy(status_str, str+62, 3);
    status_str[2] = '\0';
    log->status = (xme_core_dataHandler_port_status_t) atoi(status_str);

    xme_hal_mem_free(str);

    xme_hal_fileio_fclose(fp);
}


/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
class Base64EncodeDecodeIntegratoionTest: public ::testing::Test
{
protected:
    Base64EncodeDecodeIntegratoionTest()
    {
        xme_hal_safeString_snprintf(input_str, 12, "%s", "Hello World");        
    }

    virtual ~Base64EncodeDecodeIntegratoionTest()
    {
    }

    char input_str[20];
};

class Base64FileIOIntegrationTest: public ::testing::Test
{
protected:
    Base64FileIOIntegrationTest()
    {
        xme_hal_safeString_snprintf(input_str, 12, "%s", "Hello World");        
    }

    virtual ~Base64FileIOIntegrationTest()
    {
    }

    char input_str[20];
};

class Base64PortAuditLogFileIOIntegrationTest: public ::testing::Test
{
protected:
    Base64PortAuditLogFileIOIntegrationTest()
    {
        time(&log.timestamp);
        log.componentId = (xme_core_component_t) 0;
        xme_hal_mem_set(log.componentName, 0, 33);
        xme_hal_safeString_strncpy(log.componentName, "trial_component", 15);
        log.componentName[15] = '\0';
        log.portId = (xme_core_dataManager_dataPacketId_t) 0;
        log.portType = XME_CORE_COMPONENT_PORTTYPE_DCC_SUBSCRIPTION;
        log.portTopic = (xme_core_topic_t) 5001;
        log.status = XME_CORE_DATAHANDLER_PORT_CREATING;
    }

    virtual ~Base64PortAuditLogFileIOIntegrationTest()
    {
    }

    xme_core_dataHandler_port_log_t log;
    xme_core_dataHandler_port_log_t read_log;
};

TEST_F(Base64EncodeDecodeIntegratoionTest, encodeAndDecodeString)
{
    size_t encoded_strlen=0;
    string_b64* encoded_str;
    size_t decoded_strlen=0;
    char* decoded_str;

    encoded_str = (string_b64 *) xme_hal_mem_alloc(strlen_after_encoding(strlen(input_str)));
    encoded_strlen = xme_hal_base64io_encode(input_str, &encoded_str);
    EXPECT_GT((int)encoded_strlen, 0);

    decoded_str = (char *) xme_hal_mem_alloc(strlen_after_decoding(strlen(input_str)));
    decoded_strlen = xme_hal_base64io_decode(encoded_str, &decoded_str);
    EXPECT_GT((int)decoded_strlen, 0);
    EXPECT_EQ(0, strncmp(input_str, decoded_str, 11));

    xme_hal_mem_free(encoded_str);
    xme_hal_mem_free(decoded_str);
}

TEST_F(Base64FileIOIntegrationTest, writeAndReadBackFromAFile)
{
    xme_hal_fileio_fileHandle_t fp;
    size_t encoded_strlen = 0;
    char *decoded_str;
    size_t decoded_strlen;

    fp = xme_hal_fileio_fopen("b.txt", XME_HAL_FILEIO_MODE_WRITEONLY);
    encoded_strlen = xme_hal_base64io_fprintf(fp, "%s", input_str);
    EXPECT_GT((int)encoded_strlen, 0);
    xme_hal_fileio_fclose(fp);

    fp= NULL;
    fp = xme_hal_fileio_fopen("b.txt", XME_HAL_FILEIO_MODE_READONLY);
    decoded_str = (char *)xme_hal_mem_alloc(strlen_after_decoding(encoded_strlen));
    decoded_strlen = xme_hal_base64io_read_file(fp, &decoded_str, strlen_after_decoding(encoded_strlen));
    EXPECT_GT((int)decoded_strlen, 0);
    xme_hal_fileio_fclose(fp);

    EXPECT_EQ(0, strncmp(decoded_str, input_str, 11));

    xme_hal_mem_free(decoded_str);
}

TEST_F(Base64PortAuditLogFileIOIntegrationTest, writePortAuditLogToFileAndReadBack)
{

    xme_core_dataHandler_portActivity_record(&log);
    xme_core_dataHandler_portActivity_read(&read_log);

    EXPECT_EQ(log.timestamp, read_log.timestamp);
    EXPECT_EQ(log.componentId, read_log.componentId);
    EXPECT_EQ(log.portId, read_log.portId);
    EXPECT_EQ(log.portType, read_log.portType);
    EXPECT_EQ(log.portTopic, read_log.portTopic);
    EXPECT_EQ(log.status, read_log.status);
}

//TEST(XMEHalBase64ioTest, xme_tests_hal_base64io)

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
