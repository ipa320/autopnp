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
 * $Id: fileioTest.cpp 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         File input/output unit test.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>
#include "xme/hal/include/fileio.h"

#include "xme/hal/include/mem.h"
#include "xme/hal/include/safeString.h"

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

struct container_info_t {
    char component_name[32];
    char shared_library_path[256];
    bool auto_start;
    bool start_on_network_connection;
};

#define MAX_CONTAINERS 10
#define CONTAINER_INFO_MEMBERS_END 4

#define CONTAINER_INFO_COMPONENT_NAME 0
#define CONTAINER_INFO_SHARED_LIBRARY_PATH 1
#define CONTAINER_INFO_AUTO_START 2
#define CONTAINER_INFO_START_ON_NETWORK_CONNECTION 3

container_info_t containers[MAX_CONTAINERS];
int num_of_containers_in_array = 0;

//char buffer[16];

int get_token(char* buffer, int buffer_size, char* token, FILE** fp)
{
    static int read_size = 0;
    static int parser_head = 0;
    int token_length = 0;

    while(1) {
        if(read_size - parser_head <= 0) {
            read_size = xme_hal_fileio_fread(buffer, sizeof(char), buffer_size, *fp);
            if(read_size == 0)
            {
                token_length = 0;
                token[token_length] = EOF;
                read_size = 0;
                parser_head = 0;
                break;
            }
            parser_head = 0;
        }

//TODO: Handle Mac Line End Character
        if(buffer[parser_head] == ',' || buffer[parser_head] == '\n') {
            token[token_length++] = '\0';
            parser_head++;
            break;
        }
        else if(buffer[parser_head] != '\r')
        {
            token[token_length++] = buffer[parser_head];
        }
        parser_head++;
    }

    return token_length;
}


TEST(FileIo, xme_hal_fileio_test_basic)
{
    int write_size;
    int read_size;
    char buf[32];
    xme_hal_fileio_fileHandle_t fh;
    xme_status_t status;

    // Create a new file and write to it
    fh = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_test_file.txt", XME_HAL_FILEIO_MODE_WRITE_UPDATE);
    ASSERT_FALSE(XME_HAL_FILEIO_INVALID_FILE_HANDLE == fh);
    write_size = xme_hal_fileio_fwrite("Hello World", sizeof(char), 11, fh);
    EXPECT_GT(write_size, 0);
    status = xme_hal_fileio_fclose(fh);
    EXPECT_TRUE(XME_STATUS_SUCCESS == status);
    fh = XME_HAL_FILEIO_INVALID_FILE_HANDLE;

    // Read the file and check its contents
    fh = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_test_file.txt", XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_FALSE(XME_HAL_FILEIO_INVALID_FILE_HANDLE == fh);
    read_size = xme_hal_fileio_fread(buf, 1, 32, fh);
    EXPECT_GT(read_size, 0);
    EXPECT_EQ(0, strncmp(buf, "Hello World", 11));
    status = xme_hal_fileio_fclose(fh);
    EXPECT_TRUE(XME_STATUS_SUCCESS == status);
    fh = XME_HAL_FILEIO_INVALID_FILE_HANDLE;

    //Read a CSV file and populate containers array
    fh = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_1.txt", XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_FALSE(XME_HAL_FILEIO_INVALID_FILE_HANDLE == fh);
    char *token;
    token = (char*)xme_hal_mem_alloc(sizeof(char)*256);
    char *buffer;
    buffer = (char*)xme_hal_mem_alloc(sizeof(char)*16);
    int token_length = 0;
    token_length = get_token(buffer, 16, token, &fh);
    int i=0;
    while(token[0] != EOF)
    {
        switch(i)
        {
        case CONTAINER_INFO_COMPONENT_NAME: xme_hal_safeString_strncpy(containers[num_of_containers_in_array].component_name, token, token_length);
            break;
        case CONTAINER_INFO_SHARED_LIBRARY_PATH: xme_hal_safeString_strncpy(containers[num_of_containers_in_array].shared_library_path, token, token_length);
            break;
        case CONTAINER_INFO_AUTO_START: containers[num_of_containers_in_array].auto_start = (atoi(token) != 0);
            break;
        case CONTAINER_INFO_START_ON_NETWORK_CONNECTION: containers[num_of_containers_in_array].start_on_network_connection = (atoi(token) != 0);
            break;
        default: break;
        }
        i++;
        if(i == CONTAINER_INFO_MEMBERS_END)
        {
            i=0;
            num_of_containers_in_array++;
        }
        token_length = get_token(buffer, 16, token, &fh);
    }
    xme_hal_mem_free(buffer);
    xme_hal_mem_free(token);
    status = xme_hal_fileio_fclose(fh);
    EXPECT_TRUE(XME_STATUS_SUCCESS == status);
    fh = XME_HAL_FILEIO_INVALID_FILE_HANDLE;

    EXPECT_EQ(0, strncmp(containers[0].component_name, "ComponentA", 10));
    EXPECT_EQ(0, strncmp(containers[0].shared_library_path, "0", 1));
    EXPECT_TRUE(containers[0].auto_start);
    EXPECT_TRUE(containers[0].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[1].component_name, "ComponentB", 10));
    EXPECT_EQ(0, strncmp(containers[1].shared_library_path, "Container1.dll", 14));
    EXPECT_TRUE(containers[1].auto_start);
    EXPECT_TRUE(containers[1].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[2].component_name, "ComponentC", 10));
    EXPECT_EQ(0, strncmp(containers[2].shared_library_path, "Container1.dll", 14));
    EXPECT_FALSE(containers[2].auto_start);
    EXPECT_FALSE(containers[2].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[3].component_name, "ComponentD", 10));
    EXPECT_EQ(0, strncmp(containers[3].shared_library_path, "Container2.dll", 14));
    EXPECT_FALSE(containers[3].auto_start);
    EXPECT_FALSE(containers[3].start_on_network_connection);

    //Read another CSV file and populate the containers array
    fh = xme_hal_fileio_fopen("xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_2.txt", XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_FALSE(XME_HAL_FILEIO_INVALID_FILE_HANDLE == fh);
    //char *token;
    token = (char*)xme_hal_mem_alloc(sizeof(char)*256);
    //char *buffer;
    buffer = (char*)xme_hal_mem_alloc(sizeof(char)*256);
    token_length = get_token(buffer, 256, token, &fh);
    //int i=0;
    i=0;
    while(token[0] != EOF)
    {
        switch(i)
        {
        case CONTAINER_INFO_COMPONENT_NAME: xme_hal_safeString_strncpy(containers[num_of_containers_in_array].component_name, token, token_length);
            break;
        case CONTAINER_INFO_SHARED_LIBRARY_PATH: xme_hal_safeString_strncpy(containers[num_of_containers_in_array].shared_library_path, token, token_length);
            break;
        case CONTAINER_INFO_AUTO_START: containers[num_of_containers_in_array].auto_start = (atoi(token) != 0);
            break;
        case CONTAINER_INFO_START_ON_NETWORK_CONNECTION: containers[num_of_containers_in_array].start_on_network_connection = (atoi(token) != 0);
            break;
        default: break;
        }
        i++;
        if(i == CONTAINER_INFO_MEMBERS_END)
        {
            i=0;
            num_of_containers_in_array++;
        }
        token_length = get_token(buffer, 256, token, &fh);
    }
    xme_hal_mem_free(buffer);
    xme_hal_mem_free(token);
    status = xme_hal_fileio_fclose(fh);
    EXPECT_TRUE(XME_STATUS_SUCCESS == status);
    fh = XME_HAL_FILEIO_INVALID_FILE_HANDLE;

    EXPECT_EQ(0, strncmp(containers[4].component_name, "ComponentA", 10));
    EXPECT_EQ(0, strncmp(containers[4].shared_library_path, "0", 1));
    EXPECT_TRUE(containers[4].auto_start);
    EXPECT_TRUE(containers[4].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[5].component_name, "ComponentB", 10));
    EXPECT_EQ(0, strncmp(containers[5].shared_library_path, "Container1.so", 14));
    EXPECT_TRUE(containers[5].auto_start);
    EXPECT_TRUE(containers[5].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[6].component_name, "ComponentC", 10));
    EXPECT_EQ(0, strncmp(containers[6].shared_library_path, "Container1.so", 14));
    EXPECT_FALSE(containers[6].auto_start);
    EXPECT_FALSE(containers[6].start_on_network_connection);

    EXPECT_EQ(0, strncmp(containers[7].component_name, "ComponentX", 10));
    EXPECT_EQ(0, strncmp(containers[7].shared_library_path, "Container2.so", 14));
    EXPECT_FALSE(containers[7].auto_start);
    EXPECT_FALSE(containers[7].start_on_network_connection);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
