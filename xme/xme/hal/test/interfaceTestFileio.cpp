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
 * $Id: interfaceTestFileio.cpp 4597 2013-08-07 14:18:28Z ruiz $
 */

/**
 * \file
 *         File I/O interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/fileio.h"

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/

#define BUFFER_SIZE 64

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class FileioInterfaceTest: public ::testing::Test
{
protected:
    FileioInterfaceTest()
    : fileHandle(XME_HAL_FILEIO_INVALID_FILE_HANDLE)
    , position(0)
    {
        emptyFilename       = "";
        nonExistingFilename = "NonExistingFile.xyz";
        existingFilename    = "xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_1.txt";
    }

    virtual ~FileioInterfaceTest()
    {
        if (fileHandle != XME_HAL_FILEIO_INVALID_FILE_HANDLE)
        {
            xme_hal_fileio_fclose(fileHandle);
        }
    }

    const char* emptyFilename;
    const char* nonExistingFilename;
    const char* existingFilename;

    xme_hal_fileio_fileHandle_t fileHandle;
    uint64_t position;
    char buf[BUFFER_SIZE];
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     FileioInterfaceTest                                                    //
//----------------------------------------------------------------------------//

// fopen tests

TEST_F(FileioInterfaceTest, openInvalidFilename)
{
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_READONLY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_WRITEONLY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_APPEND));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_READ_UPDATE));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_WRITE_UPDATE));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_APPEND_UPDATE));

    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_WRITEONLY_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_APPEND_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_READ_UPDATE_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_WRITE_UPDATE_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(emptyFilename, XME_HAL_FILEIO_MODE_APPEND_UPDATE_BINARY));
}

TEST_F(FileioInterfaceTest, openNonExistingFilename)
{
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READONLY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READ_UPDATE));

    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY));
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READ_UPDATE_BINARY));
}

// fclose tests

TEST_F(FileioInterfaceTest, openAndCloseExistingFilenameReadOnly)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, openAndCloseExistingFilenameReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, openAndCloseExistingFilenameReadUpdate)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READ_UPDATE);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, openAndCloseExistingFilenameReadUpdateBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READ_UPDATE_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

// ftell tests

TEST_F(FileioInterfaceTest, tellFromInvalidHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_fileio_ftell(XME_HAL_FILEIO_INVALID_FILE_HANDLE, &position));
    EXPECT_EQ(0UL, position);
}

TEST_F(FileioInterfaceTest, tellFromValidFileHandleReadOnly)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(0UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, tellFromValidFileHandleReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(0UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}


TEST_F(FileioInterfaceTest, seekSetOnInvalidFileHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_fileio_fseek(XME_HAL_FILEIO_INVALID_FILE_HANDLE, 0L, XME_HAL_FILEIO_SEEKORIGIN_SET));
}

TEST_F(FileioInterfaceTest, seekCurOnInvalidFileHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_fileio_fseek(XME_HAL_FILEIO_INVALID_FILE_HANDLE, 0L, XME_HAL_FILEIO_SEEKORIGIN_CUR));
}

TEST_F(FileioInterfaceTest, seekEndOnInvalidFileHandle)
{
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_fileio_fseek(XME_HAL_FILEIO_INVALID_FILE_HANDLE, 0L, XME_HAL_FILEIO_SEEKORIGIN_END));
}

TEST_F(FileioInterfaceTest, seekSetOnValidFileHandleReadOnly)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 17L, XME_HAL_FILEIO_SEEKORIGIN_SET));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(17UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 28L, XME_HAL_FILEIO_SEEKORIGIN_SET));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(28UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, seekSetOnValidFileHandleReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 17L, XME_HAL_FILEIO_SEEKORIGIN_SET));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(17UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 28L, XME_HAL_FILEIO_SEEKORIGIN_SET));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(28UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, seekCurOnValidFileHandleReadOnly)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 17L, XME_HAL_FILEIO_SEEKORIGIN_CUR));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(17UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 28L-17L, XME_HAL_FILEIO_SEEKORIGIN_CUR));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(28UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, seekCurOnValidFileHandleReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 17L, XME_HAL_FILEIO_SEEKORIGIN_CUR));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(17UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 28L-17L, XME_HAL_FILEIO_SEEKORIGIN_CUR));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(28UL, position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, seekEndOnValidFileHandleReadOnly)
{
    uint64_t fileSize = 0;

    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 0L, XME_HAL_FILEIO_SEEKORIGIN_END));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &fileSize));
    EXPECT_GT(fileSize, 0UL);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, -17L, XME_HAL_FILEIO_SEEKORIGIN_END));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(static_cast<uint64_t>(fileSize-17L), position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, -28L, XME_HAL_FILEIO_SEEKORIGIN_END));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(static_cast<uint64_t>(fileSize-28L), position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

TEST_F(FileioInterfaceTest, seekEndOnValidFileHandleReadOnlyBinary)
{
    uint64_t fileSize = 0;

    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, 0L, XME_HAL_FILEIO_SEEKORIGIN_END));

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &fileSize));
    EXPECT_GT(fileSize, 0UL);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, -17L, XME_HAL_FILEIO_SEEKORIGIN_END));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(static_cast<uint64_t>(fileSize-17L), position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fseek(fileHandle, -28L, XME_HAL_FILEIO_SEEKORIGIN_END));

    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(static_cast<uint64_t>(fileSize-28L), position);

    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_fclose(fileHandle));
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
}

// fread tests

TEST_F(FileioInterfaceTest, readFromFileReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    ASSERT_EQ(sizeof(buf[0])*16, xme_hal_fileio_fread(buf, sizeof(buf[0]), 16, fileHandle));
    buf[16] = 0;
    EXPECT_STREQ("ComponentA,0,1,1", buf);

    // Notice: This only works for binary file streams!
    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(16UL, position);

    ASSERT_EQ(sizeof(buf[0])*16, xme_hal_fileio_fread(buf, sizeof(buf[0]), 16, fileHandle));
    buf[16] = 0;
    EXPECT_STREQ("\nComponentB,Cont", buf);

    // Notice: This only works for binary file streams!
    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(32UL, position);
}

// fwrite tests

// TODO: Implement fwrite tests!

// fgetc tests

TEST_F(FileioInterfaceTest, getcFromFileReadOnlyBinary)
{
    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    EXPECT_EQ('C', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('o', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('m', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('p', xme_hal_fileio_fgetc(fileHandle));

    // Notice: This only works for binary file streams!
    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(4UL, position);

    EXPECT_EQ('o', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('n', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('e', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('n', xme_hal_fileio_fgetc(fileHandle));
    EXPECT_EQ('t', xme_hal_fileio_fgetc(fileHandle));

    // Notice: This only works for binary file streams!
    position = 0x1234;
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_fileio_ftell(fileHandle, &position));
    EXPECT_EQ(9UL, position);
}

// fgets tests

TEST_F(FileioInterfaceTest, getsFromFileReadOnly)
{
    char str[64];
    char *ret;
    char *ret_NULL = NULL;
    int i;

    fileHandle = xme_hal_fileio_fopen(existingFilename, XME_HAL_FILEIO_MODE_READONLY_BINARY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    ret = xme_hal_fileio_fgets(str, 64, fileHandle);
    ASSERT_NE(ret_NULL, ret);

    i=0;
    EXPECT_EQ('C', str[i++]);
    EXPECT_EQ('o', str[i++]);
    EXPECT_EQ('m', str[i++]);
    EXPECT_EQ('p', str[i++]);
    EXPECT_EQ('o', str[i++]);
    EXPECT_EQ('n', str[i++]);
    EXPECT_EQ('e', str[i++]);
    EXPECT_EQ('n', str[i++]);
    EXPECT_EQ('t', str[i++]);
    EXPECT_EQ('A', str[i++]);
    EXPECT_EQ(',', str[i++]);
    EXPECT_EQ('0', str[i++]);
    EXPECT_EQ(',', str[i++]);
    EXPECT_EQ('1', str[i++]);
    EXPECT_EQ(',', str[i++]);
    EXPECT_EQ('1', str[i++]);
    EXPECT_EQ('\0', str[i]);

}

// remove tests

TEST_F(FileioInterfaceTest, createNonExistingFileAndRemove)
{
    int ret;
    xme_status_t retFromRemove;

    // ensuring that the file does not exist
    fileHandle = xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READONLY);
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);
    
    // trying to remove a file that does not exist
    retFromRemove = xme_hal_fileio_deleteFile(nonExistingFilename);
    EXPECT_EQ(XME_STATUS_NOT_FOUND, retFromRemove);
    
    // creating a new file
    fileHandle = xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_WRITEONLY);
    ASSERT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);
    
    // writing a test string to the file
    ret = xme_hal_fileio_fprintf(fileHandle, "TestFile");
    EXPECT_EQ(8, ret);

    // closing the file
    xme_hal_fileio_fclose(fileHandle);
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;

#ifdef WIN32
    // opening the file in readonly mode
    fileHandle = xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READONLY);
    EXPECT_NE(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);

    // try to remove the open file
    retFromRemove = xme_hal_fileio_deleteFile(nonExistingFilename);
    EXPECT_EQ(XME_STATUS_PERMISSION_DENIED, retFromRemove);
    
    xme_hal_fileio_fclose(fileHandle);
    fileHandle = XME_HAL_FILEIO_INVALID_FILE_HANDLE;
#endif

    retFromRemove = xme_hal_fileio_deleteFile(nonExistingFilename);
    EXPECT_EQ(XME_STATUS_SUCCESS, retFromRemove);

    fileHandle = xme_hal_fileio_fopen(nonExistingFilename, XME_HAL_FILEIO_MODE_READONLY);
    EXPECT_EQ(XME_HAL_FILEIO_INVALID_FILE_HANDLE, fileHandle);
}

// fputc tests

// TODO: Implement fputc tests!

// fputs tests

// TODO: Implement fputs tests!

// fprintf tests

// TODO: Implement fprintf tests!

// vfprintf tests

// TODO: Implement vfprintf tests!

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
