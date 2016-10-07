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
 * $Id: interfaceTestSafeString.cpp 6275 2014-01-09 14:02:52Z geisinger $
 */

/**
 * \file
 *         Safe string interface tests.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

#include "xme/hal/include/safeString.h"

#include <inttypes.h>

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define BUFFER_SIZE 64

/******************************************************************************/
/***   Classes                                                              ***/
/******************************************************************************/

class SafeStringInterfaceTest: public ::testing::Test
{
protected:
    SafeStringInterfaceTest()
    : vi(-42)
    , vu(123)
    , vc(-15)
    , vf(0.123f)
    {
        // Ensure the buffer is initialized with non-zero, but constant values
        for (int i=0; i<(BUFFER_SIZE); i++)
        {
            buf[i] = 'A'+i;
        }

        format = "int %d, int %i, uint %u, char %c, string %s, float %.3f";

        vs = "tEs!\n";

        s8[0]  = -128;
        s8[1]  =    0;
        s8[2]  =  127;

        s16[0] = -32768;
        s16[1] =      0;
        s16[2] =  32767;

        s32[0] = -2147483648LL;
        s32[1] =           0LL;
        s32[2] =  2147483647LL;

        // Notice the following value is not -9223372036854775808LL!
        // This is because of GCC warnings:
        //  - integer constant is so large that it is unsigned
        //  - this decimal constant is unsigned only in ISO C90
        s64[0] = -9223372036854775807LL;
        s64[1] =                    0LL;
        s64[2] =  9223372036854775807LL;

        u8[0]  =   0;
        u8[1]  = 128;
        u8[2]  = 255;

        u16[0]  =     0;
        u16[1]  = 32768;
        u16[2]  = 65535;

        u32[0]  =          0ULL;
        u32[1]  = 2147483648ULL;
        u32[2]  = 4294967295ULL;

        u64[0]  =                   0ULL;
        u64[1]  =  922337203685477580ULL;
        u64[2]  = 1844674407370955159ULL;
    }

    virtual ~SafeStringInterfaceTest()
    {
    }

    char buf[BUFFER_SIZE];

    const char* format;

    const int vi;
    const unsigned int vu;
    const char vc;
    const float vf;
    const char* vs;

    int8_t s8[3];
    int16_t s16[3];
    int32_t s32[3];
    int64_t s64[3];

    uint8_t u8[3];
    uint16_t u16[3];
    uint32_t u32[3];
    uint64_t u64[3];
};

/******************************************************************************/
/***   Tests                                                                ***/
/******************************************************************************/

//----------------------------------------------------------------------------//
//     SafeStringInterfaceTest                                                //
//----------------------------------------------------------------------------//

// scprintf tests

TEST_F(SafeStringInterfaceTest, countFormattedCharacters)
{
    int length = xme_hal_safeString_scprintf(format, vi, vi, vu, vc, vs, vf);
    EXPECT_EQ(61, length);
}

// snprintf tests

TEST_F(SafeStringInterfaceTest, snprintfWithBigEnoughBuffer)
{
    int length = xme_hal_safeString_snprintf(buf, 62, format, vi, vi, vu, vc, vs, vf);
    EXPECT_EQ(61, length);
    EXPECT_STREQ(buf, "int -42, int -42, uint 123, char ñ, string tEs!\n, float 0.123");
}

TEST_F(SafeStringInterfaceTest, snprintfWithTooSmallBuffer61)
{
    // Two allowed cases:
    // 1) Status is negative
    // 2) Status is positive and it must be the length that the complete (not truncated) formatted string
    //    would have had if the buffer had been big enough (excluding the terminating null character)

    char templ[] = "int -42, int -42, uint 123, char ñ, string tEs!\n, float 0.123";

    for (int i = 61; i > 0; i--)
    {
        int length = xme_hal_safeString_snprintf(buf, i, format, vi, vi, vu, vc, vs, vf);
        EXPECT_NE(0, length);

        templ[i-1] = 0;
        EXPECT_STREQ(buf, templ);

        if (length >= 0)
        {
            EXPECT_EQ(61, length);
        }
    }
}

TEST_F(SafeStringInterfaceTest, snprintfWithNullBuffer)
{
    int length = xme_hal_safeString_snprintf(NULL, 0, format, vi, vi, vu, vc, vs, vf);
    if (length >= 0)
    {
        EXPECT_EQ(length, 61);
    }
}

// Format signed 8 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRId8)
{
    const char* str[3] = { "-128 127", "0 -1", "127 -128" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRId8 " %" PRId8, s8[i], ~s8[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIi8)
{
    const char* str[3] = { "-128 127", "0 -1", "127 -128" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIi8 " %" PRIi8, s8[i], ~s8[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format signed 16 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRId16)
{
    const char* str[3] = { "-32768 32767", "0 -1", "32767 -32768" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRId16 " %" PRId16, s16[i], ~s16[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIi16)
{
    const char* str[3] = { "-32768 32767", "0 -1", "32767 -32768" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIi16 " %" PRIi16, s16[i], ~s16[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format signed 32 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRId32)
{
    const char* str[3] = { "-2147483648 2147483647", "0 -1", "2147483647 -2147483648" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRId32 " %" PRId32, s32[i], ~s32[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIi32)
{
    const char* str[3] = { "-2147483648 2147483647", "0 -1", "2147483647 -2147483648" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIi32 " %" PRIi32, s32[i], ~s32[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format signed 64 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRId64)
{
    const char* str[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRId64 " %" PRId64, s64[i], ~s64[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIi64)
{
    const char* str[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIi64 " %" PRIi64, s64[i], ~s64[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format signed 64 bit integer as MAX decimal (must support at least 64 bit to pass)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIdMAX)
{
    const char* str[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIdMAX " %" PRIdMAX, s64[i], ~s64[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIiMAX)
{
    const char* str[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIiMAX " %" PRIiMAX, s64[i], ~s64[i]);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format signed 64 bit integer as PTR decimal (depends on host pointer size)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIdPTR)
{
    const char* str32bit[3] = { "1 -2147483648", "0 0", "-1 2147483647" };
    const char* str64bit[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIdPTR " %" PRIdPTR, s64[i], ~s64[i]);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIiPTR)
{
    const char* str32bit[3] = { "1 -2147483648", "0 0", "-1 2147483647" };
    const char* str64bit[3] = { "-9223372036854775807 9223372036854775806", "0 -1", "9223372036854775807 -9223372036854775808" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIdPTR " %" PRIdPTR, s64[i], ~s64[i]);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

// Format unsigned 8 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIu8)
{
    const char* str[3] = { "0 0", "128 42", "255 85" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIu8 " %" PRIu8, u8[i], u8[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 8 bit integer as octal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIo8)
{
    const char* str[3] = { "00 00", "0200 052", "0377 0125" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIo8 " 0%" PRIo8, u8[i], u8[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 8 bit integer as hex

TEST_F(SafeStringInterfaceTest, sprintfWithPRIx8)
{
    const char* str[3] = { "0x0 0x0", "0x80 0x2a", "0xff 0x55" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIx8 " 0x%" PRIx8, u8[i], u8[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIX8)
{
    const char* str[3] = { "0x0 0x0", "0x80 0x2A", "0xFF 0x55" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIX8 " 0x%" PRIX8, u8[i], u8[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 16 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIu16)
{
    const char* str[3] = { "0 0", "32768 10922", "65535 21845" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIu16 " %" PRIu16, u16[i], u16[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 16 bit integer as octal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIo16)
{
    const char* str[3] = { "00 00", "0100000 025252", "0177777 052525" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIo16 " 0%" PRIo16, u16[i], u16[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 16 bit integer as hex

TEST_F(SafeStringInterfaceTest, sprintfWithPRIx16)
{
    const char* str[3] = { "0x0 0x0", "0x8000 0x2aaa", "0xffff 0x5555" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIx16 " 0x%" PRIx16, u16[i], u16[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIX16)
{
    const char* str[3] = { "0x0 0x0", "0x8000 0x2AAA", "0xFFFF 0x5555" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIX16 " 0x%" PRIX16, u16[i], u16[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 32 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIu32)
{
    const char* str[3] = { "0 0", "2147483648 715827882", "4294967295 1431655765" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIu32 " %" PRIu32, u32[i], u32[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 32 bit integer as octal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIo32)
{
    const char* str[3] = { "00 00", "020000000000 05252525252", "037777777777 012525252525" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIo32 " 0%" PRIo32, u32[i], u32[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 32 bit integer as hex

TEST_F(SafeStringInterfaceTest, sprintfWithPRIx32)
{
    const char* str[3] = { "0x0 0x0", "0x80000000 0x2aaaaaaa", "0xffffffff 0x55555555" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIx32 " 0x%" PRIx32, u32[i], u32[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIX32)
{
    const char* str[3] = { "0x0 0x0", "0x80000000 0x2AAAAAAA", "0xFFFFFFFF 0x55555555" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIX32 " 0x%" PRIX32, u32[i], u32[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as decimal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIu64)
{
    const char* str[3] = { "0 0", "922337203685477580 307445734561825860", "1844674407370955159 614891469123651719" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIu64 " %" PRIu64, u64[i], u64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as octal

TEST_F(SafeStringInterfaceTest, sprintfWithPRIo64)
{
    const char* str[3] = { "00 00", "063146314631463146314 021042104210421042104", "0146314631463146314627 042104210421042104207" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIo64 " 0%" PRIo64, u64[i], u64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as hex

TEST_F(SafeStringInterfaceTest, sprintfWithPRIx64)
{
    const char* str[3] = { "0x0 0x0", "0xccccccccccccccc 0x444444444444444", "0x1999999999999997 0x888888888888887" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIx64 " 0x%" PRIx64, u64[i], u64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIX64)
{
    const char* str[3] = { "0x0 0x0", "0xccccccccccccccc 0x444444444444444", "0x1999999999999997 0x888888888888887" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIX64 " 0x%" PRIX64, u64[i], u64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as MAX decimal (must support at least 64 bit to pass)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIuMAX)
{
    const char* str[3] = { "9223372036854775809 15372286728091293014", "0 0", "9223372036854775807 3074457345618258602" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIuMAX " %" PRIuMAX, s64[i], s64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as MAX octal (must support at least 64 bit to pass)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIoMAX)
{
    const char* str[3] = { "01000000000000000000001 01525252525252525252526", "00 00", "0777777777777777777777 0252525252525252525252" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIoMAX " 0%" PRIoMAX, s64[i], s64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as MAX hex (must support at least 64 bit to pass)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIxMAX)
{
    const char* str[3] = { "0x8000000000000001 0xd555555555555556", "0x0 0x0", "0x7fffffffffffffff 0x2aaaaaaaaaaaaaaa" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIxMAX " 0x%" PRIxMAX, s64[i], s64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIXMAX)
{
    const char* str[3] = { "0x8000000000000001 0xD555555555555556", "0x0 0x0", "0x7FFFFFFFFFFFFFFF 0x2AAAAAAAAAAAAAAA" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIXMAX " 0x%" PRIXMAX, s64[i], s64[i]/3);
        EXPECT_STRCASEEQ(str[i], buf);
    }
}

// Format unsigned 64 bit integer as PTR decimal (depends on host pointer size)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIuPTR)
{
    const char* str32bit[3] = { "1 2147483648", "0 0", "4294967295 2147483647" };
    const char* str64bit[3] = { "9223372036854775809 15372286728091293014", "0 0", "9223372036854775807 3074457345618258602" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "%" PRIuPTR " %" PRIuPTR, s64[i], s64[i]/3);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

// Format unsigned 64 bit integer as PTR octal (depends on host pointer size)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIoPTR)
{
    const char* str32bit[3] = { "01 020000000000", "00 00", "037777777777 017777777777" };
    const char* str64bit[3] = { "01000000000000000000001 01525252525252525252526", "00 00", "0777777777777777777777 0252525252525252525252" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0%" PRIoPTR " 0%" PRIoPTR, s64[i], s64[i]/3);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

// Format unsigned 64 bit integer as PTR hex (depends on host pointer size)

TEST_F(SafeStringInterfaceTest, sprintfWithPRIxPTR)
{
    const char* str32bit[3] = { "0x1 0x80000000", "0x0 0x0", "0xffffffff 0x7fffffff" };
    const char* str64bit[3] = { "0x8000000000000001 0xd555555555555556", "0x0 0x0", "0x7fffffffffffffff 0x2aaaaaaaaaaaaaaa" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIxPTR " 0x%" PRIxPTR, s64[i], s64[i]/3);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

TEST_F(SafeStringInterfaceTest, sprintfWithPRIXPTR)
{
    const char* str32bit[3] = { "0x1 0x80000000", "0x0 0x0", "0xFFFFFFFF 0x7FFFFFFF" };
    const char* str64bit[3] = { "0x8000000000000001 0xD555555555555556", "0x0 0x0", "0x7FFFFFFFFFFFFFFF 0x2AAAAAAAAAAAAAAA" };
    for (int i = 0; i < 3; i++)
    {
        xme_hal_safeString_snprintf(buf, BUFFER_SIZE, "0x%" PRIXPTR " 0x%" PRIXPTR, s64[i], s64[i]/3);
        switch (sizeof(void*))
        {
            case 4:
                EXPECT_STRCASEEQ(str32bit[i], buf);
                break;
            case 8:
                EXPECT_STRCASEEQ(str64bit[i], buf);
                break;
            default:
                ASSERT_TRUE(sizeof(void*) == 4 || sizeof(void*) == 8);
                break;
        }
    }
}

// strnlen tests

TEST_F(SafeStringInterfaceTest, strnlenWithNullStringAndZeroLength)
{
    EXPECT_EQ(0U, xme_hal_safeString_strnlen(NULL, 0U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithEmptyStringAndZeroLength)
{
    EXPECT_EQ(0U, xme_hal_safeString_strnlen("", 0U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithEmptyStringAndNonZeroLength)
{
    EXPECT_EQ(0U, xme_hal_safeString_strnlen("", 10U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithShortStringAndZeroLength)
{
    EXPECT_EQ(0U, xme_hal_safeString_strnlen("fortiss", 0U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithShortStringAndShortLength)
{
    EXPECT_EQ(3U, xme_hal_safeString_strnlen("fortiss", 3U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithShortStringAndExactLength)
{
    EXPECT_EQ(5U, xme_hal_safeString_strnlen("fortiss", 5U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithShortStringAndLongLength)
{
    EXPECT_EQ(7U, xme_hal_safeString_strnlen("fortiss", 10U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithSpecialCharacters)
{
    EXPECT_EQ(10U, xme_hal_safeString_strnlen("n\nr\rt\tf\f  ", 255U));
}

TEST_F(SafeStringInterfaceTest, strnlenWithStringIncludingNulCharacter)
{
    EXPECT_EQ(5U, xme_hal_safeString_strnlen("first\0second", 255U));
}

// strncpy tests

TEST_F(SafeStringInterfaceTest, strncpyWithNullDestination)
{
    ASSERT_EQ(NULL, xme_hal_safeString_strncpy(NULL, "", 0U));
}

TEST_F(SafeStringInterfaceTest, strncpyWithZeroSize)
{
    buf[0] = 0;
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "abc", 0U));
    EXPECT_STREQ("", buf);
}

TEST_F(SafeStringInterfaceTest, strncpyWithEmptyString)
{
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "", 1U));
    EXPECT_STREQ("", buf);
}

TEST_F(SafeStringInterfaceTest, strncpyWithShortString)
{
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "a", 2U));
    EXPECT_STREQ("a", buf);
}

TEST_F(SafeStringInterfaceTest, strncpyWithPartialString)
{
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "fortiss", 4U));
    EXPECT_STREQ("for", buf);
}

TEST_F(SafeStringInterfaceTest, strncpyWithFullString)
{
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "fortiss", 8U));
    EXPECT_STREQ("fortiss", buf);
}

TEST_F(SafeStringInterfaceTest, strncpyWithFullStringAndLargeBuffer)
{
    ASSERT_EQ(buf, xme_hal_safeString_strncpy(buf, "fortiss", 16U));
    EXPECT_STREQ("fortiss", buf);
}

// strncat tests

TEST_F(SafeStringInterfaceTest, strncatWithNullDestination)
{
    xme_hal_safeString_strncat(NULL, "", 0U);
}

TEST_F(SafeStringInterfaceTest, strncatWithZeroSize)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "abc", 0U);
    EXPECT_STREQ("", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithEmptyStringAndEmptyString)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "", 1U);
    EXPECT_STREQ("", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithEmptyStringAndShortString)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "a", 2U);
    EXPECT_STREQ("a", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithEmptyStringAndPartialString)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 4U);
    EXPECT_STREQ("for", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithEmptyStringAndFullString)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 8U);
    EXPECT_STREQ("fortiss", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithEmptyStringAndLargeBuffer)
{
    buf[0] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 16U);
    EXPECT_STREQ("fortiss", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithShortStringAndEmptyString)
{
    buf[0] = 'a';
    buf[1] = 0;
    xme_hal_safeString_strncat(buf, "", 2U);
    EXPECT_STREQ("a", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithShortStringAndShortString)
{
    buf[0] = 'a';
    buf[1] = 0;
    xme_hal_safeString_strncat(buf, "b", 3U);
    EXPECT_STREQ("ab", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithShortStringAndPartialString)
{
    buf[0] = '*';
    buf[1] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 5U);
    EXPECT_STREQ("*for", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithShortStringAndFullString)
{
    buf[0] = '*';
    buf[1] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 9U);
    EXPECT_STREQ("*fortiss", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithShortStringAndLargeBuffer)
{
    buf[0] = '*';
    buf[1] = 0;
    xme_hal_safeString_strncat(buf, "fortiss", 16U);
    EXPECT_STREQ("*fortiss", buf);
}

TEST_F(SafeStringInterfaceTest, strncatWithFullStringAndFullString)
{
    buf[0] = 'f';
    buf[1] = 'o';
    buf[2] = 'r';
    buf[3] = 't';
    buf[4] = 'i';
    buf[5] = 's';
    buf[6] = 's';
    buf[7] = 0;
    xme_hal_safeString_strncat(buf, "*fortiss", 16U);
    EXPECT_STREQ("fortiss*fortiss", buf);
}

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
