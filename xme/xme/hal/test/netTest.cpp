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
 * $Id: netTest.cpp 7664 2014-03-04 08:47:41Z geisinger $
 */

/**
 * \file
 *         Network communication testsuite.
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include <gtest/gtest.h>

//#include "tests/hal/test_net.h"

#include "xme/com/interface.h"

#include "xme/hal/include/net.h"

#if defined(_WIN32) && !defined(CYGWIN)
#	include <windows.h>
#else // #if defined(_WIN32) && !defined(CYGWIN)
#	include <limits.h>
#	include <unistd.h>
#	include <stdlib.h>
#	include <string.h>
#	include <sys/types.h>
#	include <sys/wait.h>
#	include <errno.h>
#endif // #if defined(_WIN32) && !defined(CYGWIN)

/******************************************************************************/
/***   Defines                                                              ***/
/******************************************************************************/
#define RECEIVE_BUFFER_SIZE ((uint16_t)256)

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
// TODO: See ticket #536
xme_com_interfaceDescr_t intf;

// TODO: See ticket #536
xme_com_interface_address_t intfAddress =
{
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
#if defined(_WIN32) && !defined(CYGWIN)
HANDLE
    launchServerProcess(const char* protocol, uint16_t port, const char* expectedRequest, const char* responseToSend)
{
    const char* serverProcessName = "tests_slave_net.exe";
    STARTUPINFO si;
    PROCESS_INFORMATION pi;
    char buffer[MAX_PATH+64];

    memset(&si, 0, sizeof(si));
    si.cb = sizeof(si);
    si.wShowWindow = SW_SHOW;
    si.dwFlags = STARTF_USESHOWWINDOW;

    memset(&pi, 0, sizeof(pi));

#if defined(_WIN32) && !defined(CYGWIN)
    _snprintf_s
        (
        (char*)&buffer, sizeof(buffer), _TRUNCATE,
#else // #if defined(_WIN32) && !defined(CYGWIN) //FIXME: THIS is impossible here because of outer guard!
    snprintf
        (
        &buffer, sizeof(buffer),
#endif // #if defined(_WIN32) && !defined(CYGWIN)
        "\"%s\" %s %u 5 \"%s\" 1 \"%s\"",
        serverProcessName,
        protocol,
        port,
        expectedRequest,
        responseToSend
        );

    if (0 == CreateProcess(0, buffer, 0, 0, true, 0, 0, 0, &si, &pi))
    {
        return 0;
    }
    else
    {
        // Make sure the server is initialized
        WaitForInputIdle(pi.hProcess, 2000);
        Sleep(500);

        return pi.hProcess;
    }
}
#else // #if defined(_WIN32) && !defined(CYGWIN)
pid_t
    launchServerProcess(const char* protocol, uint16_t port, const char* expectedRequest, const char* responseToSend)
{
    char buffer[PATH_MAX+64];	//excessive memory assignment
    //char buffer[100];
    pid_t processId;
    char* args[] = {(char*)"tests_slave_net", (char*)protocol, NULL, (char*)"5", (char*)expectedRequest, (char*)"1", (char*)responseToSend, NULL};

    memset(&buffer, 0, sizeof(buffer));

    snprintf((char*)&buffer, sizeof(buffer), (const char*)"%u",port);
    args[2]=buffer;

    //printf("%s %s %s %s %s %s %s\n",args[0],args[1],args[2],args[3],args[4],args[5],args[6]);
    processId=fork();

    if((pid_t)-1 == processId)
    {
        //RecordProperty("Fork error. Exiting.");  /* something went wrong */
        exit(1);
        return 0;
    }

    else if( processId==0 ) // Child
    {
        //RecordProperty("I am the child!"); /* A zero indicates the child process */
        execv ("tests_slave_net", args);
        sleep(5);
		return 0;
    }
    else	// Parent
    {
        //RecordProperty("I am the parent!"); /* A non-zero indicates the parent process */
        //wait(&processId);
        //RecordProperty("PROCESS_ID: ",processId);
        return processId;

    }
}
#endif // #if defined(_WIN32) && !defined(CYGWIN)

void
xme_hal_net_test_defines(void)
{
    // We currently assume little or big endian host systems.
    // If this ever changes, we probably do not only have to to
    // adapt this test. Hence, we better fail here in this case.
    EXPECT_TRUE
    (
        XME_HAL_NET_BYTEORDER_LITTLE_ENDIAN == xme_hal_net_getHostByteOrder() ||
        XME_HAL_NET_BYTEORDER_BIG_ENDIAN == xme_hal_net_getHostByteOrder()
    );

    if (XME_HAL_NET_BYTEORDER_LITTLE_ENDIAN == xme_hal_net_getHostByteOrder())
    {
        printf("Host system uses little endianness.\n");

        EXPECT_EQ(0x3412,                xme_hal_net_htons(0x1234));
        EXPECT_EQ(0x78563412UL,          xme_hal_net_htonl(0x12345678UL));
        EXPECT_EQ(0xEFCDAB9078563412ULL, xme_hal_net_htonll(0x1234567890ABCDEFULL));
    }
    else if (XME_HAL_NET_BYTEORDER_BIG_ENDIAN == xme_hal_net_getHostByteOrder())
    {
        printf("Host system uses big endianness.\n");

        EXPECT_EQ(0x1234,                xme_hal_net_htons(0x1234));
        EXPECT_EQ(0x12345678UL,          xme_hal_net_htonl(0x12345678));
        EXPECT_EQ(0x1234567890ABCDEFULL, xme_hal_net_htonll(0x1234567890ABCDEFULL));
    }
    else
    {
        FAIL();
    }
}

void
xme_hal_net_test_init()
{
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_initInterface(&intfAddress));
}

void
xme_hal_net_test_generic()
{
    // Performs generic tests against the xme_hal_net_*() API.

    // TODO: Test select() API! See ticket #726

    xme_hal_net_socketHandle_t s1, s2, s3;

    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_net_destroySocket(XME_HAL_NET_INVALID_SOCKET_HANDLE));
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_net_destroySocket((xme_hal_net_socketHandle_t) 1234));

    EXPECT_NE(XME_HAL_NET_INVALID_SOCKET_HANDLE, (s1 = xme_hal_net_createSocket(&intf, XME_HAL_NET_SOCKET_TCP, "localhost", 1234)));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s1));
    EXPECT_EQ(XME_STATUS_INVALID_HANDLE, xme_hal_net_destroySocket(s1));

    EXPECT_NE(XME_HAL_NET_INVALID_SOCKET_HANDLE, (s2 = xme_hal_net_createSocket(&intf, XME_HAL_NET_SOCKET_TCP, "localhost", 1234)));
    EXPECT_NE(XME_HAL_NET_INVALID_SOCKET_HANDLE, (s3 = xme_hal_net_createSocket(&intf, XME_HAL_NET_SOCKET_UDP, "localhost", 2345)));
    EXPECT_TRUE(s1 == s2); // Handles should be reused
    EXPECT_TRUE(s2 != s3); // Handles must not collide

    //RecordProperty("Testing TCP timeout condition. This might take a while...");
    EXPECT_EQ(XME_STATUS_CONNECTION_REFUSED, xme_hal_net_openSocket(s2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s2));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s2));

    //RecordProperty("Testing UDP timeout condition. This might take a while...");
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_openSocket(s3)); // UDP connections should connect immediatly
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s3));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s3));
}

void
xme_hal_net_test_local(uint16_t socketType)
{
    // Sends data over a non-blocking local connection.
    // This testcase creates two non-blocking local sockets.
    // A request is sent through the socket, followed by a response.

    char receiveBuffer[RECEIVE_BUFFER_SIZE];
    const char* sendBuffer = "Hello world!";
    const char* expectedResponse = "It works!";
    xme_hal_net_socketHandle_t s, c;

    // Create a non-blocking local socket that will act as a server
    //RecordProperty("Creating local server socket...");
    s = xme_hal_net_createSocket(&intf, socketType | XME_HAL_NET_SOCKET_NONBLOCKING, 0, 1230);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_openSocket(s));

    // Create a non-blocking local socket that will act as a client
    //RecordProperty("Creating local client socket...");
    c = xme_hal_net_createSocket(&intf, socketType | XME_HAL_NET_SOCKET_NONBLOCKING, "localhost", 1230);
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_openSocket(c));

    // Client sends some data
    //RecordProperty("Sending client request...");
    EXPECT_EQ(strlen(sendBuffer), xme_hal_net_writeSocket(c, sendBuffer, (uint16_t)strlen(sendBuffer)));

    //RecordProperty("Waiting...");
#if defined(_WIN32) && !defined(CYGWIN)
    Sleep(500);
#else // #if defined(_WIN32) && !defined(CYGWIN)
    sleep(1);
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    // Server reads the data
    //RecordProperty("Reading client request...");
    memset(receiveBuffer, 0, sizeof(receiveBuffer));
    EXPECT_EQ(strlen(sendBuffer), xme_hal_net_readSocket(s, receiveBuffer, RECEIVE_BUFFER_SIZE));
    //XME_TEST(0 == strcmp(receiveBuffer, sendBuffer));
    EXPECT_STREQ(receiveBuffer, sendBuffer);

    if (strlen(receiveBuffer) > 0)
    {
        // Send a reply to the client
        //RecordProperty("Sending server response...");
        EXPECT_TRUE(strlen(expectedResponse) == xme_hal_net_writeSocket(s, expectedResponse, (uint16_t)strlen(expectedResponse)));

        // Close the server socket
        //RecordProperty("Closing server socket...");
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s));

        //RecordProperty("Waiting...");
#if defined(_WIN32) && !defined(CYGWIN)
        Sleep(500);
#else // #if defined(_WIN32) && !defined(CYGWIN)
        sleep(1);
#endif // #if defined(_WIN32) && !defined(CYGWIN)

        // Read the response
        //RecordProperty("Receiving server response...");
        memset(receiveBuffer, 0, sizeof(receiveBuffer));
        EXPECT_TRUE(xme_hal_net_readSocket(c, receiveBuffer, sizeof(receiveBuffer)) == strlen(expectedResponse));
        //XME_TEST(0 == strcmp(receiveBuffer, expectedResponse));
        EXPECT_STREQ(expectedResponse, receiveBuffer);
    }
    else
    {
        //RecordProperty("Did not receive client request! Skipping sending of reply.");

        // Close the server socket
        //RecordProperty("Closing server socket...");
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s));
    }

    // Close the client socket
    //RecordProperty("Closing client socket...");
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(c));
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(c));
}

void
xme_hal_net_test_localhost(uint16_t socketType)
{
    // Send data between two processes.
    // This testcase creates a server processes that responds to requests
    // and responds with a response once a request has been received.

    char receiveBuffer[RECEIVE_BUFFER_SIZE];
    uint16_t port = 1234;
    const char* sendBuffer = "Hello server!";
    const char* expectedResponse = "Server response!";
    uint16_t responseTimeout = 5;
    xme_hal_net_socketHandle_t s;
    xme_core_status_t ready;

#if defined(_WIN32) && !defined(CYGWIN)
    HANDLE process;
    DWORD slaveExitCode;
    //process = launchServerProcess((XME_HAL_NET_SOCKET_TCP & socketType) ? "tcp" : "udp", port, sendBuffer, expectedResponse);
    //EXPECT_HRESULT_SUCCEEDED(process);
#else // #if defined(_WIN32) && !defined(CYGWIN)
    pid_t process;
    int slaveExitCode;
#endif // #if defined(_WIN32) && !defined(CYGWIN)
    //RecordProperty("[server] Configured to send request on %s port %u", (XME_HAL_NET_SOCKET_TCP & socketType) ? "TCP" : "UDP", port);

    //RecordProperty("[client] Launching server process...");

    EXPECT_TRUE(0 != (process = launchServerProcess((XME_HAL_NET_SOCKET_TCP & socketType) ? "tcp" : "udp", port, sendBuffer, expectedResponse)));

    if (0 != process)
    {
        //RecordProperty("[client] Sending request '%s' (%d byte(s))...", sendBuffer, (int)strlen(sendBuffer));

        // Open a socket and send some data
        s = xme_hal_net_createSocket(&intf, socketType | XME_HAL_NET_SOCKET_NONBLOCKING, "localhost", port);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_openSocket(s));
        EXPECT_TRUE(strlen(sendBuffer) == xme_hal_net_writeSocket(s, sendBuffer, (uint16_t)strlen(sendBuffer)));

        // Await response
        ready = xme_hal_net_selectSocket(s, true, false, 3000);
        EXPECT_EQ(XME_STATUS_SUCCESS, ready);

        if (XME_STATUS_SUCCESS == ready)
        {
            uint16_t time;
            uint16_t receiveBufferPos;
            uint16_t expectedResponseLength = (uint16_t)strlen(expectedResponse);

            // Keep waiting for packets until the timeout elapses or until
            // we have received at least the expected number of bytes
            memset(receiveBuffer, 0, sizeof(receiveBuffer));
            receiveBufferPos = 0;
            for (time = 0; time <= responseTimeout && receiveBufferPos < expectedResponseLength; time++)
            {
                uint16_t numBytes;

                //RecordProperty("\r[client] Waiting for incoming data... [%3d second(s), %5u byte(s)]", responseTimeout-time, receiveBufferPos);

                if (0 != (numBytes = xme_hal_net_readSocket(s, &receiveBuffer[receiveBufferPos], (uint16_t)(RECEIVE_BUFFER_SIZE-receiveBufferPos))))
                {
                    receiveBufferPos += numBytes;
                }
                else
                {
                    // Sleep for 1 second and try again later
#if defined(_WIN32) && !defined(CYGWIN)
                    Sleep(1000);
#else // #if defined(_WIN32) && !defined(CYGWIN)
                    sleep(2);
#endif // #if defined(_WIN32) && !defined(CYGWIN)

                }
            }

            //RecordProperty("\r[client] Waiting for incoming data... [%5u byte(s)]               \n", receiveBufferPos);

            EXPECT_LE(time, responseTimeout);

            //RecordProperty("[client] Received response '%s' (%d byte(s), expected '%s')\n", receiveBuffer, receiveBufferPos, expectedResponse);

            //XME_TEST(0 == strcmp(receiveBuffer, expectedResponse));
            EXPECT_STREQ(receiveBuffer, expectedResponse);
        }

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s));
#if defined(_WIN32) && !defined(CYGWIN)
        // Wait for the slave to terminate
        do
        {
            if (0 == GetExitCodeProcess(process, &slaveExitCode))
            {
                break;
            }
        }
        while (STILL_ACTIVE == slaveExitCode);
#else // #if defined(_WIN32) && !defined(CYGWIN)
        waitpid(0,&slaveExitCode,WNOHANG); //One can wait for children in ones own process group
        /*
        do
        {
        if ( kill ( process, 0 ) == -1 && errno == ESRCH )
        {
        break;
        }
        }
        while (1);
        */
#endif // #if defined(_WIN32) && !defined(CYGWIN)

        EXPECT_EQ(0, slaveExitCode);
    }
}

void
xme_hal_net_test_udp_broadcast()
{
    // Broadcast data over a local UDP connection
    // This testcase creates a server processes that responds to UDP
    // broadcast requests and responds with a response once a request
    // has been received.

    uint16_t socketType = XME_HAL_NET_SOCKET_UDP; // Broadcast of course only works with UDP
    char receiveBuffer[RECEIVE_BUFFER_SIZE];
    uint16_t port = 1235;
    const char* sendBuffer = "Hello world!";
    const char* expectedResponse = "Server broadcast response!";
    uint16_t responseTimeout = 5;
    xme_hal_net_socketHandle_t s;
    xme_core_status_t ready;

#if defined(_WIN32) && !defined(CYGWIN)
    HANDLE process;
    DWORD slaveExitCode;
#else // #if defined(_WIN32) && !defined(CYGWIN)
    pid_t process;
    int slaveExitCode;
#endif // #if defined(_WIN32) && !defined(CYGWIN)

    //RecordProperty("[server] Configured to send broadcast request on %s port %u\n", (XME_HAL_NET_SOCKET_TCP & socketType) ? "TCP" : "UDP", port);

    //RecordProperty("[client] Launching server process...\n");

    EXPECT_TRUE(0 != (process = launchServerProcess((XME_HAL_NET_SOCKET_TCP & socketType) ? "tcp" : "udp", port, sendBuffer, expectedResponse)));

    if (0 != process)
    {
        //RecordProperty("[client] Sending request '%s' (%d byte(s))...\n", sendBuffer, (int) strlen(sendBuffer));

        // Open a socket and broadcast some data
        s = xme_hal_net_createSocket(&intf, socketType | XME_HAL_NET_SOCKET_NONBLOCKING | XME_HAL_NET_SOCKET_BROADCAST, 0, port);
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_openSocket(s));
        EXPECT_TRUE(strlen(sendBuffer) == xme_hal_net_writeSocket(s, sendBuffer, (uint16_t)strlen(sendBuffer)));

        // Await response
        ready = xme_hal_net_selectSocket(s, true, false, 3000);
        EXPECT_EQ(XME_STATUS_SUCCESS, ready);

        if (XME_STATUS_SUCCESS == ready)
        {
            uint16_t time;
            uint16_t receiveBufferPos;
            uint16_t expectedResponseLength = (uint16_t)strlen(expectedResponse);

            // Keep waiting for packets until the timeout elapses or until
            // we have received at least the expected number of bytes
            memset(receiveBuffer, 0, sizeof(receiveBuffer));
            receiveBufferPos = 0;
            for (time = 0; time <= responseTimeout && receiveBufferPos < expectedResponseLength; time++)
            {
                uint16_t numBytes;

                //RecordProperty("\r[client] Waiting for incoming data... [%3d second(s), %5u byte(s)]", responseTimeout-time, receiveBufferPos);

                if (0 != (numBytes = xme_hal_net_readSocket(s, &receiveBuffer[receiveBufferPos], (uint16_t)(RECEIVE_BUFFER_SIZE-receiveBufferPos))))
                {
                    receiveBufferPos += numBytes;
                }
                else
                {
                    // Sleep for 1 second and try again later
#if defined(_WIN32) && !defined(CYGWIN)
                    Sleep(1000);
#else // #if defined(_WIN32) && !defined(CYGWIN)
                    sleep(2);
#endif // #if defined(_WIN32) && !defined(CYGWIN)

                }
            }

            //RecordProperty("\r[client] Waiting for incoming data... [%5u byte(s)]               \n", receiveBufferPos);

            EXPECT_LE(time, responseTimeout);

            //RecordProperty("[client] Received response '%s' (%d byte(s), expected '%s')\n", receiveBuffer, receiveBufferPos, expectedResponse);

            EXPECT_STREQ(receiveBuffer, expectedResponse);
        }

        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_closeSocket(s));
        EXPECT_EQ(XME_STATUS_SUCCESS, xme_hal_net_destroySocket(s));

        // Wait for the slave to terminate
#if defined(_WIN32) && !defined(CYGWIN)
        do
        {
            if (0 == GetExitCodeProcess(process, &slaveExitCode))
            {
                break;
            }
        }
        while (STILL_ACTIVE == slaveExitCode);
#else // #if defined(_WIN32) && !defined(CYGWIN)
        waitpid(0,&slaveExitCode,WNOHANG); // One can wait for children in ones own process group:
        /*
        do
        {
        if ( kill ( process, 0 ) == -1 && errno == ESRCH )
        {
        break;
        }
        }
        while (1);
        */
#endif // #if defined(_WIN32) && !defined(CYGWIN)

        EXPECT_EQ(0, slaveExitCode);
    }
}

void
xme_hal_net_test_fini()
{
    xme_hal_net_finiInterface(&intf);
}

TEST(XMEHalNetTest, xme_tests_hal_net)
{
    // Setup
    EXPECT_EQ(XME_STATUS_SUCCESS, xme_core_init());

    xme_hal_net_test_defines();

    //xme_hal_net_test_init();
    //xme_hal_net_test_generic();
    //xme_hal_net_test_local(XME_HAL_NET_SOCKET_UDP);
    ////xme_hal_net_test_local(XME_HAL_NET_SOCKET_TCP); // Disabled for the moment since it does not work as expected (TCP needs blocking accept())
    //xme_hal_net_test_localhost(XME_HAL_NET_SOCKET_UDP);
    //xme_hal_net_test_localhost(XME_HAL_NET_SOCKET_TCP);
    //xme_hal_net_test_udp_broadcast();
    //xme_hal_net_test_fini();

    // Teardown
    xme_core_fini();
}
