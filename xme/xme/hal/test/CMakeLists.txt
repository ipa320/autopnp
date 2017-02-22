#
# Copyright (c) 2011-2013, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7660 2014-03-03 18:24:35Z geisinger $
#
# File:
#         CMake configuration file for XME HAL unit tests.
#

#------------------------------------------------------------------------------#
#-     xme_defines                                                            -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_defines"
    TYPE smoke
    smokeTestDefines.cpp
    xme_core_log
)

xme_unit_test(
    "xme_defines"
    TYPE interface
    interfaceTestDefines.cpp
    definesC.c
    xme_core_log
)

#------------------------------------------------------------------------------#
#-     xme_hal_base64io                                                       -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_base64io"
    TYPE integration
    base64ioTest.cpp
    xme_hal_mem
    xme_hal_safeString
)

#------------------------------------------------------------------------------#
#-     xme_hal_circularBuffer                                                 -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_circularBuffer" 
    TYPE smoke 
    smokeTestCircularBuffer.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_cmdLine                                                        -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_cmdLine"
    TYPE smoke
    smokeTestCmdLine.cpp
    xme_defines
)

xme_unit_test(
    "xme_hal_cmdLine"
    TYPE interface
    interfaceTestCmdLine.cpp
    xme_defines
)

#------------------------------------------------------------------------------#
#-     xme_hal_context                                                        -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_context"
    TYPE smoke
    smokeTestContext.cpp
    MOCK "xme_hal_tls" contextTlsMock.c
)

xme_unit_test(
    "xme_hal_context"
    TYPE interface
    interfaceTestContext.cpp
    MOCK "xme_hal_tls" contextTlsMock.c
)

#------------------------------------------------------------------------------#
#-     xme_hal_env                                                            -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_env"
    TYPE smoke
    smokeTestEnv.cpp
    xme_defines
)

#------------------------------------------------------------------------------#
#-     xme_hal_fileio                                                         -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_fileio"
    TYPE interface
    interfaceTestFileio.cpp
)

xme_unit_test(
    "xme_hal_fileio"
    TYPE integration
    fileioTest.cpp
    xme_hal_mem
)

configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/xme_fileio_manifest_csv_test_file_1.txt"
    "${CMAKE_BINARY_DIR}/xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_1.txt"
    COPYONLY
)
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/xme_fileio_manifest_csv_test_file_2.txt"
    "${CMAKE_BINARY_DIR}/xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_2.txt"
    COPYONLY
)
configure_file(
    "${CMAKE_CURRENT_LIST_DIR}/xme_fileio_manifest_csv_test_file_3_malformed.txt"
    "${CMAKE_BINARY_DIR}/xme_hal_fileio_test/xme_fileio_manifest_csv_test_file_3_malformed.txt"
    COPYONLY
)

#------------------------------------------------------------------------------#
#-     xme_hal_graph                                                          -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_graph"
    TYPE smoke
    smokeTestGraph.cpp
)

xme_unit_test(
    "xme_hal_graph"
    TYPE interface
    interfaceTestGraph.cpp
)

xme_unit_test(
    "xme_hal_graph"
    TYPE integration
    integrationTestGraph.cpp
    TIMEOUT 120
)

#------------------------------------------------------------------------------#
#-     xme_hal_linkedList                                                     -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_linkedList"
    TYPE smoke
    smokeTestLinkedList.cpp
)

xme_unit_test(
    "xme_hal_linkedList"
    TYPE interface
    interfaceTestLinkedList.cpp
)

xme_unit_test(
    "xme_hal_linkedList"
    TYPE integration
    integrationTestLinkedList.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_math                                                           -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_math"
    TYPE interface
    interfaceTestMath.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_mem                                                            -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_mem"
    TYPE smoke
    smokeTestMem.cpp
)

xme_unit_test(
    "xme_hal_mem"
    TYPE interface
    interfaceTestMem.cpp
)

xme_unit_test(
    "xme_hal_mem"
    TYPE integration
    integrationTestMem.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_mmap                                                           -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_mmap"
    TYPE smoke
    smokeTestMmap.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_net                                                            -#
#------------------------------------------------------------------------------#

# Temporarily disabled until migration is complete!
#
#xme_unit_test(
#    "xme_hal_net"
#    TYPE integration
#    ../../../tests/tests.cpp
#    netTest.cpp
#)

#------------------------------------------------------------------------------#
#-     xme_hal_random                                                         -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_random"
    TYPE smoke
    smokeTestRandom.cpp
)

xme_unit_test(
    "xme_hal_random"
    TYPE interface
    interfaceTestRandom.cpp
)

xme_unit_test(
    "xme_hal_random"
    TYPE integration
    integrationTestRandom.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_safeString                                                     -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_safeString"
    TYPE interface
    interfaceTestSafeString.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_sched                                                          -#
#------------------------------------------------------------------------------#

# Temporarily disabled until migration is complete!
#
#xme_unit_test(
#    "xme_hal_sched"
#    TYPE integration
#    ../../../tests/tests.cpp
#    schedTest.cpp
#)

#------------------------------------------------------------------------------#
#-     xme_hal_sharedPtr                                                      -#
#------------------------------------------------------------------------------#

# Temporarily disabled until migration is complete!
#
#xme_unit_test(
#    "xme_hal_sharedPtr"
#    TYPE integration
#    ../../../tests/tests.cpp
#    sharedPtrTest.cpp
#)

#------------------------------------------------------------------------------#
#-     xme_hal_sync                                                           -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_sync"
    TYPE smoke
    smokeTestSync.cpp
)

xme_unit_test(
    "xme_hal_sync"
    TYPE interface
    interfaceTestSync.cpp
)

xme_unit_test(
    "xme_hal_sync"
    TYPE integration
    integrationTestSync.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_table                                                          -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_table"
    TYPE interface
    interfaceTestTable.cpp
)

xme_unit_test(
    "xme_hal_table"
    TYPE integration
    integrationTestTable.cpp
)

#------------------------------------------------------------------------------#
#-     xme_hal_time                                                           -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_time"
    TYPE smoke
    smokeTestTime.cpp
)

xme_unit_test(
    "xme_hal_time"
    TYPE interface
    interfaceTestTime.cpp
    xme_hal_sleep
)

#------------------------------------------------------------------------------#
#-     xme_hal_tls                                                            -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_tls"
    TYPE smoke
    smokeTestTls.cpp
    MOCK "xme_hal_sync" tlsSyncMock.c
)

xme_unit_test(
    "xme_hal_tls"
    TYPE interface
    interfaceTestTls.cpp
    MOCK "xme_hal_sync" tlsSyncMock.c
)

#------------------------------------------------------------------------------#
#-     xme_hal_byteOrder                                                      -#
#------------------------------------------------------------------------------#

xme_unit_test(
    "xme_hal_byteOrder"
    TYPE interface
    interfaceTestByteOrder.cpp
)
