#
# Copyright (c) 2011-2012, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7459 2014-02-18 10:25:58Z geisinger $
#
# File:
#         CMake configuration file for XME Execution Manager.
#

# source files for execution manager
set(EM_SOURCES
    src/descriptorTable.c
    src/dispatcher.c
    src/scheduler.c
    src/configurator.c
    src/scheduleOperations.c
    src/scheduleSetOperations.c
    src/executionManager.c
    src/executionManagerUtils.c
    src/wrapperInterface.c
    src/taskTimer.c
    src/componentRepository.c
    src/cycleControl.c
)

# header files for execution manager
set(EM_HEADERS
    include/internDescriptorTable.h
    include/executionManager.h
    include/executionManagerComponentRepositoryInterface.h
    include/executionManagerConfigurator.h
    include/executionManagerDataStructures.h
    include/executionManagerIntern.h
    include/executionManagerScheduleManagementInterface.h
    include/executionManagerWrapperInterface.h
)

xme_add_component(
    "xme_core_executionManager"
    ${EM_SOURCES}
    ${EM_HEADERS}
    xme_hal_linkedList
    xme_hal_mem
    xme_hal_sleep
    xme_hal_sync
    xme_hal_sched
    xme_hal_table
    xme_hal_time
    xme_core_log
)

xme_add_component(
    "xme_core_executionManagerProxy"
    src/wrapperInterface.c
    src/cycleControl.c
    include/executionManagerWrapperInterface.h
    xme_hal_linkedList
    xme_hal_mem
    xme_hal_sleep
    xme_hal_sync
    xme_hal_sched
    xme_hal_table
    xme_hal_time
    xme_core_log
)

set(TEST_HELPER
    test/testHelper_executionManager.c
    test/testHelper_executionManager.h
    test/mHelperApplication/src/mHelperApplicationFunction.c
    test/mHelperApplication/src/mHelperApplicationFunctionWrapper.c
)

xme_unit_test("xme_core_executionManager"
    TYPE interfaceComponentRepo
    test/testExecutionManagerComponentRepositoryInterface.cpp
    test/testExecutionManager.cpp
    ${TEST_HELPER}
    xme_core_dataHandler
)

xme_unit_test("xme_core_executionManager"
    TYPE interfaceScheduleMgmt
    test/testExecutionManagerScheduleManagementInterface.cpp
    test/testExecutionManager.cpp
    ${TEST_HELPER}
    xme_core_dataHandler
)

#xme_unit_test("xme_core_executionManager"
#    TYPE integration
#    test/integrationTestExecutionManager.cpp
#    test/testExecutionManager.cpp
#    ${TEST_HELPER}
#    xme_core_dataHandler
#)

xme_unit_test("xme_core_executionManager"
    TYPE smokeTestScheduler_calculateNextComponent
    test/smokeTest_ExecutionManager_Scheduler_calculateNextComponent.cpp
    xme_core_dataHandler
)

xme_unit_test("xme_core_executionManager"
    TYPE smokeTestScheduler_registerSchedule
    test/smokeTest_ExecutionManager_Scheduler_registerSchedule.cpp
    xme_core_executionManager
    xme_core_dataHandler
)

xme_unit_test("xme_core_executionManager"
    TYPE smokeTestScheduler_activateSchedule
    test/smokeTest_ExecutionManager_Scheduler_activateSchedule.cpp
    xme_core_dataHandler
)


#xme_unit_test("xme_core_executionManager"
#    TYPE interfaceConfiguration
#    test/testExecutionManagerConfigurationInterface.cpp
#    ${TEST_HELPER}
#    xme_core_dataHandler
#)

#xme_unit_test("xme_core_executionManager"
#    TYPE smokeInit
#    test/smokeTestExecutionManagerInit.cpp
#    #test/testFunctionsExecutionManager.cpp
#    #test/testFunctionsExecutionManager.h
#    test/testExecutionManager.cpp
#    xme_core_dataHandler
#)


#xme_unit_test("xme_core_executionManager"
#    TYPE smokeNoInit
#    test/smokeTestExecutionManagerNoInit.cpp
##    test/testFunctionsExecutionManager.cpp
##    test/testFunctionsExecutionManager.h
#    test/testExecutionManager.cpp
#    xme_core_dataHandler
#)
