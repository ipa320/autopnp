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
# $Id: CMakeLists.txt 4595 2013-08-07 13:49:46Z ruiz $
#
# File:
#         CMake configuration file for XME Datahandler.
#

xme_add_component("xme_core_dataHandler"
                  include/dataHandler.h
                  include/dataHandlerInternalTypes.h
                  include/dataHandlerTestsystemInterface.h
                  include/auditHandler.h
                  src/auditHandler.c
                  src/dataHandler.c                  
                  src/dataFlowHandler.c
                  src/dataHandlerShadow.c
                  src/dataHandlerInternalMethods.c
                  xme_core_log
                  xme_core_broker)

xme_unit_test("xme_core_dataHandler" 
              TYPE smoke 
              test/smokeTestDataHandler.cpp
              xme_core_executionManager)
              
xme_unit_test("xme_core_dataHandler" 
              TYPE interface 
              test/interfaceTestDataHandler.cpp
              test/interfaceTestDataHandlerInternalMethods.cpp
              test/interfaceTestDataHandlerShadow.cpp
              test/interfaceTestAuditHandler.cpp
              xme_core_executionManager)
              
xme_unit_test("xme_core_dataHandler" 
              TYPE integration 
              test/integrationTestDataHandler.cpp
              test/integrationTestDataHandlerShadow.cpp
              xme_core_executionManager)
