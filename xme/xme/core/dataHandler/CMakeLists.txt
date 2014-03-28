#
# Copyright (c) 2011-2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7684 2014-03-05 15:00:06Z ruiz $
#
# File:
#         CMake configuration file for XME Datahandler.
#

# Database Configurator
xme_add_component("xme_core_databaseConfigurator"
                  internal/databaseConfigurator.h
                  src/databaseConfigurator.c
                  internal/databaseInternalMethods.h
                  src/databaseInternalMethods.c
                  xme_hal_linkedList
                  xme_hal_circularBuffer
				  xme_hal_table
				  xme_hal_mmap)

# Database
xme_add_component("xme_core_database"
                  internal/database.h
                  src/database.c
                  internal/databaseInternalMethods.h
                  src/databaseInternalMethods.c
				  xme_core_databaseTestProbe
                  xme_hal_linkedList
                  xme_hal_circularBuffer
				  xme_hal_table
				  xme_hal_mmap)

xme_unit_test("xme_core_database" 
              TYPE smoke 
              test/smokeTestDatabase.cpp
              xme_core_databaseConfigurator)

# Database Builder
xme_add_component("xme_core_databaseBuilder"
    internal/databaseBuilder.h
    src/databaseBuilder.c
    xme_core_databaseConfigurator)

xme_unit_test("xme_core_databaseBuilder" 
              TYPE smoke 
              test/smokeTestDatabaseBuilder.cpp
              xme_core_databaseBuilder)

xme_unit_test("xme_core_databaseBuilder" 
              TYPE interface 
              test/interfaceTestDatabaseBuilder.cpp
              xme_core_databaseBuilder)			  
			  
# DataHandler
xme_add_component("xme_core_dataHandler"
                  include/dataHandler.h
                  include/dataHandlerConfigurator.h
                  src/dataHandler.c
				  xme_core_databaseBuilder
				  xme_core_database
                  xme_core_log
                  xme_core_broker
                  xme_hal_sync
                  xme_hal_tls)

xme_unit_test("xme_core_dataHandler" 
              TYPE smoke 
              test/smokeTestDataHandler.cpp)
              
xme_unit_test("xme_core_dataHandler" 
              TYPE interface 
              test/interfaceTestDataHandler.cpp)

xme_unit_test("xme_core_dataHandler"
			  TYPE measurement
			  test/measurementTestDataHandler.cpp
			  xme_hal_time
			  xme_core_log
			  xme_hal_random)

# Datatabase TestProbe
xme_add_component("xme_core_databaseTestProbe"
                  internal/databaseTestProbe.h
                  src/databaseTestProbe.c
				  internal/databaseTestProbeManipulations.h
                  xme_core_database)			  
			  
# DataHandler TestProbe
xme_add_component("xme_core_dataHandlerTestProbe"
                  include/dataHandlerTestProbe.h
                  src/dataHandlerTestProbe.c
				  xme_core_databaseTestProbe)

xme_unit_test("xme_core_dataHandlerTestProbe" 
              TYPE smoke 
              test/smokeTestDataHandlerTestProbe.cpp
			  xme_core_dataHandler
			  xme_core_broker)

xme_unit_test("xme_core_dataHandlerTestProbe" 
              TYPE interface 
              test/interfaceTestDataHandlerTestProbe.cpp
			  xme_core_dataHandler
			  xme_core_broker)
			  