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
# $Id: CMakeLists.txt 2898 2013-04-12 18:10:32Z geisinger $
#

xme_add_executable(
	unitTestingTests
	unitTestingTests.cpp
	unitTestingTestsUtils.h
	unitTestingTestsUtils.c
)

xme_link_components(
	unitTestingTests
	xme_hal_mem
	xme_hal_sync
)

find_package (Threads)
target_link_libraries(
	unitTestingTests
	gtest
	${CMAKE_THREAD_LIBS_INIT}
)
