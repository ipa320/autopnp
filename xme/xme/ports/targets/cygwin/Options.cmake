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
# $Id: Options.cmake 3345 2013-05-17 12:07:58Z geisinger $
#

xme_add_subdirectory(../../hardware/cpu/x86 FALSE "Options.cmake")
xme_add_subdirectory(../../hardware/mcu/generic-x86 FALSE "Options.cmake")
xme_add_subdirectory(../../hardware/board/pc FALSE "Options.cmake")

xme_add_subdirectory(../../software/os/cygwin FALSE "Options.cmake")
