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
# $Id: Options.cmake 7672 2014-03-05 08:04:41Z ruiz $
#
# File:
#         Data handler specific options.
#

xme_build_option(XME_CORE_DATAHANDLER_MAXDATASTORE_COUNT 1024 "xme/xme_opt.h" "The maximum number of data stores that can be declared for XME.")
xme_build_option(XME_CORE_DATAHANDLER_MAXDATASTORE_ATTRIBUTES 256 "xme/xme_opt.h" "Maximum number of attributes that can be defined for a given data packet.")
xme_build_option(XME_CORE_DATAHANDLER_MAXATTRIBUTE_COUNT 1024 "xme/xme_opt.h" "The maximum number of attributes that can be defined for the data handler.")
xme_build_option(XME_CORE_DATAHANDLER_MAXMEMORYREGION_COUNT 16 "xme/xme_opt.h" "The maximum number of memory regions that can be declared for XME.")
xme_build_option(XME_CORE_DATAHANDLER_MAXMEMORYREGIONCOPIES_COUNT 8 "xme/xme_opt.h" "Maximum number of copies inside a given memory region.")

xme_build_option(XME_CORE_DATAHANDLER_CRITICALSECTION_CONCURRENT 8 "xme/xme_opt.h" "The maximum number of data packets concurrently getting a critical section handle.")

xme_build_option(XME_CORE_DATAHANDLER_DATABASETESTPROBE_MAXMANIPULATIONS 64 "xme/xme_opt.h" "The maximum number of manipulations.")
