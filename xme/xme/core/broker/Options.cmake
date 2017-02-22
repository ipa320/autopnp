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
# $Id: Options.cmake 4345 2013-07-24 11:00:48Z geisinger $
#
# File:
#         Build options file for XME Broker.
#

xme_build_option(XME_CORE_BROKER_TRANSFERTABLE_SIZE 512 "xme/xme_opt.h" "Used for declaring the size of the transfer table.")
xme_build_option(XME_CORE_BROKER_MAX_DATAPACKET_ITEMS 20 "xme/xme_opt.h" "defines the maximum number data packets that can be registered in the broker.")
xme_build_option(XME_CORE_BROKER_MAX_SUBSCRIBER_FUNCTIONS 25 "xme/xme_opt.h" "defines the maximum number of functions associated to a given data packet.")
xme_build_option(XME_CORE_BROKER_MAX_FUNCTION_DATA_ITEMS 30 "xme/xme_opt.h" "defines the maximum number of functions registered in the broker.")
xme_build_option(XME_CORE_BROKER_MAX_FUNCTION_PARAMETERS 15 "xme/xme_opt.h" "defines the maximum number of data packets (parameters) allowed for a function.")
