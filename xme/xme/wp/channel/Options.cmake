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
# $Id: Options.cmake 6684 2014-02-07 14:34:26Z geisinger $
#
# File:
#         Channel waypoint specific options.
#

xme_build_option(XME_WP_CHANNEL_CHANNELINJECTOR_CONFIGURATIONTABLE_SIZE 10 "xme/xme_opt.h" "Maximum size of table to store the Channel Injector waypoint configuration")
xme_build_option(XME_WP_CHANNEL_CHANNELSELECTOR_CHANNELMAPPINGS_CONFIGURATIONTABLE_SIZE 10 "xme/xme_opt.h" "Maximum size of the table in Channel Selector waypoint to store the channel mappings of configuration")
xme_build_option(XME_WP_CHANNEL_CHANNELSELECTOR_INPUTPORTS_CONFIGURATIONTABLE_SIZE 10 "xme/xme_opt.h" "Maximum size of the table in Channel Selector wapoint to store the input ports configuration")
