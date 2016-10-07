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
# $Id: Options.cmake 3915 2013-06-28 14:18:02Z gulati $
#
# File:
#         Waypoint UDP specific options.
#

xme_build_option(XME_WP_UDP_SEND_CONFIGURATIONTABLE_SIZE 100 "xme/xme_opt.h" "Maximum number of configuration entries for the udp send waypoint")
xme_build_option(XME_WP_UDP_RECEIVE_CONFIGURATIONTABLE_SIZE 100 "xme/xme_opt.h" "Maximum number of configuration entries for the udp receive waypoint")
xme_build_option(XME_WP_UDP_HEADER_KEY_LENGTH 4 "xme/xme_opt.h" "Maximum length of connecting KEY beteween udpSend and udpRecv, stored in config table of both WayPoints and is passed via packet header")
