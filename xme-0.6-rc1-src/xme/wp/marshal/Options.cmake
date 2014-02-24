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
# $Id: Options.cmake 4755 2013-08-23 12:30:26Z wiesmueller $
#
# File:
#         Waypoint specific options.
#

xme_build_option(
	XME_WP_MARSHAL_CONFIGURATIONTABLE_SIZE 
	100 # TODO: Choose a reasonable value (compare with allowed number of ports on a node)
	"xme/xme_opt.h" 
	"Maximum number of configuration entries for the marshaler and demarshaler waypoint"
)
