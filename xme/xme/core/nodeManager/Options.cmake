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
# $Id: Options.cmake 7483 2014-02-18 16:14:01Z wiesmueller $
#
# File:
#         Build options file for node manager.
#

xme_build_option(XME_CORE_NODEMGR_COMPREP_COMPONENT_TABLE_MAX 100 "xme/xme_opt.h" "Maximum number of component instances in the component repository.")
xme_build_option(XME_CORE_NODEMGR_COMPREP_FUNCTION_TABLE_MAX 200 "xme/xme_opt.h" "Maximum number of function instances in the component repository (across all components).")
xme_build_option(XME_CORE_NODEMGR_COMPREP_PORT_TABLE_MAX 400 "xme/xme_opt.h" "Maximum number of port instances in the component repository (across all components).")
