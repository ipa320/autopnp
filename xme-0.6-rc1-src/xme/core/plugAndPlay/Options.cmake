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
# $Id: Options.cmake 5214 2013-09-27 14:24:34Z ruiz $
#
# File:
#         Build options file for XME plug and play.
#

xme_build_option(XME_CORE_DIRECTORY_NCC_NODE_TABLE_SIZE 20 "xme/xme_opt.h" "Maximum number of node entries")
xme_build_option(XME_CORE_DIRECTORY_NCC_INTERFACE_TABLE_SIZE 20 "xme/xme_opt.h" "Maximum number of Interfaces for a node")
xme_build_option(XME_CORE_PNP_PNPMANAGER_INSTANCE_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of instances in plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAXIMUMNUMBEROFNODES_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of nodes in plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_LOGICALROUTES_LIST_SIZE 16 "xme/xme_opt.h" "Maximum number of logical routes supported in the plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_PHYSICALROUTES_LIST_SIZE 32 "xme/xme_opt.h" "Maximum number of physical routes supported in the plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_RUNTIMEGRAPHS_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of runtime graphs allowed during a component instance manifest notification.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_PHYSICAL_ROUTE 20 "xme/xme_opt.h" "Maximum number of vertices in a physical route graph.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_PHYSICAL_ROUTE 20 "xme/xme_opt.h" "Maximum number of edges in a physical route graph.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH 16 "xme/xme_opt.h" "Maximum number of vertices in a runtime graph.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH 16 "xme/xme_opt.h" "Maximum number of edges in a runtime graph.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_COMPONENT_MAP_LIST_SIZE 16 "xme/xme_opt.h" "Maximum list size for component id and component type mapping.")
xme_build_option(XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX 10 "xme/xme_opt.h" "Maximum number of configuration item in table for component/waypoints in plug and play client.")
