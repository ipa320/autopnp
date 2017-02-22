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
# $Id: Options.cmake 7674 2014-03-05 12:25:50Z wiesmueller $
#
# File:
#         Build options file for XME plug and play.
#

xme_build_option(XME_CORE_PNP_LRM_INITIAL_CHANNELID 4096 "xme/xme_opt.h" "Initial last assigned channel identifier in Logical Route Manager.")
xme_build_option(XME_CORE_PNP_LRM_MAX_CHANNELID_MAPPINGS 32 "xme/xme_opt.h" "Maximum size of the table to store the channel ID associated with a given publication/subscription pair.")
xme_build_option(XME_CORE_DIRECTORY_NCC_NODE_TABLE_SIZE 20 "xme/xme_opt.h" "Maximum number of node entries")
xme_build_option(XME_CORE_DIRECTORY_NCC_INTERFACE_TABLE_SIZE 20 "xme/xme_opt.h" "Maximum number of Interfaces for a node")
xme_build_option(XME_CORE_DIRECTORY_NCC_PORTTOQUEUESIZE_TABLE_SIZE 255 "xme/xme_opt.h" "Maximum number of ports for which can be stored in port to queue size map (for automatically determining input port queue size).")
xme_build_option(XME_CORE_PNP_PNPMANAGER_INSTANCE_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of instances in plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAXIMUMNUMBEROFNODES_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of nodes in plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_LOGICALROUTES_LIST_SIZE 16 "xme/xme_opt.h" "Maximum number of logical routes supported in the plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_PHYSICALROUTES_LIST_SIZE 32 "xme/xme_opt.h" "Maximum number of physical routes supported in the plug and play manager.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_RUNTIMEGRAPHS_LIST_SIZE 20 "xme/xme_opt.h" "Maximum number of runtime graphs allowed during a component instance manifest notification.")
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_VERTICES_IN_RUNTIME_GRAPH 60 "xme/xme_opt.h" "Maximum number of vertices in a runtime graph.") # When adapting this, also update the runtimeGraphModel topic in XMT
xme_build_option(XME_CORE_PNP_PNPMANAGER_MAX_EDGES_IN_RUNTIME_GRAPH 60 "xme/xme_opt.h" "Maximum number of edges in a runtime graph.") # When adapting this,  also update the runtimeGraphModel topic in XMT
xme_build_option(XME_CORE_PNP_PNPMANAGER_COMPONENT_MAP_LIST_SIZE 16 "xme/xme_opt.h" "Maximum list size for component id and component type mapping.")
xme_build_option(XME_CORE_PNP_PNPCLIENT_CONFIG_TABLE_MAX 10 "xme/xme_opt.h" "Maximum number of configuration item in table for component/waypoints in plug and play client.")
xme_build_option(XME_CORE_PNP_PNPCLIENT_PORTS_MAX 100 "xme/xme_opt.h" "Maximum number of ports utilized by PnPClient in order to instantiate the components from RT Graph.")
xme_build_option(XME_CORE_PNP_PNPCLIENT_MAX_COMPONENT_ACTIONS 10 "xme/xme_opt.h" "Maximum number of concurrent component actions in the Plug and Play Client.")
xme_build_option(XME_CORE_PNP_PNPCLIENT_MAX_NODE_ACTIONS 10 "xme/xme_opt.h" "Maximum number of concurrent node actions in the Plug and Play Client.")
xme_build_option(XME_CORE_PNP_CONFIGEXT_CONFIGURATORS_MAX 10 "xme/xme_opt.h" "Maximum number of configurators that can be registered at once in the configurator extension.")
xme_build_option(XME_CORE_PNP_CONFIGEXT_EDGES_TO_REMOVE_MAX 255 "xme/xme_opt.h" "Maximum number of edges that can be removed in a single configurator execution.")
