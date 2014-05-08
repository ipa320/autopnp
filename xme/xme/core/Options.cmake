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
# $Id: Options.cmake 7483 2014-02-18 16:14:01Z wiesmueller $
#
# File:
#         Core options script.
#

xme_build_option(XME_CORE_LOG_MAX_COMPONENT_REGISTRY_ITEMS 32 "xme/xme_opt.h" "Maximum number of components in logging utility component registry.")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT 32 "xme/xme_opt.h" "Maximum number of port per single component.")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT 16 "xme/xme_opt.h" "Maximum number of functions per single component.")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_FUNCTION 10 "xme/xme_opt.h" "Maximum number of required and optional ports per function.")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_COMPONENTS_PER_CONTAINER 10 "xme/xme_opt.h" "Maximum number of components inside a container")
xme_build_option(XME_CORE_NODE_INTERFACE_TABLE_SIZE 10 "xme/xme_opt.h" "Maximum number of interface addresses associated to one single node.")
xme_build_option(XME_CORE_TOPIC_PNP_COMPONENTINSTANCEMANIFEST_LENGTH 10 "xme/xme_opt.h" "Maximum number of components in component instance manifest topic.")

xme_add_subdirectory("broker" FALSE "Options.cmake")
xme_add_subdirectory("dataHandler" FALSE "Options.cmake")
xme_add_subdirectory("directory" FALSE "Options.cmake")
xme_add_subdirectory("executionManager" FALSE "Options.cmake")
xme_add_subdirectory("login" FALSE "Options.cmake")
xme_add_subdirectory("manifestRepository" FALSE "Options.cmake")
xme_add_subdirectory("nodeManager" FALSE "Options.cmake")
xme_add_subdirectory("plugAndPlay" FALSE "Options.cmake")
