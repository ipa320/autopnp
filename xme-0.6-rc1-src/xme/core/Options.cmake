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
# $Id: Options.cmake 4437 2013-07-31 13:55:03Z ruiz $
#
# File:
#         Core options script.
#

xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_PORTS_PER_COMPONENT 10 "xme/xme_opt.h" "Maximum number of port per single component.")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_FUNCTIONS_PER_COMPONENT 10 "xme/xme_opt.h" "Maximum number of functions per component")
xme_build_option(XME_CORE_MANIFEST_TYPES_MAX_COMPONENTS_PER_CONTAINER 10 "xme/xme_opt.h" "Maximum number of components inside a container")
xme_build_option(XME_CORE_NODE_INTERFACE_TABLE_SIZE 10 "xme/xme_opt.h" "Maximum number of interface addresses associated to one single node.")

xme_add_subdirectory("broker" FALSE "Options.cmake")
xme_add_subdirectory("dataHandler" FALSE "Options.cmake")
xme_add_subdirectory("directory" FALSE "Options.cmake")
xme_add_subdirectory("executionManager" FALSE "Options.cmake")
xme_add_subdirectory("manifestRepository" FALSE "Options.cmake")
xme_add_subdirectory("login" FALSE "Options.cmake")
xme_add_subdirectory("plugAndPlay" FALSE "Options.cmake")
