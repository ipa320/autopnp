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
#         Build options file for XME directory services.
#

xme_build_option(XME_CORE_DIRECTORY_NODEREGISTRYCONTROLLER_NODE_TABLE_SIZE 20 "xme/xme_opt.h" "Maximum number of node entries.")
xme_build_option(XME_CORE_DIRECTORY_ATTRIBUTE_MAX_REGISTRY_SIZE 255 "xme/xme_opt.h" "Maximum size of attribute descriptors stored in the registry.")
xme_build_option(XME_CORE_DIRECTORY_ATTRIBUTE_MAX_TOPIC_ATTRIBUTE_ITEMS 16 "xme/xme_opt.h" "Maximum number of attributes associated to a given topic.")
xme_build_option(XME_CORE_DIRECTORY_ATTRIBUTE_MAX_ATTRIBUTE_SETS 16 "xme/xme_opt.h" "Maximum number of attribute sets in a given node. Note that the attribute storage is targeted for Plug and Play Manager.")
xme_build_option(XME_CORE_DIRECTORY_ATTRIBUTE_MAX_ATTRIBUTE_KEYS 16 "xme/xme_opt.h" "The maximum number of keys allowed in the keymap.")
