#
# Copyright (c) 2011-2012, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
#
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: Options.cmake 3345 2013-05-17 12:07:58Z geisinger $
#
# File:
#         XME compiler options script.
#
# TODO: Verify that below default settings make sense
#
# TODO: Is it correct that these options are in namespace
#       XME_HAL, while they are used in XME_CORE?
#
# TODO: Some of these options should probably be overriden in the respective
#       XME hal library instance.

xme_build_option(XME_HAL_DEFINES_MAX_COMPONENT_PORT_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_COMPONENT_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_LOCAL_PUBLICATION_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_LOCAL_SUBSCRIPTION_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_LOCAL_REQUEST_SENDER_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_LOCAL_REQUEST_HANDLER_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_LOCAL_SOURCE_ROUTE_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_LOCAL_DESTINATION_ROUTE_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_OUTBOUND_ROUTE_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_INBOUND_ROUTE_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_TRANSLATION_ROUTE_ITEMS 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_DATACHANNELMAPPING_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_NODE_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_COMMUNICATION_LINKS_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_LINK_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_DATACHANNEL_TRANSLATION_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_REMOTE_MODIFY_ROUTING_TABLE_DATACHANNEL_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_ENDPOINT_LINK_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_PUBLICATION_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_SUBSCRIPTION_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_REQUEST_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_REQUEST_HANDLER_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_PATH_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_DIRECTORY_COMMUNICATION_RELATION_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_LOGIN_SERVER_NODE_ID_ASSIGNMENT_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_TOPIC_META_DATA_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_TOPIC_META_DATA_DESCRIPTORS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_RESOURCEMANAGER_LOCAL_RESPONSE_INSTANCE_ITEMS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_SOCKETS 5 "xme/xme_opt.h" "Description missing")

xme_build_option(XME_HAL_DEFINES_MAX_TASK_DESCRIPTORS 8 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_CRITICAL_SECTION_DESCRIPTORS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DEFINES_MAX_TLS_ITEMS 5 "xme/xme_opt.h" "Description missing")

# Option used by xme/prim components
xme_build_option(XME_HAL_DEFINES_MAX_IPLOGINSERVER_PENDINGRESPONSE_ITEMS 5 "xme/xme_opt.h" "Description missing")

xme_build_option(XME_HAL_SCHED_PRIORITY_NORMAL 127 "xme/xme_opt.h" "Description missing")

# These values match the EK-LM3S8962. If needed, create a Options.cmake in the respective
# subdirectory of xme/ports and preset a different value (using xme_build_option_set())
xme_build_option(XME_HAL_CHAR_SIZE_X 6 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_CHAR_SIZE_Y 8 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DISPLAY_SIZE_X 128 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_DISPLAY_SIZE_Y 96 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_CONSOLE_CHAR_COUNT_X 21 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_CONSOLE_CHAR_COUNT_Y 10 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_CONSOLE_CHAR_OFFSET_X 0 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_CONSOLE_CHAR_OFFSET_Y 0 "xme/xme_opt.h" "Description missing")

# For embedded platforms, the size of TLS (thread local storage) is limited
xme_build_option(XME_HAL_TLS_MAX_TASKS_USING_TLS 5 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_HAL_TLS_MAX_ALLOCATABLE_TLS 2 "xme/xme_opt.h" "Description missing")

# For embedded platforms, the number of tracked neighbors is fixed
xme_build_option(XME_PRIM_NEIGHBORHOODISCOVERY_MAX_NUMBER_OF_TRACKED_NEIGHBORS 16 "xme/xme_opt.h" "Description missing")

# the maximum number of slots and tasks in a slot for the Time Division Multiple Access (TDMA) scheduler
xme_build_option(XME_PRIM_TDMA_MAX_SLOT_COUNT 16 "xme/xme_opt.h" "Description missing")
xme_build_option(XME_PRIM_TDMA_MAX_TASK_COUNT_IN_SLOT 16 "xme/xme_opt.h" "Description missing")

xme_add_subdirectory("core" FALSE "Options.cmake")
xme_add_subdirectory("wp" FALSE "Options.cmake")
