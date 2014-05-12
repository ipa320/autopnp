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
# $Id: Options.cmake 2833 2013-04-02 19:22:28Z rupanov $
#
# File:
#         Execution manager specific options.
#

# TODO: Add documentation, chose meaningful standard value
xme_build_option(XME_CORE_EXEC_TASKDESCRIPTORTABLE_SIZE 256 "xme/xme_opt.h" "Maximum number of separate tasks in the system")
xme_build_option(XME_CORE_EXEC_SCHEDULETABLE_SIZE 100 "xme/xme_opt.h" "Maximum number of items in a schedule table; on platforms with non-dynamic memory management")
xme_build_option(XME_CORE_EXEC_MAX_FUNCTIONS_PER_CHUNK 100 "xme/xme_opt.h" "Maximum number of functions per chunk; on platforms with non-dynamic memory management")
xme_build_option(XME_CORE_EXEC_MAX_FUNCTIONS_PER_COMPONENT 5 "xme/xme_opt.h" "Maximum number of functions per component; on platforms with non-dynamic memory management")
xme_build_option(XME_CORE_EXEC_MAX_RTE_FUNCTIONS 256 "xme/xme_opt.h" "Maximum number of functions in RTE; on platforms with non-dynamic memory management")

xme_build_option(XME_CORE_EXEC_SCHEDULER_MAX_SCHEDULES 100 "xme/xme_opt.h" "Maximum number of schedules; on platforms with non-dynamic memory management")
xme_build_option(XME_CORE_EXEC_REPOSITORY_MAX_COMPONENTS 64 "xme/xme_opt.h" "Maximum number of components; on platforms with non-dynamic memory management")
