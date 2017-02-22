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

xme_set (PROPERTY_GLOBAL XME_TARGET_OS "posix")

# Custom build options for this platform
xme_build_option (linux 1 "xme/xme_opt.h" "Compiling for Linux")
xme_build_option (__linux 1 "xme/xme_opt.h" "Compiling for Linux")
xme_build_option (__linux__ 1 "xme/xme_opt.h" "Compiling for Linux")

xme_build_option (HAVE_ISNAN 1 "xme/xme_opt.h" "Have isnan()")
xme_build_option (HAVE_RINT 1 "xme/xme_opt.h" "Have rint()")
xme_build_option (HAVE_STRTOK_R 1 "xme/xme_opt.h" "Have strtrok_r()")
