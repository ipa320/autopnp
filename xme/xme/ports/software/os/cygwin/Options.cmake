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
#         CMake options for Cygwin platform port.
#

xme_set (PROPERTY_GLOBAL XME_TARGET_OS "cygwin")

# Custom build options for this platform
xme_build_option (CYGWIN 1 "xme/xme_opt.h" "Compiling for CYGWIN")
xme_build_option (__CYGWIN__ 1 "xme/xme_opt.h" "Compiling for CYGWIN")

# _WIN32 should not be defined on this platform!
# (notice that _WIN32 and WIN32 might still be defined on CYGWIN if
# certain conditions apply; in this case you might want to adapt these checks)
xme_build_option_check (_WIN32 UNDEFINED "xme/xme_opt.h" "Ensure that we are not compiling for native Windows")
xme_build_option_check (WIN32 UNDEFINED "xme/xme_opt.h" "Ensure that we are not compiling for native Windows")
xme_build_option_check (__WIN32__ UNDEFINED "xme/xme_opt.h" "Ensure that we are not compiling for native Windows")
