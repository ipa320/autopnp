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
# $Id: CMakeLists.txt 7314 2014-02-13 16:34:40Z geisinger $
#

xme_add_component(
    "xme_hal_xml"
    xme/hal/xml_arch.cpp
    xme_hal_safeString
    PACKAGES TinyXML2
    TARGETS tinyxml2
    INCLUDE_PATH VALUE_OF TINYXML2_INCLUDE_DIR
    LINK VALUE_OF TINYXML2_LIB_FILE_STATIC
)

xme_include_directory()
