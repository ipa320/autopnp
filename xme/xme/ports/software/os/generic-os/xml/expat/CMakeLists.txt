#
# Copyright (c) 2011-2014, fortiss GmbH.
# Licensed under the Apache License, Version 2.0.
# 
# Use, modification and distribution are subject to the terms specified
# in the accompanying license file LICENSE.txt located at the root directory
# of this software distribution. A copy is available at
# http://chromosome.fortiss.org/.
#
# This file is part of CHROMOSOME.
#
# $Id: CMakeLists.txt 7726 2014-03-10 10:45:38Z geisinger $
#

xme_add_component(
    "xme_hal_xml"
    xme/hal/xml_arch.c
    DEPENDS
        xme_hal_safeString
    PROPERTIES
        COMPILE_DEFINITIONS XML_STATIC
    PACKAGES
        Expat
    TARGETS
        expat
    INCLUDE_PATH
        VALUE_OF EXPAT_INCLUDE_DIR
    LINK
        VALUE_OF EXPAT_LIB_FILE_STATIC
)

xme_include_directory()
