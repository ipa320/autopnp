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
# $Id: CMakeLists.txt 7678 2014-03-05 14:06:01Z geisinger $
#

xme_add_component(
    "xme_hal_graphviz"
    SOURCES
        xme/hal/graphviz_arch.cpp
    DEPENDS
        xme_hal_sync
    PACKAGES
        Graphviz
    INCLUDE_PATH
        VALUE_OF GRAPHVIZ_INCLUDE_PATH
    LINK
        TEMPLATE "gvc"
        TEMPLATE "graph"
        TEMPLATE "gvplugin_dot_layout"
)

xme_include_directory()
