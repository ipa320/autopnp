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
# $Id: CMakeLists.txt 6367 2014-01-16 15:50:31Z geisinger $
#

xme_add_component (
    "configuratorExtension_configurator_demoConfigurator"
    demoConfigurator/include/demoConfigurator.h
    demoConfigurator/src/demoConfigurator.c
    DEPENDS xme_core_pnp_configuratorExtension
)
