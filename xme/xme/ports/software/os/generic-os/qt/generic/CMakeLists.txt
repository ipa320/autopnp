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
# $Id: CMakeLists.txt 7638 2014-03-03 09:44:10Z kainz $
#

function (xme_hal_qt_before_use_callback)
    ### Qt configuration ###
    # Qt modules (QtCore and QtGui are enabled by default)
    set(QT_USE_QTMAIN TRUE CACHE STRING "" FORCE)

    find_package(Qt4 REQUIRED)

    # Add some useful macros and variables
    # (QT_USE_FILE is a variable defined by find_package(Qt4) that contains a path to CMake script)
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITIONS})

    set(QT_LIBRARIES ${QT_LIBRARIES} CACHE STRING "" FORCE)
endfunction (xme_hal_qt_before_use_callback)

function (xme_hal_qt_after_use_callback)
    find_package(Qt4 REQUIRED)

    # Add some useful macros and variables
    # (QT_USE_FILE is a variable defined by find_package(Qt4) that contains a path to CMake script)
    include(${QT_USE_FILE})

    set_target_properties(xme_hal_qt PROPERTIES AUTOMOC TRUE)
endfunction (xme_hal_qt_after_use_callback)

xme_add_component(
    "xme_hal_qt"
    HEADERS
        xme/hal/QtApplication.h
    SOURCES
        xme/hal/QtApplication.cpp
        xme/hal/qt_arch.cpp
    DEPENDS
        xme_hal_cmdLine
        xme_hal_sched
        xme_hal_table
        xme_hal_time
    LINK
        VALUE_OF QT_LIBRARIES
    CALLBACK
        BEFORE_USE xme_hal_qt_before_use_callback
        AFTER_USE xme_hal_qt_after_use_callback
)

xme_include_directory()
