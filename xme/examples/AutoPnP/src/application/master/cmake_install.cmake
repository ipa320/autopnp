# Install script for directory: /home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/LICENSE.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src" TYPE DIRECTORY FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/tools")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src" TYPE DIRECTORY FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme" REGEX "/ports\\/[^/]*$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/build/AutoPnP/src/application/master" TYPE FILE RENAME "dummy" FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/tools/cmake/dummy.in")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/Makefile")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/moc_capabilitiesWindow.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/moc_QtApplication.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/ui_capabilitiesWindow.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/master.pro.user")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/xme_hal_qt_automoc.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/master.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/CMakeApplication.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/autoPnP_adv_exampleGUI_automoc.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/master.pro")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/cmake_install.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/link.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/freegetopt-build.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/callback.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/autoPnP_adv_capabilitiesView_automoc.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/DartConfiguration.tcl")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/ui_window.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/moc_window.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/application/master" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/CTestTestfile.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/cpu/x86_64" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/cpu/x86_64/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/cpu/x86_64" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/cpu/x86_64/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/mcu/generic-x86" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/mcu/generic-x86/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/mcu/generic-x86" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/mcu/generic-x86/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/board/pc" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/board/pc/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/hardware/board/pc" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/hardware/board/pc/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/software/os/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/software/os/posix/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/software/os/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/software/os/posix/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/targets/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/targets/posix/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/targets/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/targets/posix/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/broker" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/broker/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/broker" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/broker/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/dataHandler" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/dataHandler/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/dataHandler" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/dataHandler/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/directory" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/directory/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/directory" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/directory/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/executionManager" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/executionManager/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/executionManager" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/executionManager/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/login" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/login/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/login" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/login/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/manifestRepository" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/manifestRepository/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/manifestRepository" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/manifestRepository/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/nodeManager" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/nodeManager/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/nodeManager" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/nodeManager/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/plugAndPlay" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/plugAndPlay/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core/plugAndPlay" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/plugAndPlay/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/dataManagerTypes.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/node.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/dataChannel.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/topicData.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/device.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/node.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/topic.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/component.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/testUtils.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/componentContext.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/logUtils.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/componentInfo.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/log.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/componentList.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/log.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/deprecated.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/logUtils.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/coreTypes.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/componentInfo.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/manifestTypes.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/README.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/xme_api.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/container.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/core" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/core/executionManagerCallback.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/channel" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/channel/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/channel" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/channel/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/marshal" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/marshal/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/marshal" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/marshal/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/udp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/udp/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp/udp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/udp/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/waypoint.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/waypointConfigInfrastructure.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/waypointConfigInfrastructure.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/wp" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/wp/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/defines.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/defines.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/targets/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/targets/posix/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme/ports/targets/posix" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/ports/targets/posix/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/Options.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/defines.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/defines.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/xme" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/xme/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/external" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/external/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/capabilitiesViewComponentWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/capabilitiesViewComponent.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/addComponentFunctionWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/addComponentFunction.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/removeComponentFunctionWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/removeComponentFunction.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/capabilitiesViewManifest.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/include/capabilitiesWindow.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/capabilitiesViewComponentWrapper.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/capabilitiesViewComponent.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/addComponentFunctionWrapper.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/addComponentFunction.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/removeComponentFunctionWrapper.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/removeComponentFunction.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/capabilitiesViewManifest.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/src/capabilitiesWindow.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/capabilitiesView.pro")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/capabilitiesView.pro.user")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/capabilitiesView" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/capabilitiesView/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/exampleGUIComponentWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/exampleGUIComponent.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/doSomethingFunctionWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/doSomethingFunction.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/exampleGUIManifest.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/include/window.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/exampleGUIComponentWrapper.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/exampleGUIComponent.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/doSomethingFunctionWrapper.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/doSomethingFunction.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/exampleGUIManifest.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/src/window.cpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/autoPnP/adv/exampleGUI" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/autoPnP/adv/exampleGUI/CMakeLists.txt")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/include-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/include-gen/marshaler.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/include-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/include-gen/marshalerFunctionWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/src-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/src-gen/marshaler.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/src-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/src-gen/marshalerFunctionWrapper.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/include-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/include-gen/demarshaler.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/include-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/include-gen/demarshalerFunctionWrapper.h")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/src-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/src-gen/demarshaler.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples/AutoPnP/src/xme/wp/marshal/src-gen" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/xme/wp/marshal/src-gen/demarshalerFunctionWrapper.c")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master"
         RPATH "")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64" TYPE EXECUTABLE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/target/master")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master")
    FILE(RPATH_REMOVE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/posix_x86_64/master")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/examples" TYPE DIRECTORY FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master" REGEX "/[^/]*\\/build[^/]*$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/.catkin")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/.catkin")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/_setup_util.py")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE PROGRAM FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/_setup_util.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/env.sh")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE PROGRAM FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/env.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/setup.bash")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/setup.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/setup.sh")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/setup.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/setup.zsh")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/setup.zsh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install/.rosinstall")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/install" TYPE FILE FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/catkin_generated/installspace/.rosinstall")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/groovy/share/catkin/cmake/env-hooks/05.catkin_make.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/groovy/share/catkin/cmake/env-hooks/05.catkin_make_isolated.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/html")
FILE(INSTALL DESTINATION "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master" TYPE DIRECTORY FILES "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/CMakeFiles/master_doc.doxygen.dir/html" REGEX "/[^/]*\\.dot$" EXCLUDE REGEX "/[^/]*\\.map$" EXCLUDE REGEX "/[^/]*\\.md5$" EXCLUDE)
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/XME_ROOT/xme/ports/targets/posix/cmake_install.cmake")
  INCLUDE("/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/XME_ROOT/xme/cmake_install.cmake")
  INCLUDE("/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/XME_ROOT/external/cmake_install.cmake")
  INCLUDE("/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/gtest/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/rmb-om/git/care-o-bot/autopnp/xme/examples/AutoPnP/src/application/master/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
