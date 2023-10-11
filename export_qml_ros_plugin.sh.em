#!/bin/bash
# This script is only needed for non-global install and will add the plugin to the path
# If you copy this file to add another package to the qml import path, note that the filename of this script needs to be unique!

@[if DEVELSPACE]@
  case $QML2_IMPORT_PATH in
    *"@(CATKIN_DEVEL_PREFIX)/@(CATKIN_PACKAGE_LIB_DESTINATION)/@(PROJECT_NAME)"*) ;;
    *) export QML2_IMPORT_PATH="@(CATKIN_DEVEL_PREFIX)/@(CATKIN_PACKAGE_LIB_DESTINATION)/@(PROJECT_NAME):${QML2_IMPORT_PATH}"
  esac
@[else]@
  case $QML2_IMPORT_PATH in
    *"@(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_LIB_DESTINATION)/@(PROJECT_NAME)"*) ;;
    *) export QML2_IMPORT_PATH="@(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_LIB_DESTINATION)/@(PROJECT_NAME):${QML2_IMPORT_PATH}"
  esac
@[end if]@
