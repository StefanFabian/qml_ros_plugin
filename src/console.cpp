// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/console.h"

namespace qml_ros_plugin
{


bool Console::setLoggerLevel( const QString &ros_console_name, ros_console_levels::RosConsoleLevel level )
{
  if ( !ros::console::set_logger_level( ros_console_name.toStdString(), static_cast<ros::console::Level>(level)))
    return false;
  ros::console::notifyLoggerLevelsChanged();
  return true;
}

QString Console::defaultName() const { return ROSCONSOLE_DEFAULT_NAME; }
}
