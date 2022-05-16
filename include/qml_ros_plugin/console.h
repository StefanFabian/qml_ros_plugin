// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_CONSOLE_H
#define QML_ROS_PLUGIN_CONSOLE_H

#include <QtCore>
#include <ros/console.h>

namespace qml_ros_plugin
{

namespace ros_console_levels
{
Q_NAMESPACE

enum RosConsoleLevel {
  Debug = ros::console::levels::Debug,
  Info = ros::console::levels::Info,
  Warn = ros::console::levels::Warn,
  Error = ros::console::levels::Error,
  Fatal = ros::console::levels::Fatal,

  Count = ros::console::levels::Count
};

Q_ENUM_NS( RosConsoleLevel )
} // namespace ros_console_levels

class Console
{
  Q_GADGET

  // @formatter:off
  Q_PROPERTY( QString defaultName READ defaultName CONSTANT )
  // @formatter:on
public:
  /*!
   * Sets the logger level of the given console to the given level.
   * @param ros_console_name The console for which the level is changed.
   * @param level The new logging level.
   * @return True if successful, false otherwise.
   */
  Q_INVOKABLE bool setLoggerLevel( const QString &ros_console_name,
                                   qml_ros_plugin::ros_console_levels::RosConsoleLevel level );

  QString defaultName() const;
};
} // namespace qml_ros_plugin

Q_DECLARE_METATYPE( qml_ros_plugin::ros_console_levels::RosConsoleLevel );
Q_DECLARE_METATYPE( qml_ros_plugin::Console );

#endif // QML_ROS_PLUGIN_CONSOLE_H
