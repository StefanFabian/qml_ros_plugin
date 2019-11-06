// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TIME_H
#define QML_ROS_PLUGIN_TIME_H

#include "qml_ros_plugin/qobject_ros.h"

#include <QVariant>

namespace qml_ros_plugin
{

class Time : public QObjectRos
{
Q_OBJECT
public:
  /*!
   * Returns the ros::Time converted to a QDateTime.
   * This can be either the simulation time or the system time.
   *
   * Before the ros::Time got valid, this method returns nothing.
   * @return Time if valid, empty QVariant otherwise
   */
  Q_INVOKABLE QVariant now();

  //! @return Whether the time obtained using now() is the simulation time
  Q_INVOKABLE bool isSimTime();

  //! @return Whether the time obtained using now() is the system time
  Q_INVOKABLE bool isSystemTime();

  //! @return Whether the time obtained with now() is currently valid.
  Q_INVOKABLE bool isValid();
};

class WallTime : public QObject
{
Q_OBJECT
public:
  Q_INVOKABLE QDateTime now();
};
}

#endif //QML_ROS_PLUGIN_TIME_H
