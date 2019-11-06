// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_NODE_HANDLE_H
#define QML_ROS_PLUGIN_NODE_HANDLE_H

#include "qml_ros_plugin/qobject_ros.h"
#include <ros/node_handle.h>

namespace qml_ros_plugin
{

class NodeHandle : public QObjectRos
{
Q_OBJECT

  // @formatter:off
  //! The NodeHandle namespace. Only valid after isReady is true.
  Q_PROPERTY( QString ns READ ns )
  // @formatter:on
public:
  explicit NodeHandle( std::string ns = std::string());

  Q_INVOKABLE QObject *advertise( const QString &type, const QString &topic, quint32 queue_size, bool latch = false );

  bool isReady() const;

  ros::NodeHandle &nodeHandle();

  QString ns() const;

signals:

  void ready();

protected:
  void onRosInitialized() override;

protected:
  std::unique_ptr<ros::NodeHandle> nh_;
  std::string ns_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_NODE_HANDLE_H
