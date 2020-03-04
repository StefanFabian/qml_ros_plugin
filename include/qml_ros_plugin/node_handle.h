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

  std::unique_ptr<ros::NodeHandle> nh_;
  std::string ns_;
};

struct NodeHandleReference
{
  explicit NodeHandleReference( NodeHandle *nh, bool owned = false ) : node_handle( nh ), owned_( owned ) { }

  NodeHandleReference( const NodeHandleReference &other ) noexcept : node_handle( other.node_handle ), owned_( false ) { }

  ~NodeHandleReference()
  {
    if ( owned_ ) delete node_handle;
  }

  NodeHandle &operator*() { return *node_handle; }

  NodeHandle *operator->() { return node_handle; }

  NodeHandle *get() { return node_handle; }

  NodeHandle *node_handle;
private:
  bool owned_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_NODE_HANDLE_H
