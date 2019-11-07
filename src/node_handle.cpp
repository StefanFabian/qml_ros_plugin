// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/node_handle.h"

#include "qml_ros_plugin/publisher.h"

namespace qml_ros_plugin
{
NodeHandle::NodeHandle( std::string ns ) : ns_( std::move( ns )) { }

QObject *NodeHandle::advertise( const QString &type, const QString &topic, quint32 queue_size, bool latch )
{
  return new Publisher( this, type, topic, queue_size, latch );
}

bool NodeHandle::isReady() const
{
  return nh_ != nullptr;
}

ros::NodeHandle &NodeHandle::nodeHandle()
{
  return *nh_;
}

void NodeHandle::onRosInitialized()
{
  if ( nh_ != nullptr ) return;
  nh_.reset( new ros::NodeHandle( ns_ ));
  emit ready();
}

QString NodeHandle::ns() const
{
  return nh_ == nullptr ? QString() : QString::fromStdString( nh_->getNamespace());
}
} // qml_ros_plugin