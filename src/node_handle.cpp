// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/publisher.h"
#include "qml_ros_plugin/ros.h"

#include <ros/callback_queue.h>

namespace qml_ros_plugin
{
NodeHandle::NodeHandle( std::string ns ) : ns_( std::move( ns ) ) { }

NodeHandle::NodeHandle( std::shared_ptr<ros::CallbackQueue> queue, std::string ns )
    : queue_( std::move( queue ) ), ns_( std::move( ns ) )
{
}

QObject *NodeHandle::advertise( const QString &type, const QString &topic, quint32 queue_size,
                                bool latch )
{
  return new Publisher( shared_from_this(), type, topic, queue_size, latch );
}

bool NodeHandle::isReady() const { return nh_ != nullptr; }

ros::NodeHandle &NodeHandle::nodeHandle() { return *nh_; }

void NodeHandle::onRosInitialized()
{
  if ( nh_ != nullptr )
    return;
  nh_.reset( new ros::NodeHandle( ns_ ) );
  if ( queue_ == nullptr )
    queue_ = RosQml::getInstance().callbackQueue();
  nh_->setCallbackQueue( queue_.get() );
  emit ready();
}

QString NodeHandle::ns() const
{
  return nh_ == nullptr ? QString() : QString::fromStdString( nh_->getNamespace() );
}
} // namespace qml_ros_plugin
