// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/publisher.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/node_handle.h"
#include <ros/ros.h>

using namespace ros_babel_fish;
using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{

Publisher::Publisher( NodeHandle::Ptr nh, QString type, QString topic, uint32_t queue_size,
                      bool latch )
    : nh_( std::move( nh ) ), is_advertised_( false ), type_( std::move( type ) ),
      topic_( std::move( topic ) ), is_latched_( latch ), queue_size_( queue_size )
{
  std_type_ = type_.toStdString();
  babel_fish_ = BabelFishDispenser::getBabelFish();

  QObject::connect( nh_.get(), &NodeHandle::ready, this, &Publisher::onNodeHandleReady );
  // Advertise if NodeHandle is ready, i.e., ROS was initialized and the NodeHandle is valid otherwise wait for ready
  if ( nh_->isReady() ) {
    advertise();
    QObject::disconnect( nh_.get(), &NodeHandle::ready, this, &Publisher::onNodeHandleReady );
  }
}

Publisher::~Publisher() = default;

QString Publisher::topic() const { return QString::fromStdString( publisher_.getTopic() ); }

const QString &Publisher::type() const { return type_; }

bool Publisher::isLatched() const { return publisher_.isLatched(); }

quint32 Publisher::queueSize() const { return queue_size_; }

bool Publisher::isAdvertised() const { return is_advertised_; }

unsigned int Publisher::getNumSubscribers() { return publisher_.getNumSubscribers(); }

bool Publisher::publish( const QVariantMap &msg )
{
  if ( !is_advertised_ )
    return false;
  try {
    Message::Ptr message = babel_fish_.createMessage( type_.toStdString() );
    if ( message == nullptr )
      return false;
    if ( !fillMessage( *message, msg ) )
      return false;
    BabelFishMessage::Ptr bf_message = babel_fish_.translateMessage( message );
    publisher_.publish( bf_message );
    return true;
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to publish message: %s", ex.what() );
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to publish message: %s", ex.what() );
  }
  return false;
}

void Publisher::onNodeHandleReady()
{
  QObject::disconnect( nh_.get(), &NodeHandle::ready, this, &Publisher::onNodeHandleReady );
  if ( !is_advertised_ )
    advertise();
}

void Publisher::advertise()
{
  if ( is_advertised_ )
    publisher_.shutdown();
  if ( type_.isEmpty() )
    return;
  if ( topic_.isEmpty() )
    return;
  if ( queue_size_ == 0 )
    return;
  try {
    ros::SubscriberStatusCallback connected_cb =
        boost::bind( &Publisher::onSubscriberConnected, this, _1 );
    ros::SubscriberStatusCallback disconnected_cb =
        boost::bind( &Publisher::onSubscriberDisconnected, this, _1 );
    publisher_ = babel_fish_.advertise( nh_->nodeHandle(), std_type_, topic_.toStdString(),
                                        queue_size_, is_latched_, connected_cb, disconnected_cb );
    is_advertised_ = true;
    emit advertised();
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to create publisher: %s", ex.what() );
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to create publisher: %s", ex.what() );
  }
}

void Publisher::onSubscriberConnected( const ros::SingleSubscriberPublisher & )
{
  //  auto publisher = new SingleSubscriberPublisher( pub, babel_fish_, std_type_ );
  emit connected();
}

void Publisher::onSubscriberDisconnected( const ros::SingleSubscriberPublisher & )
{
  //  auto publisher = new SingleSubscriberPublisher( pub, babel_fish_, std_type_ );
  emit disconnected();
}
} // namespace qml_ros_plugin
