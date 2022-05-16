// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/subscriber.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/node_handle.h"

using namespace qml_ros_plugin::conversion;
using namespace ros_babel_fish;

namespace qml_ros_plugin
{

Subscriber::Subscriber() : running_( true ), is_subscribed_( false ), queue_size_( 1 )
{
  nh_ = std::make_shared<NodeHandle>();
  babel_fish_ = BabelFishDispenser::getBabelFish();
  connect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  initTimer();
}

Subscriber::Subscriber( NodeHandle::Ptr nh, QString topic, quint32 queue_size, bool running )
    : nh_( std::move( nh ) ), running_( running ), is_subscribed_( false ),
      topic_( std::move( topic ) ), queue_size_( queue_size )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  connect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  initTimer();
  if ( nh_->isReady() )
    subscribe();
}

Subscriber::~Subscriber() = default;

void Subscriber::initTimer()
{
  connect( &throttle_timer_, &QTimer::timeout, this, &Subscriber::updateMessage );
  throttle_timer_.setSingleShot( false );
  throttle_timer_.setInterval( 50 );
  throttle_timer_.start();
}

QString Subscriber::topic() const { return QString::fromStdString( subscriber_.getTopic() ); }

void Subscriber::setTopic( const QString &value )
{
  topic_ = value;
  subscribe();
  emit topicChanged();
}

quint32 Subscriber::queueSize() const { return queue_size_; }

void Subscriber::setQueueSize( quint32 value )
{
  queue_size_ = value;
  subscribe();
  emit queueSizeChanged();
}

QString Subscriber::ns() const { return nh_->ns(); }

void Subscriber::setNs( const QString &value )
{
  shutdown();
  disconnect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  nh_ = std::make_shared<NodeHandle>( value.toStdString() );
  connect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  if ( nh_->isReady() )
    subscribe();
  emit nsChanged();
}

bool Subscriber::running() const { return running_; }

void Subscriber::setRunning( bool value )
{
  if ( value == running_ )
    return;
  running_ = value;
  if ( running_ )
    subscribe();
  else
    shutdown();
  emit runningChanged();
}

int Subscriber::throttleRate() const { return throttle_rate_; }

void Subscriber::setThrottleRate( int value )
{
  throttle_rate_ = value;
  throttle_timer_.setInterval( 1000 / throttle_rate_ );
  emit throttleRateChanged();
}

const QVariant &Subscriber::message() const { return message_; }

const QString &Subscriber::messageType() const { return message_type_; }

unsigned int Subscriber::getNumPublishers()
{
  return is_subscribed_ ? subscriber_.getNumPublishers() : 0;
}

void Subscriber::onRosInitialized() { subscribe(); }

void Subscriber::subscribe()
{
  if ( is_subscribed_ ) {
    shutdown();
  }
  if ( topic_.isEmpty() )
    return;
  if ( nh_ == nullptr || !nh_->isReady() )
    return;
  subscriber_ = nh_->nodeHandle().subscribe<BabelFishMessage>( topic_.toStdString(), queue_size_,
                                                               &Subscriber::messageCallback, this );
  is_subscribed_ = true;
}

void Subscriber::shutdown()
{
  if ( !is_subscribed_ )
    return;
  subscriber_.shutdown();
  is_subscribed_ = false;
}

void Subscriber::messageCallback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  std::lock_guard<std::mutex> lock( message_mutex_ );
  last_message_ = msg;
}

void Subscriber::updateMessage()
{
  std::unique_lock<std::mutex> lock( message_mutex_ );
  if ( !last_message_ )
    return;
  ros_babel_fish::BabelFishMessage::ConstPtr msg = std::move( last_message_ );
  last_message_ = nullptr;
  lock.unlock(); // Don't need the mutex anymore
  TranslatedMessage::Ptr translated = babel_fish_.translateMessage( msg );
  message_ = msgToMap( translated, translated->translated_message->as<CompoundMessage>() );
  if ( msg->dataType() != message_type_.toStdString() ) {
    message_type_ = QString::fromStdString( msg->dataType() );
    emit messageTypeChanged();
  }
  emit messageChanged();
  emit newMessage( message_ );
}
} // namespace qml_ros_plugin
