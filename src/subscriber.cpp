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
}

Subscriber::Subscriber( NodeHandle::Ptr nh, QString topic, quint32 queue_size, bool running )
  : nh_( std::move( nh )), running_( running ), is_subscribed_( false ), topic_( std::move( topic )), queue_size_(
  queue_size )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  connect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  if ( nh_->isReady()) subscribe();
}

Subscriber::~Subscriber() = default;

QString Subscriber::topic() const { return QString::fromStdString( subscriber_.getTopic()); }

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
  nh_ = std::make_shared<NodeHandle>( value.toStdString());
  connect( nh_.get(), &NodeHandle::ready, this, &Subscriber::subscribe );
  if ( nh_->isReady())subscribe();
  emit nsChanged();
}

bool Subscriber::running() const { return running_; }

void Subscriber::setRunning( bool value )
{
  if ( value == running_ ) return;
  running_ = value;
  if ( running_ ) subscribe();
  else shutdown();
  emit runningChanged();
}

const QVariant &Subscriber::message() const { return message_; }

const QString &Subscriber::messageType() const { return message_type_; }

unsigned int Subscriber::getNumPublishers() { return is_subscribed_ ? subscriber_.getNumPublishers() : 0; }

void Subscriber::onRosInitialized()
{
  subscribe();
}

void Subscriber::subscribe()
{
  if ( is_subscribed_ )
  {
    shutdown();
  }
  if ( topic_.isEmpty()) return;
  if ( nh_ == nullptr || !nh_->isReady()) return;
  subscriber_ = nh_->nodeHandle().subscribe<BabelFishMessage>( topic_.toStdString(), queue_size_,
                                                               &Subscriber::messageCallback, this );
  is_subscribed_ = true;
}

void Subscriber::shutdown()
{
  if ( !is_subscribed_ ) return;
  subscriber_.shutdown();
  is_subscribed_ = false;
}

void Subscriber::messageCallback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  TranslatedMessage::Ptr translated = babel_fish_.translateMessage( msg );
  message_ = msgToMap( translated, translated->translated_message->as<CompoundMessage>());
  if ( msg->dataType() != message_type_.toStdString())
  {
    message_type_ = QString::fromStdString( msg->dataType());
    emit messageTypeChanged();
  }
  emit messageChanged();
  emit newMessage( message_ );
}
}
