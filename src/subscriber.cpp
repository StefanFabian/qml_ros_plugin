// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/subscriber.h"
#include "qml_ros_plugin/message_conversions.h"

#include <chrono>

using namespace ros_babel_fish;

namespace qml_ros_plugin
{

Subscriber::Subscriber() : running_( true ), is_subscribed_( false ) { }

Subscriber::~Subscriber() = default;

const QString &Subscriber::topic() const { return topic_; }

void Subscriber::setTopic( const QString &value )
{
  topic_ = value;
  subscribe();
  emit topicChanged();
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

unsigned int Subscriber::getNumPublishers()
{
  return subscriber_.getNumPublishers();
}

void Subscriber::subscribe()
{
  if ( topic_.isEmpty()) return;
  if ( is_subscribed_ )
  {
    shutdown();
  }
  subscriber_ = nh_.subscribe( topic_.toStdString(), 10, &Subscriber::messageCallback, this );
  is_subscribed_ = true;
}

void Subscriber::shutdown()
{
  subscriber_.shutdown();
  is_subscribed_ = false;
}

namespace
{

// For debugging
void dumpMap( const QVariant &node, const QString &prefix = "" )
{
  if ( node.typeName() == QString( "QVariantMap" ))
  {
    const QVariantMap &map = *reinterpret_cast<const QVariantMap *>(node.data());
    for ( auto &key : map.keys())
    {
      std::cout << (prefix + key).toStdString() << ": " << map[key].typeName() << std::endl;
      dumpMap( map[key], prefix + "-" );
    }
  }
  else if ( node.typeName() == QString( "QVariantList" ))
  {
    const QVariantList &list = node.value<QVariantList>();
    for ( size_t i = 0; i < list.size(); ++i )
    {
      std::cout << prefix.toStdString() << " [" << i << "]:" << std::endl;
      dumpMap( list[i], prefix + "-" );
    }
  }
}
}

void Subscriber::messageCallback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg )
{
  TranslatedMessage::Ptr translated = babel_fish_.translateMessage( msg );
  message_ = msgToMap( translated, translated->translated_message->as<CompoundMessage>());
  emit messageChanged();
  emit newMessage( message_ );
}
}
