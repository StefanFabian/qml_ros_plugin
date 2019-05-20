// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/message_conversions.h"

using namespace ros_babel_fish;

namespace qml_ros_plugin
{
Array::Array() : translated_message_( nullptr ), message_( nullptr )
{
}

Array::Array( ros_babel_fish::TranslatedMessage::ConstPtr translated_message,
              const ros_babel_fish::ArrayMessageBase *message )
  : translated_message_( std::move( translated_message )), message_( message ) { }

//Array::Array( const Array &other ) : translated_message_( other.translated_message_ ), message_( other.message_ ) { }

qulonglong Array::length() const
{
  if ( message_ == nullptr ) return 0;
  return message_->length();
}

QVariant Array::get( uint index ) const
{
  if (index >= length())
  {
    throw std::runtime_error("Index out of array bounds!"); // TODO check if this is the correct behavior for QML
  }
  switch ( message_->elementType())
  {
    case MessageTypes::None:
      break;
    case MessageTypes::Bool:
      return QVariant::fromValue( message_->as<ArrayMessage<bool>>()[index] );
    case MessageTypes::UInt8:
      return QVariant::fromValue( message_->as<ArrayMessage<uint8_t>>()[index] );
    case MessageTypes::UInt16:
      return QVariant::fromValue( message_->as<ArrayMessage<uint16_t>>()[index] );
    case MessageTypes::UInt32:
      return QVariant::fromValue( message_->as<ArrayMessage<uint32_t>>()[index] );
    case MessageTypes::UInt64:
      return QVariant::fromValue( message_->as<ArrayMessage<uint64_t>>()[index] );
    case MessageTypes::Int8:
      return QVariant::fromValue( message_->as<ArrayMessage<int8_t>>()[index] );
    case MessageTypes::Int16:
      return QVariant::fromValue( message_->as<ArrayMessage<int16_t>>()[index] );
    case MessageTypes::Int32:
      return QVariant::fromValue( message_->as<ArrayMessage<int32_t>>()[index] );
    case MessageTypes::Int64:
      return QVariant::fromValue( message_->as<ArrayMessage<int64_t>>()[index] );
    case MessageTypes::Float32:
      return QVariant::fromValue( message_->as<ArrayMessage<float>>()[index] );
    case MessageTypes::Float64:
      return QVariant::fromValue( message_->as<ArrayMessage<double>>()[index] );
    case MessageTypes::Time:
      return QVariant::fromValue( message_->as<ArrayMessage<ros::Time>>()[index].toSec());
    case MessageTypes::Duration:
      return QVariant::fromValue( message_->as<ArrayMessage<ros::Duration>>()[index].toSec());
    default:
      break;
  }
  if ( cache_.size() <= index )
  {
    cache_.reserve( index + 1 );
    for ( size_t i = cache_.size(); i <= index; ++i )
    {
      cache_.push_back( QVariant());
    }
  }
  if ( !cache_[index].isValid())
  {
    cache_[index] = msgToMap( translated_message_, message_->as<ArrayMessage<Message>>()[index] );
  }
  return cache_[index];
}

QVariant Array::set( uint index, const QVariant &value )
{
  ROS_ERROR("Setting array values is not yet supported!");
  // TODO
  return QVariant();
}
} // qml_ros_plugin
