// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/array.h"

#include <ros_babel_fish/message_types.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

namespace qml_ros_plugin
{

QVariantMap msgToMap( const std_msgs::Header &msg )
{
  QVariantMap result;
  result.insert( "frame_id", QString::fromStdString( msg.frame_id ));
  result.insert( "stamp", msg.stamp.toSec());
  result.insert( "seq", msg.seq );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Transform &msg )
{
  QVariantMap result;
  result.insert( "translation", QVariant::fromValue( msgToMap( msg.translation )));
  result.insert( "rotation", QVariant::fromValue( msgToMap( msg.rotation )));
  return result;
}

QVariantMap msgToMap( const geometry_msgs::TransformStamped &msg )
{
  QVariantMap result;
  result.insert( "header", QVariant::fromValue( msgToMap( msg.header )));
  result.insert( "child_frame_id", QString::fromStdString( msg.child_frame_id ));
  result.insert( "transform", QVariant::fromValue( msgToMap( msg.transform )));
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Vector3 &msg )
{
  QVariantMap result;
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Quaternion &msg )
{
  QVariantMap result;
  result.insert( "w", msg.w );
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}

QVariant msgToMap( const TranslatedMessage::ConstPtr &msg )
{
  return msgToMap( msg, *msg->translated_message );
}

QVariant msgToMap( const TranslatedMessage::ConstPtr &storage, const Message &msg )
{
  if ( msg.type() == MessageTypes::Compound )
  {
    QVariantMap result;
    const auto &compound = msg.as<CompoundMessage>();
    for ( size_t i = 0; i < compound.keys().size(); ++i )
    {
      result.insert( QString::fromStdString( compound.keys()[i] ), msgToMap( storage, *compound.values()[i] ));
    }
    return result;
  }
  else if ( msg.type() == MessageTypes::Array )
  {
    return QVariant::fromValue( new Array( storage, &msg.as<ArrayMessageBase>()));
  }
  switch ( msg.type())
  {
    case MessageTypes::Bool:
      return QVariant::fromValue( msg.as<ValueMessage<bool>>().getValue());
    case MessageTypes::UInt8:
      return QVariant::fromValue( msg.as<ValueMessage<uint8_t>>().getValue());
    case MessageTypes::UInt16:
      return QVariant::fromValue( msg.as<ValueMessage<uint16_t>>().getValue());
    case MessageTypes::UInt32:
      return QVariant::fromValue( msg.as<ValueMessage<uint32_t>>().getValue());
    case MessageTypes::UInt64:
      return QVariant::fromValue( msg.as<ValueMessage<uint64_t>>().getValue());
    case MessageTypes::Int8:
      return QVariant::fromValue( msg.as<ValueMessage<int8_t>>().getValue());
    case MessageTypes::Int16:
      return QVariant::fromValue( msg.as<ValueMessage<int16_t>>().getValue());
    case MessageTypes::Int32:
      return QVariant::fromValue( msg.as<ValueMessage<int32_t>>().getValue());
    case MessageTypes::Int64:
      return QVariant::fromValue( msg.as<ValueMessage<int64_t>>().getValue());
    case MessageTypes::Float32:
      return QVariant::fromValue( msg.as<ValueMessage<float>>().getValue());
    case MessageTypes::Float64:
      return QVariant::fromValue( msg.as<ValueMessage<double>>().getValue());
    case MessageTypes::String:
      return QVariant::fromValue( QString::fromStdString( msg.as<ValueMessage<std::string>>().getValue()));
    case MessageTypes::Time:
      return QVariant::fromValue( msg.as<ValueMessage<ros::Time>>().getValue().toSec());
    case MessageTypes::Duration:
      return QVariant::fromValue( msg.as<ValueMessage<ros::Duration>>().getValue().toSec());
    default:
      ROS_WARN( "Unknown type while processing message!" );
      break;
  }
  return QVariant();
}

namespace
{

template<typename T>
void fillValue( Message &msg, T value, uint32_t types )
{
  if ( !(msg.type() & types))
  {
    ROS_WARN( "Tried to fill field with incompatible type!" );
    return;
  }
  switch ( msg.type())
  {
    case MessageTypes::Bool:
      msg.as<ValueMessage<bool>>().setValue( static_cast<bool>(value));
      break;
    case MessageTypes::UInt8:
      msg.as<ValueMessage<uint8_t>>().setValue( static_cast<uint8_t>(value));
      break;
    case MessageTypes::UInt16:
      msg.as<ValueMessage<uint16_t>>().setValue( static_cast<uint16_t >(value));
      break;
    case MessageTypes::UInt32:
      msg.as<ValueMessage<uint32_t>>().setValue( static_cast<uint32_t >(value));
      break;
    case MessageTypes::UInt64:
      msg.as<ValueMessage<uint64_t>>().setValue( static_cast<uint64_t >(value));
      break;
    case MessageTypes::Int8:
      msg.as<ValueMessage<int8_t>>().setValue( static_cast<int8_t >(value));
      break;
    case MessageTypes::Int16:
      msg.as<ValueMessage<int16_t>>().setValue( static_cast<int16_t >(value));
      break;
    case MessageTypes::Int32:
      msg.as<ValueMessage<int32_t>>().setValue( static_cast<int32_t >(value));
      break;
    case MessageTypes::Int64:
      msg.as<ValueMessage<int64_t>>().setValue( static_cast<int64_t >(value));
      break;
    case MessageTypes::Float32:
      msg.as<ValueMessage<float>>().setValue( static_cast<float>(value));
      break;
    case MessageTypes::Float64:
      msg.as<ValueMessage<double>>().setValue( static_cast<double>(value));
      break;
    case MessageTypes::String:
      msg.as<ValueMessage<std::string>>().setValue( std::to_string( value ));
      break;
    case MessageTypes::Time:
      msg.as<ValueMessage<ros::Time>>().setValue( ros::Time( static_cast<double>(value)));
      break;
    case MessageTypes::Duration:
      msg.as<ValueMessage<ros::Duration>>().setValue( ros::Duration( static_cast<double>(value)));
      break;
    default:
      ROS_WARN( "Unknown type while filling message!" );
      break;
  }
}

template<>
void fillValue<std::string>( Message &msg, std::string value, uint32_t types )
{
  if ( msg.type() != MessageTypes::String )
  {
    ROS_WARN( "Tried to fill field with incompatible type!" );
    return;
  }
  auto &value_msg = msg.as<ValueMessage<std::string>>();
  value_msg.setValue( std::move( value ));
}

// Default implementation is for all integer types
template<typename T>
bool isCompatible( const QVariant &variant )
{

  if ( variant.type() == QVariant::UInt )
  {
    uint val = variant.toUInt();
    return val <= std::numeric_limits<T>::max();
  }
  if ( variant.type() == QVariant::ULongLong )
  {
    qulonglong val = variant.toULongLong();
    return val <= std::numeric_limits<T>::max();
  }
  if ( variant.type() == QVariant::Int )
  {
    int val = variant.toInt();
    return std::numeric_limits<T>::min() <= val && val <= std::numeric_limits<T>::max();
  }
  if ( variant.type() == QVariant::LongLong )
  {
    qlonglong val = variant.toLongLong();
    return std::numeric_limits<T>::min() <= val && val <= std::numeric_limits<T>::max();
  }
  if ( variant.type() == QVariant::Double )
  {
    double val = variant.toDouble();
    return val == std::round( val ) && std::numeric_limits<T>::min() <= val && std::numeric_limits<T>::max();
  }
  return false;
}

template<>
bool isCompatible<bool>( const QVariant &variant ) { return variant.type() == QVariant::Bool; }

template<>
bool isCompatible<float>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<double>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<ros::Time>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<ros::Duration>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong;
}

template<typename T>
T getValue( const QVariant &variant )
{
  switch ( variant.type())
  {
    case QVariant::Bool:
      return variant.toBool();
    case QVariant::Int:
      return static_cast<T>(variant.toInt());
    case QVariant::UInt:
      return static_cast<T>(variant.toUInt());
    case QVariant::LongLong:
      return static_cast<T>(variant.toLongLong());
    case QVariant::ULongLong:
      return static_cast<T>(variant.toULongLong());
    case QVariant::Double:
      return static_cast<T>(variant.toDouble());
    default:
      ROS_WARN( "Tried to get value from incompatible type!" );
  }
  return T();
}

template<>
ros::Time getValue<ros::Time>( const QVariant &variant )
{
  switch ( variant.type())
  {
    case QVariant::Int:
      return ros::Time( variant.toInt());
    case QVariant::UInt:
      return ros::Time( variant.toUInt());
    case QVariant::LongLong:
      return ros::Time( variant.toLongLong());
    case QVariant::ULongLong:
      return ros::Time( variant.toULongLong());
    case QVariant::Double:
      return ros::Time( variant.toDouble());
    case QVariant::Date:
    case QVariant::Time:
    case QVariant::DateTime:
    default:
      ROS_WARN( "Tried to get value from incompatible type!" );
  }
}

template<>
ros::Duration getValue<ros::Duration>( const QVariant &variant )
{
  switch ( variant.type())
  {
    case QVariant::Int:
      return ros::Duration( variant.toInt());
    case QVariant::UInt:
      return ros::Duration( variant.toUInt());
    case QVariant::LongLong:
      return ros::Duration( variant.toLongLong());
    case QVariant::ULongLong:
      return ros::Duration( variant.toULongLong());
    case QVariant::Double:
      return ros::Duration( variant.toDouble());
    case QVariant::Date:
    case QVariant::Time:
    case QVariant::DateTime:
    default:
      ROS_WARN( "Tried to get value from incompatible type!" );
  }
}

template<typename T>
void fillArray( Message &msg, const QVariantList &list )
{
  auto &array = msg.as<ArrayMessage<T>>();
  size_t count = list.size();
  if ( array.isFixedSize() && count > array.length())
    count = array.length();
  for ( size_t i = 0; i < count; ++i )
  {
    const QVariant &variant = list[i];
    if ( !isCompatible<T>( variant ))
    {
      ROS_WARN( "Tried to fill array with incompatible value! Skipped." );
      continue;
    }
    if ( array.isFixedSize()) array.setItem( i, getValue<T>( variant ));
    else array.addItem( getValue<T>( variant ));
  }
}
}

template<typename T>
using limits = std::numeric_limits<T>;

void fillMessage( ros_babel_fish::Message &msg, const QVariant &value )
{
  if ( value.typeName() == QString( "QVariantMap" ))
  {
    if ( msg.type() != MessageTypes::Compound )
    {
      ROS_WARN( "Invalid value passed to fillMessage!" );
      return;
    }
    auto &compound = msg.as<CompoundMessage>();
    const QVariantMap &map = *reinterpret_cast<const QVariantMap *>(value.data());
    for ( auto &key : map.keys())
    {
      std::string skey = key.toStdString();
      if ( !compound.containsKey( skey ))
      {
        ROS_WARN_STREAM( "Message doesn't have field '" << skey << "'!" );
        continue;
      }
      fillMessage( compound[skey], map[key] );
    }
    return;
  }
  else if ( value.typeName() == QString( "QVariantList" ))
  {
    const QVariantList &list = value.value<QVariantList>();
    if ( msg.type() != MessageTypes::Array )
    {
      ROS_WARN( "Invalid value passed to fillMessage!" );
      return;
    }
    auto &array = msg.as<ArrayMessageBase>();
    size_t count = list.size();
    if ( array.isFixedSize() && count > array.length())
    {
      ROS_WARN( "Too many values for fixed size array (%lu vs %lu)! Only using first %lu.", count, array.length(),
                array.length());
      count = array.length();
    }
    switch ( array.elementType())
    {
      case MessageTypes::None:
        break;
      case MessageTypes::Bool:
        fillArray<bool>( array, list );
        break;
      case MessageTypes::UInt8:
        fillArray<uint8_t>( array, list );
        break;
      case MessageTypes::UInt16:
        fillArray<uint16_t>( array, list );
        break;
      case MessageTypes::UInt32:
        fillArray<uint32_t>( array, list );
        break;
      case MessageTypes::UInt64:
        fillArray<uint64_t>( array, list );
        break;
      case MessageTypes::Int8:
        fillArray<int8_t>( array, list );
        break;
      case MessageTypes::Int16:
        fillArray<int16_t>( array, list );
        break;
      case MessageTypes::Int32:
        fillArray<int32_t>( array, list );
        break;
      case MessageTypes::Int64:
        fillArray<int64_t>( array, list );
        break;
      case MessageTypes::Float32:
        fillArray<float>( array, list );
        break;
      case MessageTypes::Float64:
        fillArray<double>( array, list );
        break;
      case MessageTypes::Time:
        fillArray<ros::Time>( array, list );
        break;
      case MessageTypes::Duration:
        fillArray<ros::Duration>( array, list );
        break;
      case MessageTypes::String:
      {
        auto &array = msg.as<ArrayMessage<Message>>();
        for ( size_t i = 0; i < count; ++i )
        {
          const QVariant &variant = list[i];
          if ( variant.type() != QVariant::String )
          {
            ROS_WARN( "Tried to fill string array with non-string value! Skipped." );
            continue;
          }
          auto child = new ValueMessage<std::string>( variant.toString().toStdString());
          if ( array.isFixedSize()) array.setItem( i, child );
          else array.addItem( child );
        }
        break;
      }
      case MessageTypes::Compound:
      {
        auto &array = msg.as<CompoundArrayMessage>();
        for ( size_t i = 0; i < count; ++i )
        {
          const QVariant &variant = list[i];
          if ( variant.type() != QVariant::Map )
          {
            ROS_WARN( "Tried to fill compound array with non-map value! Skipped." );
            continue;
          }
          auto child = new CompoundMessage( array.elementDataType());
          fillMessage( *child, variant );
          if ( array.isFixedSize()) array.setItem( i, child );
          else array.addItem( child );
        }
        break;
      }
      case MessageTypes::Array:
//      {
//        auto &array = msg.as<ArrayMessage<Message>>();
//        for ( size_t i = 0; i < count; ++i )
//        {
//          const QVariant &variant = list[i];
//          if ( variant.type() != QVariant::List )
//          {
//            ROS_WARN( "Tried to fill array of arrays with non-list value! Skipped." );
//            continue;
//          }
//          auto child = new ArrayMessage<Message>( MessageTypes::Array, );
//          if ( array.isFixedSize()) array.setItem( i, child );
//          else array.addItem( child );
//        }
        ROS_ERROR_ONCE( "Filling arrays of arrays is not yet implemented!" ); // TODO Implement
        break;
    }
    return;
  }
  if ( msg.type() & (MessageTypes::Array | MessageTypes::Compound))
  {
    ROS_WARN_STREAM( "Invalid type for value message: " << value.typeName());
    return;
  }
  switch ( value.type())
  {

    case QVariant::Invalid:
      break;
    case QVariant::Bool:
      fillValue<bool>( msg, value.toBool(),
                       MessageTypes::Bool ); // Since it is specifically a bool, we don't accept other types
      return;
    case QVariant::Int:
    {
      int val = value.toInt();
      uint32_t compatible_types = MessageTypes::Int32 | MessageTypes::Int64 |
                                  MessageTypes::Float32 | MessageTypes::Float64;
      // If it fits, it sits
      if ( limits<int8_t>::min() <= val && val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
      if ( limits<uint8_t>::min() <= val && val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
      if ( limits<int16_t>::min() <= val && val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
      if ( limits<uint16_t>::min() <= val && val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
      if ( val >= 0 ) compatible_types |= MessageTypes::UInt32 | MessageTypes::UInt64;
      fillValue<int>( msg, val, compatible_types );
      break;
    }
    case QVariant::UInt:
    {
      uint val = value.toUInt();
      uint32_t compatible_types = MessageTypes::UInt32 | MessageTypes::UInt64 | MessageTypes::Int64 |
                                  MessageTypes::Float32 | MessageTypes::Float64;
      if ( val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
      if ( val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
      if ( val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
      if ( val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
      if ( val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
      fillValue<uint>( msg, val, compatible_types );
      break;
    }
    case QVariant::LongLong:
    {
      qlonglong val = value.toLongLong();
      uint32_t compatible_types = MessageTypes::Int64 | MessageTypes::Float32 | MessageTypes::Float64;
      if ( limits<int8_t>::min() <= val && val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
      if ( limits<uint8_t>::min() <= val && val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
      if ( limits<int16_t>::min() <= val && val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
      if ( limits<uint16_t>::min() <= val && val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
      if ( limits<int32_t>::min() <= val && val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
      if ( val >= 0 ) compatible_types |= MessageTypes::UInt32 | MessageTypes::UInt64;
      fillValue<qlonglong>( msg, val, compatible_types );
      break;
    }
    case QVariant::ULongLong:
    {
      qulonglong val = value.toULongLong();
      uint32_t compatible_types = MessageTypes::UInt64 | MessageTypes::Float32 | MessageTypes::Float64;
      if ( val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
      if ( val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
      if ( val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
      if ( val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
      if ( val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
      if ( val <= limits<uint32_t>::max()) compatible_types |= MessageTypes::UInt32;
      if ( val <= limits<int64_t>::max()) compatible_types |= MessageTypes::Int64;
      fillValue<qulonglong>( msg, val, compatible_types );
      break;
    }
    case QVariant::Double:
    {
      double val = value.toDouble();
      uint32_t compatible_types = MessageTypes::Float32 | MessageTypes::Float64;
      if ( val == std::round( val ))
      {
        if ( limits<int8_t>::min() <= val && val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
        if ( limits<uint8_t>::min() <= val && val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
        if ( limits<int16_t>::min() <= val && val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
        if ( limits<uint16_t>::min() <= val && val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
        if ( limits<int32_t>::min() <= val && val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
        if ( limits<uint32_t>::min() <= val && val <= limits<uint32_t>::max()) compatible_types |= MessageTypes::UInt32;
        if ( limits<int64_t>::min() <= val && val <= limits<int64_t>::max()) compatible_types |= MessageTypes::Int64;
        if ( limits<uint64_t>::min() <= val && val <= limits<uint64_t>::max()) compatible_types |= MessageTypes::UInt64;
      }
      fillValue<double>( msg, value.toDouble(), compatible_types );
      break;
    }
    case QVariant::String:
      fillValue<std::string>( msg, value.toString().toStdString(), 0 /* ignored for this special case anyway */);
      break;
    case QVariant::Date: // Not sure if any of these types should be supported
    case QVariant::Time:
    case QVariant::DateTime:
    case QVariant::Char:
    default:
      ROS_WARN( "Unsupported QVariant type encountered while filling message!" );
      break;
  }
}
}
