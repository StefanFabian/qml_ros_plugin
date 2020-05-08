// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

#include <QAbstractListModel>
#include <QDateTime>
#include <QMetaProperty>

#include <ros_babel_fish/message_types.h>
#include <ros/ros.h>

using namespace ros_babel_fish;

namespace qml_ros_plugin
{
namespace conversion
{

QVariantMap msgToMap( const std_msgs::Header &msg )
{
  QVariantMap result;
  result.insert( "frame_id", QString::fromStdString( msg.frame_id ));
  result.insert( "stamp", rosToQmlTime( msg.stamp ));
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

QVariantMap msgToMap( const actionlib_msgs::GoalID &msg )
{
  QVariantMap result;
  result.insert( "id", QString::fromStdString( msg.id ));
  result.insert( "stamp", rosToQmlTime( msg.stamp ));
  return result;
}

QVariantMap msgToMap( const actionlib_msgs::GoalStatus &msg )
{
  QVariantMap result;
  result.insert( "goal_id", QVariant::fromValue( msgToMap( msg.goal_id )));
  result.insert( "status", msg.status );
  result.insert( "text", QString::fromStdString( msg.text ));
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
    return QVariant::fromValue( Array( storage, &msg.as<ArrayMessageBase>()));
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
    {
      return QVariant::fromValue( rosToQmlTime( msg.value<ros::Time>()));
    }
    case MessageTypes::Duration:
      return QVariant::fromValue( rosToQmlDuration( msg.as<ValueMessage<ros::Duration>>().getValue()));
    default:
      ROS_WARN( "Unknown type '%d' while processing message!", msg.type());
      break;
  }
  return QVariant();
}

namespace
{

template<typename T>
bool fillValue( Message &msg, T value, uint32_t types )
{
  if ( !(msg.type() & types))
  {
    ROS_WARN( "Tried to fill '%s' field with incompatible type!", typeid( T ).name());
    return false;
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
      msg.as<ValueMessage<uint16_t>>().setValue( static_cast<uint16_t>(value));
      break;
    case MessageTypes::UInt32:
      msg.as<ValueMessage<uint32_t>>().setValue( static_cast<uint32_t>(value));
      break;
    case MessageTypes::UInt64:
      msg.as<ValueMessage<uint64_t>>().setValue( static_cast<uint64_t>(value));
      break;
    case MessageTypes::Int8:
      msg.as<ValueMessage<int8_t>>().setValue( static_cast<int8_t>(value));
      break;
    case MessageTypes::Int16:
      msg.as<ValueMessage<int16_t>>().setValue( static_cast<int16_t>(value));
      break;
    case MessageTypes::Int32:
      msg.as<ValueMessage<int32_t>>().setValue( static_cast<int32_t>(value));
      break;
    case MessageTypes::Int64:
      msg.as<ValueMessage<int64_t>>().setValue( static_cast<int64_t>(value));
      break;
    case MessageTypes::Float32:
      msg.as<ValueMessage<float>>().setValue( static_cast<float>(value));
      break;
    case MessageTypes::Float64:
      msg.as<ValueMessage<double>>().setValue( static_cast<double>(value));
      break;
//    case MessageTypes::String:
//      msg.as<ValueMessage<std::string>>().setValue( std::to_string( value ));
//      break;
//    case MessageTypes::Time:
//      msg.as<ValueMessage<ros::Time>>().setValue( ros::Time( static_cast<double>(value)));
//      break;
    case MessageTypes::Duration:
      // Durations in qml are given in milliseconds
      msg.as<ValueMessage<ros::Duration>>().setValue( ros::Duration( value / 1000.0 ));
      break;
    default:
      ROS_WARN( "Unknown type while filling message!" );
      return false;
  }
  return true;
}

template<>
bool fillValue( Message &msg, ros::Time value, uint32_t )
{
  if ( msg.type() != MessageTypes::Time )
  {
    ROS_WARN( "Tried to fill 'ros::Time' field with incompatible type!" );
    return false;
  }
  msg.as<ValueMessage<ros::Time>>().setValue( value );
  return true;
}

template<>
bool fillValue( Message &msg, std::string value, uint32_t )
{
  if ( msg.type() != MessageTypes::String )
  {
    ROS_WARN( "Tried to fill 'std::string' field with incompatible type!" );
    return false;
  }
  auto &value_msg = msg.as<ValueMessage<std::string>>();
  value_msg.setValue( std::move( value ));
  return true;
}

template<typename TBounds, typename TVal>
typename std::enable_if<std::is_signed<TVal>::value, bool>::type inBounds( TVal val )
{
  typedef typename std::make_signed<TBounds>::type STBounds;
  typedef typename std::make_unsigned<TBounds>::type UTBounds;
  typedef typename std::make_unsigned<TVal>::type UTVal;
  // val is in bounds of a signed target type if it is greater than the min of the signed type and either smaller than
  // zero or smaller than its max. If val is greater or equal to zero, the val and max are casted to unsigned types to
  // prevent compiler warnings if the TVal and TBounds are not both signed or both unsigned.
  return static_cast<STBounds>(std::numeric_limits<TBounds>::min()) <= val &&
         (val < 0 || static_cast<UTBounds>(val) <= static_cast<UTVal>(std::numeric_limits<TBounds>::max()));
}

template<typename TBounds, typename TVal>
typename std::enable_if<std::is_unsigned<TVal>::value, bool>::type inBounds( TVal val )
{
  return val <= static_cast<typename std::make_unsigned<TBounds>::type>(std::numeric_limits<TBounds>::max());
}

// Default implementation is for all integer types
template<typename T>
bool isCompatible( const QVariant &variant )
{
  switch ((int) variant.type())
  {
    case QMetaType::UChar:
    {
      auto val = variant.value<uint8_t>();
      return inBounds<T>( val );
    }
    case QMetaType::UShort:
    {
      auto val = variant.value<uint16_t>();
      return inBounds<T>( val );
    }
    case QVariant::UInt:
    {
      uint val = variant.toUInt();
      return inBounds<T>( val );
    }
    case QMetaType::ULong:
    {
      auto val = variant.value<unsigned long>();
      return inBounds<T>( val );
    }
    case QVariant::ULongLong:
    {
      qulonglong val = variant.toULongLong();
      return inBounds<T>( val );
    }
    case QMetaType::SChar:
    case QMetaType::Char:
    {
      auto val = variant.value<int8_t>();
      return inBounds<T>( val );
    }
    case QMetaType::Short:
    {
      auto val = variant.value<int16_t>();
      return inBounds<T>( val );
    }
    case QVariant::Int:
    {
      int val = variant.toInt();
      return inBounds<T>( val );
    }
    case QMetaType::Long:
    {
      auto val = variant.value<long>();
      return inBounds<T>( val );
    }
    case QVariant::LongLong:
    {
      qlonglong val = variant.toLongLong();
      return inBounds<T>( val );
    }
    case QMetaType::Float:
    {
      auto val = variant.value<float>();
      return val == std::round( val ) && std::numeric_limits<T>::min() <= val && val <= std::numeric_limits<T>::max();
    }
    case QVariant::Double:
    {
      double val = variant.toDouble();
      return val == std::round( val ) && std::numeric_limits<T>::min() <= val && val <= std::numeric_limits<T>::max();
    }
    default:
      break;
  }
  return false;
}

template<>
bool isCompatible<bool>( const QVariant &variant ) { return variant.type() == QVariant::Bool; }

template<>
bool isCompatible<float>( const QVariant &variant )
{
  return (int) variant.type() == QMetaType::Float || variant.type() == QVariant::Double ||
         variant.type() == QVariant::UInt || variant.type() == QVariant::Int || variant.type() == QVariant::ULongLong ||
         variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<double>( const QVariant &variant )
{
  return (int) variant.type() == QMetaType::Float || variant.type() == QVariant::Double ||
         variant.type() == QVariant::UInt || variant.type() == QVariant::Int || variant.type() == QVariant::ULongLong ||
         variant.type() == QVariant::LongLong;
}

template<>
bool isCompatible<ros::Time>( const QVariant &variant )
{
  return variant.type() == QVariant::Double || variant.type() == QVariant::UInt || variant.type() == QVariant::Int ||
         variant.type() == QVariant::ULongLong || variant.type() == QVariant::LongLong ||
         variant.type() == QVariant::DateTime;
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
  switch ((int) variant.type())
  {
    case QVariant::Bool:
      return variant.toBool();
    case QMetaType::SChar:
      return static_cast<T>(variant.value<int8_t>());
    case QMetaType::UChar:
      return static_cast<T>(variant.value<uint8_t>());
    case QMetaType::Short:
      return static_cast<T>(variant.value<int16_t>());
    case QMetaType::UShort:
      return static_cast<T>(variant.value<uint16_t>());
    case QVariant::Int:
      return static_cast<T>(variant.toInt());
    case QVariant::UInt:
      return static_cast<T>(variant.toUInt());
    case QMetaType::Long:
      return static_cast<T>(variant.value<long>());
    case QMetaType::ULong:
      return static_cast<T>(variant.value<unsigned long>());
    case QVariant::LongLong:
      return static_cast<T>(variant.toLongLong());
    case QVariant::ULongLong:
      return static_cast<T>(variant.toULongLong());
    case QMetaType::Float:
      return variant.value<float>();
    case QVariant::Double:
      return static_cast<T>(variant.toDouble());
    default:
      ROS_WARN_STREAM(
        "Tried to get " << typeid( T ).name() << " from incompatible type! Type: " << variant.typeName());
  }
  return T();
}

template<>
ros::Time getValue<ros::Time>( const QVariant &variant )
{
  switch ( variant.type())
  {
    case QVariant::Int:
      return qmlToRosTime( variant.toInt());
    case QVariant::UInt:
      return qmlToRosTime( variant.toUInt());
    case QVariant::LongLong:
      return qmlToRosTime( variant.toLongLong());
    case QVariant::ULongLong:
      return qmlToRosTime( variant.toULongLong());
    case QVariant::Double:
      return ros::Time( variant.toDouble() / 1000.0 );
    case QVariant::DateTime:
      return qmlToRosTime( variant.toDateTime().toMSecsSinceEpoch());
    case QVariant::Date:
    case QVariant::Time:
    default:
      ROS_WARN_STREAM( "Tried to get ros::Time from incompatible type! Type: " << variant.typeName());
      return {};
  }
}

template<>
ros::Duration getValue<ros::Duration>( const QVariant &variant )
{
  switch ( variant.type())
  {
    case QVariant::Int:
      return qmlToRosDuration( variant.toInt());
    case QVariant::UInt:
      return qmlToRosDuration( variant.toUInt());
    case QVariant::LongLong:
      return qmlToRosDuration( variant.toLongLong());
    case QVariant::ULongLong:
      return qmlToRosDuration( variant.toULongLong());
    case QVariant::Double:
      return qmlToRosDuration( variant.toDouble());
    case QVariant::Date:
    case QVariant::Time:
    case QVariant::DateTime:
    default:
      ROS_WARN_STREAM( "Tried to get ros::Duration from incompatible type! Type: " << variant.typeName());
      return {};
  }
}

template<typename T, typename ArrayType>
typename std::enable_if<!std::is_same<ArrayType, QAbstractListModel>::value, bool>::type
fillArray( Message &msg, const ArrayType &list )
{
  auto &array = msg.as<ArrayMessage<T>>();
  size_t count = list.size();
  bool no_error = true;
  if ( array.isFixedSize() && count > array.length())
  {
    count = array.length();
    no_error = false;
  }
  for ( size_t i = 0; i < count; ++i )
  {
    const QVariant &variant = list.at( i );
    if ( !isCompatible<T>( variant ))
    {
      ROS_WARN( "Tried to fill array of '%s' with incompatible value! Skipped. (Type: %s)", typeid( T ).name(),
                variant.typeName());
      no_error = false;
      continue;
    }
    if ( array.isFixedSize()) array.assign( i, getValue<T>( variant ));
    else array.push_back( getValue<T>( variant ));
  }
  return no_error;
}

template<typename T, typename ArrayType>
typename std::enable_if<std::is_same<ArrayType, QAbstractListModel>::value, bool>::type
fillArray( Message &msg, const ArrayType &list )
{
  auto &array = msg.as<ArrayMessage<T>>();
  size_t count = list.rowCount();
  bool no_error = true;
  if ( array.isFixedSize() && count > array.length())
  {
    count = array.length();
    no_error = false;
  }
  for ( size_t i = 0; i < count; ++i )
  {
    QModelIndex index = list.index( i, 0 );
    const QVariant &variant = list.data( index );
    if ( !isCompatible<T>( variant ))
    {
      ROS_WARN( "Tried to fill array of '%s' with incompatible value! Skipped. (Type: %s)", typeid( T ).name(),
                variant.typeName());
      no_error = false;
      continue;
    }
    if ( array.isFixedSize()) array.assign( i, getValue<T>( variant ));
    else array.push_back( getValue<T>( variant ));
  }
  return no_error;
}


template<typename T>
using limits = std::numeric_limits<T>;

uint32_t getCompatibleTypes( int val )
{
  uint32_t compatible_types = MessageTypes::Int32 | MessageTypes::Int64 |
                              MessageTypes::Float32 | MessageTypes::Float64 | MessageTypes::Duration;
  // If it fits, it sits
  if ( limits<int8_t>::min() <= val && val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
  if ( limits<uint8_t>::min() <= val && val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
  if ( limits<int16_t>::min() <= val && val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
  if ( limits<uint16_t>::min() <= val && val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
  if ( val >= 0 ) compatible_types |= MessageTypes::UInt32 | MessageTypes::UInt64;
  return compatible_types;
}

uint32_t getCompatibleTypes( uint val )
{
  uint32_t compatible_types = MessageTypes::UInt32 | MessageTypes::UInt64 | MessageTypes::Int64 |
                              MessageTypes::Float32 | MessageTypes::Float64 | MessageTypes::Duration;
  if ( val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
  if ( val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
  if ( val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
  if ( val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
  if ( val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
  return compatible_types;
}

uint32_t getCompatibleTypes( qlonglong val )
{
  uint32_t compatible_types =
    MessageTypes::Int64 | MessageTypes::Float32 | MessageTypes::Float64 | MessageTypes::Duration;
  if ( limits<int8_t>::min() <= val && val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
  if ( limits<uint8_t>::min() <= val && val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
  if ( limits<int16_t>::min() <= val && val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
  if ( limits<uint16_t>::min() <= val && val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
  if ( limits<int32_t>::min() <= val && val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
  if ( val >= 0 ) compatible_types |= MessageTypes::UInt32 | MessageTypes::UInt64;
  return compatible_types;
}

uint32_t getCompatibleTypes( qulonglong val )
{
  uint32_t compatible_types =
    MessageTypes::UInt64 | MessageTypes::Float32 | MessageTypes::Float64 | MessageTypes::Duration;
  if ( val <= limits<int8_t>::max()) compatible_types |= MessageTypes::Int8;
  if ( val <= limits<uint8_t>::max()) compatible_types |= MessageTypes::UInt8;
  if ( val <= limits<int16_t>::max()) compatible_types |= MessageTypes::Int16;
  if ( val <= limits<uint16_t>::max()) compatible_types |= MessageTypes::UInt16;
  if ( val <= limits<int32_t>::max()) compatible_types |= MessageTypes::Int32;
  if ( val <= limits<uint32_t>::max()) compatible_types |= MessageTypes::UInt32;
  if ( val <= limits<int64_t>::max()) compatible_types |= MessageTypes::Int64;
  return compatible_types;
}

uint32_t getCompatibleTypes( double val )
{
  uint32_t compatible_types = MessageTypes::Float32 | MessageTypes::Float64 | MessageTypes::Duration;
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
  return compatible_types;
}
}

bool fillMessage( ros_babel_fish::Message &msg, const QVariant &value )
{
  BabelFish fish = BabelFishDispenser::getBabelFish();
  return fillMessage( fish, msg, value );
}

template<typename Array>
bool fillArrayFromVariant( ros_babel_fish::Message &msg, const Array &list )
{
  if ( msg.type() != MessageTypes::Array )
  {
    ROS_WARN( "Tried to fill array in non-array message field!" );
    return false;
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
      return fillArray<bool>( array, list );
    case MessageTypes::UInt8:
      return fillArray<uint8_t>( array, list );
    case MessageTypes::UInt16:
      return fillArray<uint16_t>( array, list );
    case MessageTypes::UInt32:
      return fillArray<uint32_t>( array, list );
    case MessageTypes::UInt64:
      return fillArray<uint64_t>( array, list );
    case MessageTypes::Int8:
      return fillArray<int8_t>( array, list );
    case MessageTypes::Int16:
      return fillArray<int16_t>( array, list );
    case MessageTypes::Int32:
      return fillArray<int32_t>( array, list );
    case MessageTypes::Int64:
      return fillArray<int64_t>( array, list );
    case MessageTypes::Float32:
      return fillArray<float>( array, list );
    case MessageTypes::Float64:
      return fillArray<double>( array, list );
    case MessageTypes::Time:
      return fillArray<ros::Time>( array, list );
    case MessageTypes::Duration:
      return fillArray<ros::Duration>( array, list );
    case MessageTypes::String:
    {
      auto &array = msg.as<ArrayMessage<std::string>>();
      bool no_error = true;
      for ( size_t i = 0; i < count; ++i )
      {
        const QVariant &variant = list.at( i );
        if ( !variant.canConvert<QString>())
        {
          ROS_WARN( "Tried to fill string array with non-string value! Skipped." );
          no_error = false;
          continue;
        }
        if ( array.isFixedSize()) array.assign( i, variant.toString().toStdString());
        else array.push_back( variant.toString().toStdString());
      }
      return no_error;
    }
    case MessageTypes::Compound:
    {
      auto &array = msg.as<CompoundArrayMessage>();
      bool no_error = true;
      for ( size_t i = 0; i < count; ++i )
      {
        const QVariant &variant = list.at( i );
        if ( variant.type() != QVariant::Map )
        {
          ROS_WARN( "Tried to fill compound array with non-map value! Skipped." );
          no_error = false;
          continue;
        }
        auto &child = array.isFixedSize() ? array[i] : array.appendEmpty();
        fillMessage( child, variant );
      }
      return no_error;
    }
    case MessageTypes::Array:
      ROS_ERROR_ONCE( "Arrays of arrays are not supported!" );
      break;
  }
  return false;
}

template<>
bool fillArrayFromVariant( ros_babel_fish::Message &msg, const QAbstractListModel &list )
{
  if ( msg.type() != MessageTypes::Array )
  {
    ROS_WARN( "Invalid value passed to fillMessage!" );
    return false;
  }
  auto &array = msg.as<ArrayMessageBase>();
  size_t count = list.rowCount();
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
      return fillArray<bool>( array, list );
    case MessageTypes::UInt8:
      return fillArray<uint8_t>( array, list );
    case MessageTypes::UInt16:
      return fillArray<uint16_t>( array, list );
    case MessageTypes::UInt32:
      return fillArray<uint32_t>( array, list );
    case MessageTypes::UInt64:
      return fillArray<uint64_t>( array, list );
    case MessageTypes::Int8:
      return fillArray<int8_t>( array, list );
    case MessageTypes::Int16:
      return fillArray<int16_t>( array, list );
    case MessageTypes::Int32:
      return fillArray<int32_t>( array, list );
    case MessageTypes::Int64:
      return fillArray<int64_t>( array, list );
    case MessageTypes::Float32:
      return fillArray<float>( array, list );
    case MessageTypes::Float64:
      return fillArray<double>( array, list );
    case MessageTypes::Time:
      return fillArray<ros::Time>( array, list );
    case MessageTypes::Duration:
      return fillArray<ros::Duration>( array, list );
    case MessageTypes::String:
    {
      auto &array = msg.as<ArrayMessage<std::string>>();
      bool no_error = true;
      for ( size_t i = 0; i < count; ++i )
      {
        QModelIndex index = list.index( i, 0 );
        const QVariant &variant = list.data( index );
        if ( variant.type() != QVariant::String )
        {
          ROS_WARN( "Tried to fill string array with non-string value! Skipped." );
          no_error = false;
          continue;
        }
        if ( array.isFixedSize()) array.assign( i, variant.toString().toStdString());
        else array.push_back( variant.toString().toStdString());
      }
      return no_error;
    }
    case MessageTypes::Compound:
    {
      auto &array = msg.as<CompoundArrayMessage>();
      bool no_error = true;
      QHash<int, QByteArray> roleNames = list.roleNames();
      if ( roleNames.empty()) return true;
      std::vector<std::string> names;
      {
        int max_key = 0;
        for ( auto key : roleNames.keys()) max_key = std::max( max_key, key );
        names.resize( max_key + 1 );
      }
      // Collect keys
      QHashIterator<int, QByteArray> it( roleNames );
      while ( it.hasNext())
      {
        it.next();
        names[it.key()] = it.value().data();
      }
      // Check that all keys are in message
      const std::vector<std::string> &compound_names = array.elementTemplate()->compound.names;
      for ( auto &name : names )
      {
        const std::string &key = name;
        if ( key.empty()) continue;
        if ( std::find( compound_names.begin(), compound_names.end(), key ) == compound_names.end())
        {
          ROS_WARN_STREAM( "Message doesn't have field '" << key << "'!" );
          no_error = false;
          name = std::string();
        }
      }
      for ( size_t i = 0; i < count; ++i )
      {
        QModelIndex index = list.index( i );
        auto &child = array.isFixedSize() ? array[i] : array.appendEmpty();
        auto &compound = child.as<CompoundMessage>();
        for ( size_t j = 0; j < names.size(); ++j )
        {
          const std::string &key = names[j];
          if ( key.empty()) continue;
          no_error &= fillMessage( compound[key], index.data( j ));
        }
      }
      return no_error;
    }
    case MessageTypes::Array:
      ROS_ERROR_ONCE( "Arrays of arrays are not supported!" );
      break;
  }
  return false;
}

bool fillMessage( BabelFish &fish, ros_babel_fish::Message &msg, const QVariant &value )
{
  if ( value.canConvert<QVariantMap>() && msg.type() == MessageTypes::Compound )
  {
    auto &compound = msg.as<CompoundMessage>();
    const QVariantMap &map = value.value<QVariantMap>();
    bool no_error = true;
    for ( auto &key : map.keys())
    {
      std::string skey = key.toStdString();
      if ( !compound.containsKey( skey ))
      {
        ROS_WARN_STREAM( "Message doesn't have field '" << skey << "'!" );
        no_error = false;
        continue;
      }
      no_error &= fillMessage( fish, compound[skey], map[key] );
    }
    return no_error;
  }
  else if ( value.canConvert<Array>() && msg.type() == MessageTypes::Array )
  {
    return fillArrayFromVariant( msg, value.value<Array>());
  }
  else if ( value.canConvert<QVariantList>() && msg.type() == MessageTypes::Array )
  {
    const QVariantList &list = value.value<QVariantList>();
    return fillArrayFromVariant( msg, list );
  }
  else if ( value.canConvert<QObject *>())
  {
    const auto *list_model = value.value<QAbstractListModel *>();
    if ( list_model != nullptr )
    {
      return fillArrayFromVariant( msg, *list_model );
    }

    if ( msg.type() == MessageTypes::Compound )
    {
      const auto *obj = value.value<QObject *>();
      const QMetaObject *metaObj = obj->metaObject();
      auto &compound = msg.as<CompoundMessage>();
      bool no_error = true;
      for ( int i = metaObj->propertyOffset(); i < metaObj->propertyCount(); ++i )
      {
        QMetaProperty prop = metaObj->property( i );
        std::string skey = prop.name();
        // Skip keys that don't exist, we don't warn here because there will very likely be properties that don't exist.
        if ( !compound.containsKey( skey )) continue;
        no_error &= fillMessage( fish, compound[skey], prop.read( obj ));
      }
      return no_error;
    }
  }

  if ( msg.type() & (MessageTypes::Array | MessageTypes::Compound))
  {
    ROS_WARN_STREAM( "Invalid type for array/compound message: " << value.typeName() << " (" << value.type() << ")" );
    return false;
  }
  switch ((int) value.type())
  {

    case QVariant::Invalid:
      break;
    case QVariant::Bool:
      return fillValue<bool>( msg, value.toBool(),
                              MessageTypes::Bool ); // Since it is specifically a bool, we don't accept other types
    case QMetaType::Short:
    case QMetaType::SChar:
    case QVariant::Int:
    {
      int val = value.toInt();
      return fillValue<int>( msg, val, getCompatibleTypes( val ));
    }
    case QMetaType::UShort:
    case QMetaType::UChar:
    case QVariant::UInt:
    {
      uint val = value.toUInt();
      return fillValue<uint>( msg, val, getCompatibleTypes( val ));
    }
    case QMetaType::Long:
    case QVariant::LongLong:
    {
      qlonglong val = value.toLongLong();
      return fillValue<qlonglong>( msg, val, getCompatibleTypes( val ));
    }
    case QMetaType::ULong:
    case QVariant::ULongLong:
    {
      qulonglong val = value.toULongLong();
      return fillValue<qulonglong>( msg, val, getCompatibleTypes( val ));
    }
    case QMetaType::Float:
    case QVariant::Double:
    {
      double val = value.toDouble();
      return fillValue<double>( msg, value.toDouble(), getCompatibleTypes( val ));
    }
    case QVariant::String:
      return fillValue<std::string>( msg, value.toString().toStdString(), 0 /* ignored for this special case anyway */);
    case QVariant::DateTime:
      return fillValue<ros::Time>( msg, qmlToRosTime( value.toDateTime()), MessageTypes::Time );
    case QVariant::Date: // Not sure if any of these types should be supported
    case QVariant::Time:
    case QVariant::Char:
    default:
      ROS_WARN( "Unsupported QVariant type '%s' encountered while filling message!", value.typeName());
      break;
  }
  return false;
}
}
}
