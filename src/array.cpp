// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/qml_ros_conversion.h"
#include "qml_ros_plugin/time.h"

using namespace qml_ros_plugin::conversion;
using namespace ros_babel_fish;

namespace qml_ros_plugin
{

Array::Array() { p_ = std::make_shared<Data>(); }

Array::Array( ros_babel_fish::TranslatedMessage::ConstPtr translated_message,
              const ros_babel_fish::ArrayMessageBase *message )
{
  p_ = std::make_shared<Data>();
  p_->translated_message = std::move( translated_message );
  p_->message = message;
  p_->all_in_cache = false;
  p_->length = message == nullptr ? 0 : int( message->length() );
}

int Array::length() const { return p_->length; }

void Array::setLength( int value )
{
  p_->length = value;
  for ( int i = p_->modified.length(); i > p_->length; --i ) p_->modified.pop_back();
  for ( int i = p_->cache.length(); i > p_->length; --i ) p_->cache.pop_back();
}

namespace
{
template<typename T>
QVariant getElement( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( T() );
  return QVariant::fromValue( array->as<ArrayMessage<T>>()[index] );
}

template<>
QVariant getElement<ros::Duration>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( 0.0 );
  return QVariant::fromValue( rosToQmlDuration( array->as<ArrayMessage<ros::Duration>>()[index] ) );
}

template<>
QVariant getElement<ros::Time>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( QDateTime::fromMSecsSinceEpoch( 0 ) );
  return QVariant::fromValue( Time( array->as<ArrayMessage<ros::Time>>()[index] ) );
}

template<>
QVariant getElement<std::string>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( QString() );
  return QVariant::fromValue(
      QString::fromStdString( array->as<ArrayMessage<std::string>>()[index] ) );
}

template<>
QVariant getElement<uint8_t>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( 0U );
  return QVariant( uint( array->as<ArrayMessage<uint8_t>>()[index] ) );
}

template<>
QVariant getElement<int8_t>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>( index ) >= array->length() )
    return QVariant::fromValue( 0U );
  return QVariant( int( array->as<ArrayMessage<int8_t>>()[index] ) );
}
} // namespace

QVariant Array::at( int index ) const
{
  if ( index < 0 || index >= length() ) {
    return QVariant();
  }
  if ( p_->cache.size() > index && p_->cache[index].isValid() )
    return p_->cache[index];
  switch ( p_->message->elementType() ) {
  case MessageTypes::None:
    return QVariant();
  case MessageTypes::Bool:
    return getElement<bool>( p_->message, index );
  case MessageTypes::UInt8:
    return getElement<uint8_t>( p_->message, index );
  case MessageTypes::UInt16:
    return getElement<uint16_t>( p_->message, index );
  case MessageTypes::UInt32:
    return getElement<uint32_t>( p_->message, index );
  case MessageTypes::UInt64:
    return getElement<uint64_t>( p_->message, index );
  case MessageTypes::Int8:
    return getElement<int8_t>( p_->message, index );
  case MessageTypes::Int16:
    return getElement<int16_t>( p_->message, index );
  case MessageTypes::Int32:
    return getElement<int32_t>( p_->message, index );
  case MessageTypes::Int64:
    return getElement<int64_t>( p_->message, index );
  case MessageTypes::Float32:
    return getElement<float>( p_->message, index );
  case MessageTypes::Float64:
    return getElement<double>( p_->message, index );
  case MessageTypes::Time:
    return getElement<ros::Time>( p_->message, index );
  case MessageTypes::Duration:
    return getElement<ros::Duration>( p_->message, index );
  case MessageTypes::String:
    return getElement<std::string>( p_->message, index );
  default:
    break;
  }
  if ( p_->cache.size() <= index ) {
    p_->cache.reserve( index + 1 );
    for ( int i = p_->cache.size(); i <= index; ++i ) { p_->cache.push_back( QVariant() ); }
  }
  if ( !p_->cache[index].isValid() ) {
    if ( static_cast<size_t>( index ) < p_->message->length() )
      p_->cache[index] =
          msgToMap( p_->translated_message, p_->message->as<ArrayMessage<Message>>()[index] );
    else
      p_->cache[index] = QVariantMap();
  }
  return p_->cache[index];
}

void Array::spliceList( int start, int delete_count, const QVariantList &items )
{
  if ( start > p_->length )
    start = p_->length;
  else if ( start < 0 )
    start = p_->length + start;
  if ( start < 0 )
    start = 0;
  if ( start + delete_count >= length() ) {
    // cheap case where we can just remove the last elements and add the new items.
    if ( !p_->all_in_cache ) {
      enlargeCache( length() );
      for ( int i = p_->modified.length(); i > start; --i ) p_->modified.pop_back();
    }
    for ( int i = p_->cache.length(); i > start; --i ) p_->cache.pop_back();
    for ( auto &item : items ) {
      if ( !p_->all_in_cache )
        p_->modified.push_back( true );
      p_->cache.push_back( item );
    }
    p_->length = p_->cache.size();
    return;
  }
  if ( delete_count == 1 && items.size() == 1 ) {
    // this is a simple replace operation
    if ( !p_->all_in_cache ) {
      enlargeCache( start + 1 );
      p_->modified[start] = true;
    }
    p_->cache[start] = items[0];
    return;
  }
  // otherwise we have to copy the entire message
  fillCache();
  for ( int i = 0; i < delete_count; ++i ) p_->cache.removeAt( start );
  for ( int i = 0; i < items.size(); ++i ) p_->cache.insert( start + i, items[i] );
  p_->length = p_->cache.size();
}

void Array::push( const QVariant &value )
{
  enlargeCache( length() );
  p_->cache.append( value );
  if ( !p_->all_in_cache )
    p_->modified.push_back( true );
  ++p_->length;
}

void Array::unshift( const QVariant &value )
{
  fillCache();
  p_->cache.prepend( value );
  ++p_->length;
}

QVariant Array::pop()
{
  if ( length() == 0 )
    return QVariant();
  QVariant result = at( length() - 1 ); // This automatically grows the cache if not a primitive type
  if ( p_->cache.size() == length() )
    p_->cache.pop_back();
  if ( !p_->all_in_cache ) {
    if ( p_->modified.size() == p_->length )
      p_->modified.pop_back();
  }
  --p_->length;
  return result;
}

QVariant Array::shift()
{
  if ( length() == 0 )
    return QVariant();
  fillCache();
  QVariant result = at( 0 );
  p_->cache.pop_front();
  --p_->length;
  return result;
}

QVariantList Array::toArray()
{
  fillCache();
  return p_->cache;
}

bool Array::_isModified( int index ) const
{
  return p_->all_in_cache || ( index < p_->modified.size() && p_->modified[index] );
}

const ros_babel_fish::ArrayMessageBase *Array::_message() const { return p_->message; }

QVariantList Array::toVariantList() const
{
  fillCache();
  return p_->cache;
}

void Array::enlargeCache( int size ) const
{
  if ( p_->cache.size() >= size )
    return;
  p_->cache.reserve( size );
  for ( int i = p_->cache.size(); i < size; ++i ) { p_->cache.push_back( QVariant() ); }
  for ( size_t i = p_->modified.size(); i < static_cast<uint>( size ); ++i ) {
    p_->modified.push_back( false );
  }
}

bool Array::_inCache() const { return p_->all_in_cache; }

void Array::fillCache() const
{
  if ( p_->all_in_cache )
    return;
  p_->cache.reserve( length() );
  for ( int i = 0; i < p_->length; ++i ) {
    if ( ( p_->modified.size() > i && p_->modified[i] ) ||
         ( p_->cache.size() > i && p_->cache[i].isValid() ) )
      continue;
    const QVariant &variant = at( i );
    if ( p_->cache.size() <= i )
      p_->cache.push_back( variant );
    else
      p_->cache[i] = variant;
  }
  p_->all_in_cache = true;
  p_->modified.clear();
}
} // namespace qml_ros_plugin
