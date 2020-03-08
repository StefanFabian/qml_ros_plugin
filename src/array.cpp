// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

using namespace qml_ros_plugin::conversion;
using namespace ros_babel_fish;

namespace qml_ros_plugin
{

Array::Array( ros_babel_fish::TranslatedMessage::ConstPtr translated_message,
              const ros_babel_fish::ArrayMessageBase *message )
  : translated_message_( std::move( translated_message )), message_( message )
    , all_in_cache_( false ), length_( message == nullptr ? 0 : int( message->length())) { }

int Array::length() const
{
  return length_;
}

void Array::setLength( int value )
{
  length_ = value;
  for ( int i = modified_.length(); i > length_; --i )
    modified_.pop_back();
  for ( int i = cache_.length(); i > length_; --i )
    cache_.pop_back();
  emit lengthChanged();
}

namespace
{
template<typename T>
QVariant getElement( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>(index) >= array->length())
    return QVariant::fromValue( T());
  return QVariant::fromValue( array->as<ArrayMessage<T>>()[index] );
}

template<>
QVariant getElement<ros::Duration>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>(index) >= array->length())
    return QVariant::fromValue( 0.0 );
  return QVariant::fromValue( rosToQmlDuration( array->as<ArrayMessage<ros::Duration>>()[index] ));
}

template<>
QVariant getElement<ros::Time>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>(index) >= array->length())
    return QVariant::fromValue( QDateTime::fromMSecsSinceEpoch( 0 ));
  return QVariant::fromValue( rosToQmlTime( array->as<ArrayMessage<ros::Time>>()[index] ));
}

template<>
QVariant getElement<std::string>( const ArrayMessageBase *array, int index )
{
  if ( static_cast<size_t>(index) >= array->length())
    return QVariant::fromValue( QString());
  return QVariant::fromValue( QString::fromStdString( array->as<ArrayMessage<std::string>>()[index] ));
}
}

QVariant Array::at( int index ) const
{
  if ( index < 0 || index >= length())
  {
    return QVariant();
  }
  if ( cache_.size() > index && cache_[index].isValid())
    return cache_[index];
  switch ( message_->elementType())
  {
    case MessageTypes::None:
      return QVariant();
    case MessageTypes::Bool:
      return getElement<bool>( message_, index );
    case MessageTypes::UInt8:
      return getElement<uint8_t>( message_, index );
    case MessageTypes::UInt16:
      return getElement<uint16_t>( message_, index );
    case MessageTypes::UInt32:
      return getElement<uint32_t>( message_, index );
    case MessageTypes::UInt64:
      return getElement<uint64_t>( message_, index );
    case MessageTypes::Int8:
      return getElement<int8_t>( message_, index );
    case MessageTypes::Int16:
      return getElement<int16_t>( message_, index );
    case MessageTypes::Int32:
      return getElement<int32_t>( message_, index );
    case MessageTypes::Int64:
      return getElement<int64_t>( message_, index );
    case MessageTypes::Float32:
      return getElement<float>( message_, index );
    case MessageTypes::Float64:
      return getElement<double>( message_, index );
    case MessageTypes::Time:
      return getElement<ros::Time>( message_, index );
    case MessageTypes::Duration:
      return getElement<ros::Duration>( message_, index );
    case MessageTypes::String:
      return getElement<std::string>( message_, index );
    default:
      break;
  }
  if ( cache_.size() <= index )
  {
    cache_.reserve( index + 1 );
    for ( int i = cache_.size(); i <= index; ++i )
    {
      cache_.push_back( QVariant());
    }
  }
  if ( !cache_[index].isValid())
  {
    if ( static_cast<size_t>(index) < message_->length())
      cache_[index] = msgToMap( translated_message_, message_->as<ArrayMessage<Message>>()[index] );
    else
      cache_[index] = QVariantMap();
  }
  return cache_[index];
}

void Array::spliceList( int start, int delete_count, const QVariantList &items )
{
  int old_length = length_;
  if ( start > length_ ) start = length_;
  else if ( start < 0 ) start = length_ + start;
  if ( start < 0 ) start = 0;
  if ( start + delete_count >= length())
  {
    // cheap case where we can just remove the last elements and add the new items.
    if ( !all_in_cache_ )
    {
      enlargeCache( length());
      for ( int i = modified_.length(); i > start; --i )
        modified_.pop_back();
    }
    for ( int i = cache_.length(); i > start; --i )
      cache_.pop_back();
    for ( auto &item : items )
    {
      if ( !all_in_cache_ ) modified_.push_back( true );
      cache_.push_back( item );
    }
    length_ = cache_.size();
    if ( length_ != old_length )
      emit lengthChanged();
    return;
  }
  if ( delete_count == 1 && items.size() == 1 )
  {
    // this is a simple replace operation
    if ( !all_in_cache_ )
    {
      enlargeCache( start + 1 );
      modified_[start] = true;
    }
    cache_[start] = items[0];
    return;
  }
  // otherwise we have to copy the entire message
  fillCache();
  for ( int i = 0; i < delete_count; ++i ) cache_.removeAt( start );
  for ( int i = 0; i < items.size(); ++i ) cache_.insert( start + i, items[i] );
  length_ = cache_.size();
  if ( length_ != old_length )
    emit lengthChanged();
}

void Array::push( const QVariant &value )
{
  enlargeCache( length());
  cache_.append( value );
  if ( !all_in_cache_ ) modified_.push_back( true );
  ++length_;
  emit lengthChanged();
}

void Array::unshift( const QVariant &value )
{
  fillCache();
  cache_.prepend( value );
  ++length_;
  emit lengthChanged();
}

QVariant Array::pop()
{
  if ( length() == 0 ) return QVariant();
  QVariant result = at( length() - 1 ); // This automatically grows the cache if not a primitive type
  if ( cache_.size() == length())
    cache_.pop_back();
  if ( !all_in_cache_ )
  {
    if ( modified_.size() == length_ )
      modified_.pop_back();
  }
  --length_;
  emit lengthChanged();
  return result;
}

QVariant Array::shift()
{
  if ( length() == 0 ) return QVariant();
  fillCache();
  QVariant result = at( 0 );
  cache_.pop_front();
  --length_;
  emit lengthChanged();
  return result;
}

bool Array::_isModified( int index ) const
{
  return all_in_cache_ || (index < modified_.size() && modified_[index]);
}

const ros_babel_fish::ArrayMessageBase *Array::_message() const
{
  return message_;
}

void Array::enlargeCache( int size )
{
  if ( cache_.size() >= size ) return;
  cache_.reserve( size );
  for ( int i = cache_.size(); i < size; ++i )
  {
    cache_.push_back( QVariant());
  }
  for ( size_t i = modified_.size(); i < static_cast<uint>(size); ++i )
  {
    modified_.push_back( false );
  }
}

bool Array::_inCache() const
{
  return all_in_cache_;
}

void Array::fillCache()
{
  if ( all_in_cache_ ) return;
  cache_.reserve( length());
  for ( int i = 0; i < length_; ++i )
  {
    if ((modified_.size() > i && modified_[i]) || (cache_.size() > i && cache_[i].isValid())) continue;
    const QVariant &variant = at( i );
    if ( cache_.size() <= i )
      cache_.push_back( variant );
    else
      cache_[i] = variant;
  }
  all_in_cache_ = true;
  modified_.clear();
}
} // qml_ros_plugin
