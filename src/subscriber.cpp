// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/subscriber.h"

#include <chrono>
#include <include/qml_ros_plugin/subscriber.h>


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
void dumpTree( const RosIntrospection::StringTreeNode *node, const std::string &prefix = "" )
{
  std::cout << prefix << node->value() << std::endl;
  for ( const auto &child : node->children())
  {
    dumpTree( &child, prefix + "-" );
  }
}

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

template<typename T>
QVariant toQVariant( const T &value );

template<>
QVariant toQVariant<RosIntrospection::Variant>( const RosIntrospection::Variant &value )
{
  switch ( value.getTypeID())
  {
    case RosIntrospection::BOOL:
      return QVariant::fromValue( value.convert<uint8_t>() != 0 );
    case RosIntrospection::BYTE:
    case RosIntrospection::UINT8:
    case RosIntrospection::UINT16:
    case RosIntrospection::UINT32:
      return QVariant::fromValue( value.convert<unsigned int>());
    case RosIntrospection::UINT64:
      return QVariant::fromValue( value.convert<unsigned long>());
    case RosIntrospection::CHAR:
    case RosIntrospection::INT8:
    case RosIntrospection::INT16:
    case RosIntrospection::INT32:
      return QVariant::fromValue( value.convert<int>());
    case RosIntrospection::INT64:
      return QVariant::fromValue( value.convert<long>());
    case RosIntrospection::FLOAT32:
      return QVariant::fromValue( value.convert<float>());
    case RosIntrospection::FLOAT64:
      return QVariant::fromValue( value.convert<double>());
    case RosIntrospection::TIME:
      return QVariant::fromValue( value.convert<double>());
    case RosIntrospection::DURATION:
      return QVariant::fromValue( value.convert<double>());
    case RosIntrospection::STRING:
      return QVariant::fromValue( QString::fromStdString( value.convert<std::string>()));
  }
  return QVariant();
}

template<>
QVariant toQVariant<std::string>( const std::string &value )
{
  return QVariant::fromValue( QString::fromStdString( value ));
}

template<typename T>
QVariant &insertIntoTree( QVariant &map, const RosIntrospection::StringTreeLeaf &leaf,
                          const T &variant, const RosIntrospection::StringTreeNode *node, bool array, int &index )
{
  if ( node->parent() == nullptr ) return map;
  if ( node->value() == RosIntrospection::StringTreeLeaf::num_placeholder())
  {
    return insertIntoTree( map, leaf, variant, node->parent(), true, index );
  }
  auto &d = insertIntoTree( map, leaf, variant, node->parent(), false, index );
  QVariantMap &submap = *reinterpret_cast<QVariantMap *>(d.data());
  QString key = QString::fromStdString( node->value());
  if ( leaf.node_ptr == node )
  {
    if ( !node->isLeaf()) return map;
    submap.insert( key, toQVariant( variant ));
    return map;
  }
  if ( array )
  {
    auto it = submap.find( key );
    if ( it == submap.end())
    {
      it = submap.insert( key, QVariantList());
    }
    auto &d = it.value();
    QVariantList &list = *reinterpret_cast<QVariantList *>(d.data());
    for ( int i = list.size(); i <= leaf.index_array[index]; ++i )
    {
      list.push_back( QVariantMap());
    }
    return list[leaf.index_array[index++]];
  }
  auto it = submap.find( key );
  if ( it == submap.end())
  {
    it = submap.insert( key, QVariantMap());
  }
  return it.value();
}
}

void Subscriber::messageCallback( const topic_tools::ShapeShifter::ConstPtr &msg )
{
  const std::string &datatype = msg->getDataType();
  const std::string &definition = msg->getMessageDefinition();

  parser_.registerMessageDefinition( topic_.toStdString(), RosIntrospection::ROSType( datatype ), definition );
  buffer_.resize( msg->size());
  ros::serialization::OStream stream( buffer_.data(), buffer_.size());
  msg->write( stream );

  parser_.deserializeIntoFlatContainer( topic_.toStdString(), absl::Span<uint8_t>( buffer_ ), &flat_message_, 10000 );

  message_ = QVariant::fromValue( QVariantMap());
  for ( auto &kv : flat_message_.value )
  {
    int index = 0;
    insertIntoTree( message_, kv.first, kv.second, kv.first.node_ptr, false, index );
  }
  for ( auto &kv : flat_message_.name )
  {
    int index = 0;
    insertIntoTree( message_, kv.first, kv.second, kv.first.node_ptr, false, index );
  }
  emit messageChanged();
  emit newMessage( message_ );
}
}
