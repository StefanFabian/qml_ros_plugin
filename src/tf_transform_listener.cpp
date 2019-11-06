// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/tf_transform_listener.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

#include <QVariantMap>
#include <tf2_ros/transform_listener.h>

using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{

struct TfTransformListener::State
{
  State() : buffer(), listener( buffer ) { }

  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener;
};

TfTransformListener &TfTransformListener::getInstance()
{
  static TfTransformListener instance;
  return instance;
}

TfTransformListener::TfTransformListener()
{
  state_.reset();
}

TfTransformListener::~TfTransformListener() = default;

void TfTransformListener::onRosInitialized()
{
  state_.reset( new State());
  state_->buffer._addTransformsChangedListener( boost::bind( &TfTransformListener::onTransformChanged, this ));
}

void TfTransformListener::onRosShutdown()
{
  state_.reset();
}

void TfTransformListener::onTransformChanged()
{
  emit transformChanged();
}

QVariant TfTransformListener::canTransform( const QString &target_frame, const QString &source_frame,
                                            const QDateTime &time, double timeout ) const
{
  if ( !isInitialized()) return QString( "Uninitialized" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 )
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(), qmlToRosTime( time ),
                                          &error );
  }
  else
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(), qmlToRosTime( time ),
                                          qmlToRosDuration( timeout ), &error );
  }
  if ( result ) return true;
  if ( error.empty()) return false;
  return QString::fromStdString( error );
}

QVariant TfTransformListener::canTransform( const QString &target_frame, const QDateTime &target_time,
                                            const QString &source_frame, const QDateTime &source_time,
                                            const QString &fixed_frame, double timeout ) const
{
  if ( !isInitialized()) return QString( "Uninitialized" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 )
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), qmlToRosTime( target_time ),
                                          source_frame.toStdString(), qmlToRosTime( source_time ),
                                          fixed_frame.toStdString(), &error );
  }
  else
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), qmlToRosTime( target_time ),
                                          source_frame.toStdString(), qmlToRosTime( source_time ),
                                          fixed_frame.toStdString(), qmlToRosDuration( timeout ), &error );
  }
  if ( result ) return true;
  if ( error.empty()) return false;
  return QString::fromStdString( error );
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                                  const QDateTime &time, double timeout )
{
  geometry_msgs::TransformStamped transform;
  if ( !isInitialized())
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  try
  {
    if ( timeout <= 1E-6 )
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                                  qmlToRosTime( time ));
    }
    else
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                                  qmlToRosTime( time ), qmlToRosDuration( timeout ));
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  }
  catch ( tf2::LookupException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::ConnectivityException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::ExtrapolationException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::InvalidArgumentException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame, const QDateTime &target_time,
                                                  const QString &source_frame, const QDateTime &source_time,
                                                  const QString &fixed_frame, double timeout )
{
  geometry_msgs::TransformStamped transform;
  if ( !isInitialized())
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  try
  {
    if ( timeout <= 0.0000001 )
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), qmlToRosTime( target_time ),
                                                  source_frame.toStdString(), qmlToRosTime( source_time ),
                                                  fixed_frame.toStdString());
    }
    else
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), qmlToRosTime( target_time ),
                                                  source_frame.toStdString(), qmlToRosTime( source_time ),
                                                  fixed_frame.toStdString(), qmlToRosDuration( timeout ));
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  }
  catch ( tf2::LookupException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::ConnectivityException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::ExtrapolationException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
  catch ( tf2::InvalidArgumentException &ex )
  {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what()));
    return result;
  }
}

TfTransformListenerWrapper::TfTransformListenerWrapper()
{
  QObject::connect( &TfTransformListener::getInstance(), &TfTransformListener::transformChanged,
                    this, &TfTransformListenerWrapper::transformChanged );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                                         const QDateTime &time, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame, time, timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame, const QDateTime &target_time,
                                                         const QString &source_frame, const QDateTime &source_time,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, target_time, source_frame,
                                                             source_time, fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame, const QString &source_frame,
                                                   const QDateTime &time, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame, time, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame, const QDateTime &target_time,
                                                   const QString &source_frame, const QDateTime &source_time,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, target_time, source_frame, source_time,
                                                          fixed_frame, timeout );
}
}
