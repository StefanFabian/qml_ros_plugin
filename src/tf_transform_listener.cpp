// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/tf_transform_listener.h"
#include "qml_ros_plugin/message_conversions.h"

#include <QVariantMap>
#include <tf2_ros/transform_listener.h>

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
  state_ = std::make_shared<State>();
  state_->buffer._addTransformsChangedListener( boost::bind( &TfTransformListener::onTransformChanged, this ));
}

TfTransformListener::~TfTransformListener() = default;

void TfTransformListener::onTransformChanged()
{
  emit transformChanged();
}

QVariant TfTransformListener::canTransform( const QString &target_frame, const QString &source_frame, double time_sec,
                                            double timeout ) const
{
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 )
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(), ros::Time( time_sec ),
                                          &error );
  }
  else
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), source_frame.toStdString(), ros::Time( time_sec ),
                                          ros::Duration( timeout ), &error );
  }
  if ( result ) return true;
  if ( error.empty()) return false;
  return QString::fromStdString( error );
}

QVariant TfTransformListener::canTransform( const QString &target_frame, double target_time_sec,
                                            const QString &source_frame, double source_time_sec,
                                            const QString &fixed_frame, double timeout ) const
{
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 )
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), ros::Time( target_time_sec ),
                                          source_frame.toStdString(), ros::Time( source_time_sec ),
                                          fixed_frame.toStdString(),
                                          &error );
  }
  else
  {
    result = state_->buffer.canTransform( target_frame.toStdString(), ros::Time( target_time_sec ),
                                          source_frame.toStdString(), ros::Time( source_time_sec ),
                                          fixed_frame.toStdString(),
                                          ros::Duration( timeout ), &error );
  }
  if ( result ) return true;
  if ( error.empty()) return false;
  return QString::fromStdString( error );
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                                  double time_sec, double timeout )
{
  geometry_msgs::TransformStamped transform;
  try
  {
    if ( timeout <= 0.0000001 )
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                                  ros::Time( time_sec ));
    }
    else
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                                  ros::Time( time_sec ), ros::Duration( timeout ));
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

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame, double target_time_sec,
                                                  const QString &source_frame, double source_time_sec,
                                                  const QString &fixed_frame, double timeout )
{
  geometry_msgs::TransformStamped transform;
  try
  {
    if ( timeout <= 0.0000001 )
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), ros::Time( target_time_sec ),
                                                  source_frame.toStdString(), ros::Time( source_time_sec ),
                                                  fixed_frame.toStdString());
    }
    else
    {
      transform = state_->buffer.lookupTransform( target_frame.toStdString(), ros::Time( target_time_sec ),
                                                  source_frame.toStdString(), ros::Time( source_time_sec ),
                                                  fixed_frame.toStdString(), ros::Duration( timeout ));
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
                                                         double time_sec, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame, time_sec, timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame, double target_time_sec,
                                                         const QString &source_frame, double source_time_sec,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, target_time_sec, source_frame,
                                                             source_time_sec, fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame, const QString &source_frame,
                                                   double time_sec, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame, double target_time_sec,
                                                   const QString &source_frame, double source_time_sec,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, target_time_sec,
                                                          source_frame, source_time_sec,
                                                          fixed_frame, timeout );
}
}
