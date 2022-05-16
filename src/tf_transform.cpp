// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/tf_transform.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/tf_transform_listener.h"

#include <geometry_msgs/TransformStamped.h>

using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{

TfTransform::TfTransform() : throttle_time_( 1000 / 60 ), enabled_( true )
{
  geometry_msgs::TransformStamped transform;
  message_ = msgToMap( transform );
  message_.insert( "valid", false );
  connect( &throttle_timer_, &QTimer::timeout, this, &TfTransform::updateMessage );
  throttle_timer_.setSingleShot( true );
}

TfTransform::~TfTransform() { shutdown(); }

const QString &TfTransform::sourceFrame() const { return source_frame_; }

void TfTransform::setSourceFrame( const QString &value )
{
  source_frame_ = value;
  if ( source_frame_.isEmpty() )
    shutdown();
  else
    subscribe();

  emit sourceFrameChanged();
}

const QString &TfTransform::targetFrame() const { return target_frame_; }

void TfTransform::setTargetFrame( const QString &targetFrame )
{
  target_frame_ = targetFrame;
  if ( target_frame_.isEmpty() )
    shutdown();
  else
    subscribe();

  emit targetFrameChanged();
}

bool TfTransform::enabled() const { return enabled_; }

void TfTransform::setEnabled( bool value )
{
  if ( enabled_ == value )
    return;
  enabled_ = value;
  if ( enabled_ )
    subscribe();
  else
    shutdown();

  emit enabledChanged();
}

qreal TfTransform::rate() const
{
  if ( throttle_time_.count() == 0 )
    return 0;
  return 1000.0 / throttle_time_.count();
}

void TfTransform::setRate( qreal value )
{
  if ( value <= 0 )
    throttle_time_ = std::chrono::milliseconds::zero();
  throttle_time_ = std::chrono::milliseconds(
      int( 1000 / value ) ); // Rounding down as value is strictly positive

  emit rateChanged();
}

const QVariantMap &TfTransform::message() { return message_; }

const QVariant &TfTransform::translation()
{
  const QVariantMap &transform = *static_cast<const QVariantMap *>( message_["transform"].data() );
  return transform.find( "translation" ).value();
}

const QVariant &TfTransform::rotation()
{
  const QVariantMap &transform = *static_cast<const QVariantMap *>( message_["transform"].data() );
  return transform.find( "rotation" ).value();
}

bool TfTransform::valid() { return message_.contains( "valid" ) && message_["valid"].toBool(); }

void TfTransform::onTransformChanged()
{
  if ( !enabled_ )
    return;
  std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
  if ( !throttle_timer_.isActive() ) {
    std::chrono::milliseconds delta_t =
        std::chrono::duration_cast<std::chrono::milliseconds>( now - last_transform_ );
    if ( delta_t >= throttle_time_ )
      updateMessage();
    else
      throttle_timer_.start( throttle_time_ - delta_t );
  }
  last_transform_ = now;
}

void TfTransform::subscribe()
{
  if ( source_frame_.isEmpty() || target_frame_.isEmpty() || !enabled_ )
    return;

  TfTransformListener::getInstance().registerWrapper();
  QObject::connect( &TfTransformListener::getInstance(), &TfTransformListener::transformChanged,
                    this, &TfTransform::onTransformChanged );
  // Load transform
  if ( valid() )
    onTransformChanged();
}

void TfTransform::shutdown()
{
  QObject::disconnect( &TfTransformListener::getInstance(), &TfTransformListener::transformChanged,
                       this, &TfTransform::onTransformChanged );
  TfTransformListener::getInstance().unregisterWrapper();
}

void TfTransform::updateMessage()
{
  bool was_valid = valid();
  message_ = TfTransformListener::getInstance().lookUpTransform( target_frame_, source_frame_ );
  if ( valid() != was_valid )
    emit validChanged();
  emit rotationChanged();
  emit messageChanged();
  emit translationChanged();
}
} // namespace qml_ros_plugin
