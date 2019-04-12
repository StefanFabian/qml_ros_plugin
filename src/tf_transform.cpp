// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/tf_transform.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/tf_transform_listener.h"

#include <geometry_msgs/TransformStamped.h>

namespace qml_ros_plugin
{

TfTransform::TfTransform() : active_( true )
{
  geometry_msgs::TransformStamped transform;
  message_ = msgToMap( transform );
  message_.insert( "valid", false );
}

TfTransform::~TfTransform()
{
  shutdown();
}

const QString &TfTransform::sourceFrame() const { return source_frame_; }

void TfTransform::setSourceFrame( const QString &value )
{
  source_frame_ = value;
  if ( source_frame_.isEmpty()) shutdown();
  else subscribe();
  emit sourceFrameChanged();
}

const QString &TfTransform::targetFrame() const { return target_frame_; }

void TfTransform::setTargetFrame( const QString &targetFrame )
{
  target_frame_ = targetFrame;
  if ( target_frame_.isEmpty()) shutdown();
  else subscribe();
  emit targetFrameChanged();
}

bool TfTransform::active() const { return active_; }

void TfTransform::setActive( bool value )
{
  if ( active_ == value ) return;
  active_ = value;
  if ( active_ ) subscribe();
  else shutdown();
  emit activeChanged();
}

const QVariant TfTransform::message() const { return message_; }

const QVariant &TfTransform::translation() const
{
  const QVariantMap &transform = *static_cast<const QVariantMap *>(message_["transform"].data());
  return transform.find( "translation" ).value();
}

const QVariant &TfTransform::rotation() const
{
  const QVariantMap &transform = *static_cast<const QVariantMap *>(message_["transform"].data());
  return transform.find( "rotation" ).value();
}

void TfTransform::onTransformChanged()
{
  message_ = TfTransformListener::getInstance().lookUpTransform( target_frame_, source_frame_ );
  emit messageChanged();
  emit translationChanged();
  emit rotationChanged();
}

void TfTransform::subscribe()
{
  if ( source_frame_.isEmpty() || target_frame_.isEmpty() || !active_ ) return;

  QObject::connect( &TfTransformListener::getInstance(), &TfTransformListener::transformChanged,
                    this, &TfTransform::onTransformChanged );
  // Load transform
  onTransformChanged();
}

void TfTransform::shutdown()
{
  QObject::disconnect( &TfTransformListener::getInstance(), &TfTransformListener::transformChanged,
                       this, &TfTransform::onTransformChanged );
}
}
