// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/image_transport_subscriber.h"
#include "qml_ros_plugin/image_buffer.h"
#include "qml_ros_plugin/image_transport_manager.h"
#include "qml_ros_plugin/ros.h"

namespace qml_ros_plugin
{

ImageTransportSubscriber::ImageTransportSubscriber( NodeHandle::Ptr nh, QString topic, quint32 queue_size )
  : topic_( std::move( topic )), default_transport_( "compressed" ), nh_( std::move( nh )), queue_size_( queue_size )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::onNoImageTimeout, Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
  initSubscriber();
}

ImageTransportSubscriber::ImageTransportSubscriber()
  : default_transport_( "compressed" ), nh_( std::make_shared<NodeHandle>()), queue_size_( 1 )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::onNoImageTimeout, Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
}

QAbstractVideoSurface *ImageTransportSubscriber::videoSurface() const { return surface_; }

void ImageTransportSubscriber::setVideoSurface( QAbstractVideoSurface *surface )
{
  if ( surface == surface_ ) return;

  bool subscribed = subscribed_;
  blockSignals( true );
  shutdownSubscriber();
  surface_ = surface;
  initSubscriber();
  blockSignals( false );
  if ( subscribed != subscribed_ ) emit subscribedChanged();
}

void ImageTransportSubscriber::onNodeHandleReady()
{
  initSubscriber();
}

void ImageTransportSubscriber::onRosShutdown()
{
  shutdownSubscriber();
}

void ImageTransportSubscriber::initSubscriber()
{
  // This makes sure we lazy subscribe and only subscribe if there is a surface to write to
  if ( surface_ == nullptr ) return;
  if ( !nh_->isReady()) return;
  if ( topic_.isEmpty()) return;
  bool was_subscribed = subscribed_;
  if ( subscribed_ )
  {
    blockSignals( true );
    shutdownSubscriber();
    blockSignals( false );
  }
  // TODO Transport hints
  image_transport::TransportHints transport_hints( default_transport_.toStdString());
  subscription_ = ImageTransportManager::getInstance().subscribe( nh_, topic_, queue_size_, transport_hints,
                                                                  std::bind( &ImageTransportSubscriber::presentFrame,
                                                                             this, std::placeholders::_1 ),
                                                                  surface_, throttle_interval_ );
  subscribed_ = subscription_ != nullptr;
  if ( !was_subscribed ) emit subscribedChanged();
}

void ImageTransportSubscriber::shutdownSubscriber()
{
  if ( !subscribed_ ) return;
  subscription_.reset();
  if ( surface_ != nullptr && surface_->isActive())
    surface_->stop();
  subscribed_ = false;
  emit subscribedChanged();
}

void ImageTransportSubscriber::onNoImageTimeout()
{
  if ( !surface_->isActive()) return;
  int elapsed_time_milliseconds = static_cast<int>((ros::Time::now() - last_frame_timestamp_).toNSec() / 1000000);

  if ( timeout_ == 0 ) return;
  if ( elapsed_time_milliseconds < timeout_ )
  {
    no_image_timer_.start( timeout_ - elapsed_time_milliseconds );
    return;
  }
  surface_->present( QVideoFrame());
}

namespace
{
const char *videoSurfaceErrorToString( QAbstractVideoSurface::Error error )
{
  switch ( error )
  {
    case QAbstractVideoSurface::NoError:
      return "NoError";
    case QAbstractVideoSurface::UnsupportedFormatError:
      return "UnsupportedFormatError";
    case QAbstractVideoSurface::IncorrectFormatError:
      return "IncorrectFormatError";
    case QAbstractVideoSurface::StoppedError:
      return "StoppedError";
    case QAbstractVideoSurface::ResourceError:
      return "ResourceError";
    default:
      return "UnknownError";
  }
}
}

void ImageTransportSubscriber::presentFrame( const QVideoFrame &frame )
{

  const QVideoSurfaceFormat &surface_format = surface_->surfaceFormat();
  if ( surface_format.frameWidth() != frame.width() || surface_format.frameHeight() != frame.height() ||
       surface_format.pixelFormat() != frame.pixelFormat())
  {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat());
    surface_->stop();
  }
  if ( !surface_->isActive())
  {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat());
    if ( format_.pixelFormat() == QVideoFrame::Format_Invalid )
    {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Could not find compatible format for video surface." );
      shutdownSubscriber();
      return;
    }
    if ( !surface_->start( format_ ))
    {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to start video surface: %s",
                       videoSurfaceErrorToString( surface_->error()));
      shutdownSubscriber();
      return;
    }
  }
  surface_->present( frame );
  last_frame_timestamp_ = ros::Time::now();
  if ( timeout_ != 0 )
  {
    no_image_timer_.start( throttle_interval_ + timeout_ );
  }
}

QString ImageTransportSubscriber::topic() const
{
  if ( subscription_ ) return QString::fromStdString( subscription_->getTopic());
  return topic_;
}

void ImageTransportSubscriber::setTopic( const QString &value )
{
  if ( topic_ == value ) return;
  topic_ = value;
  emit topicChanged();
}

const QString &ImageTransportSubscriber::defaultTransport() const { return default_transport_; }

void ImageTransportSubscriber::setDefaultTransport( const QString &value )
{
  if ( default_transport_ == value ) return;
  default_transport_ = value;
  emit defaultTransportChanged();
}

bool ImageTransportSubscriber::subscribed() const { return subscribed_; }

int ImageTransportSubscriber::timeout() const { return timeout_; }

void ImageTransportSubscriber::setTimeout( int value )
{
  timeout_ = value;
  emit timeoutChanged();
}

double ImageTransportSubscriber::throttleRate() const
{
  return 1000.0 / throttle_interval_;
}

void ImageTransportSubscriber::setThrottleRate( double value )
{
  throttle_interval_ = value == 0 ? 0 : static_cast<int>(1000 / value);
  if ( subscription_ )
  {
    subscription_->updateThrottleInterval( throttle_interval_ );
  }
  emit throttleRateChanged();
}
}
