// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/image_transport_subscriber.h"
#include "qml_ros_plugin/image_buffer.h"

namespace qml_ros_plugin
{

ImageTransportSubscriber::ImageTransportSubscriber( NodeHandle *nh, QString topic, quint32 queue_size )
  : topic_( std::move( topic )), default_transport_( "compressed" ), nh_( nh ), surface_( nullptr )
    , queue_size_( queue_size ), subscribed_( false )
{
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
  subscribe();
}

ImageTransportSubscriber::ImageTransportSubscriber()
  : default_transport_( "raw" ), nh_( new NodeHandle, true ), surface_( nullptr ), queue_size_( 1 )
    , subscribed_( false )
{
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
}

QAbstractVideoSurface *ImageTransportSubscriber::videoSurface() const { return surface_; }

void ImageTransportSubscriber::setVideoSurface( QAbstractVideoSurface *surface )
{
  if ( surface == surface_ ) return;
  unsubscribe();
  surface_ = surface;
  subscribe();
}

void ImageTransportSubscriber::onNodeHandleReady()
{
  transport_.reset( new image_transport::ImageTransport( nh_->nodeHandle()));
  subscribe();
}

void ImageTransportSubscriber::onRosShutdown()
{
  unsubscribe();
  transport_.reset();
}

void ImageTransportSubscriber::subscribe()
{
  if ( subscribed_ ) unsubscribe();
  // This makes sure we lazy subscribe and only subscribe if there is a surface to write to
  if ( surface_ == nullptr ) return;
  if ( !nh_->isReady()) return;
  if ( topic_.isEmpty()) return;
  if ( transport_ == nullptr ) return;
  // TODO Transport hints
  image_transport::TransportHints transport_hints( default_transport_.toStdString());
  try
  {
    subscriber_ = transport_->subscribe( topic_.toStdString(), queue_size_, &ImageTransportSubscriber::imageCallback,
                                         this, transport_hints );
    ROS_DEBUG_NAMED( "qml_ros_plugin", "Subscribed to '%s' with transport '%s'.", topic_.toStdString().c_str(),
                     transport_hints.getTransport().c_str());
  }
  catch ( image_transport::TransportLoadException &ex )
  {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Could not subscribe to image topic: %s", ex.what());
    return;
  }
  subscribed_ = true;
  if ( subscribed_ && surface_ != nullptr )
    surface_->start( format_ );
}

void ImageTransportSubscriber::unsubscribe()
{
  subscriber_.shutdown();
  if ( surface_ != nullptr && surface_->isActive())
    surface_->stop();
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

void ImageTransportSubscriber::imageCallback( const sensor_msgs::ImageConstPtr &img )
{
  if ( surface_ == nullptr ) return;
  last_image_ = img;
  QMetaObject::invokeMethod( this, "processImage", Qt::AutoConnection );
}

void ImageTransportSubscriber::processImage()
{
  if ( last_image_ == nullptr )
  {
    surface_->present( QVideoFrame());
    return;
  }
  auto buffer = new ImageBuffer( last_image_, surface_->supportedPixelFormats());
  if ( !surface_->isActive())
  {
    format_ = QVideoSurfaceFormat( QSize( last_image_->width, last_image_->height ), buffer->format());
    if ( !surface_->start( format_ ))
    {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to start video surface: %s",
                       videoSurfaceErrorToString( surface_->error()));
      unsubscribe();
    }
  }
  surface_->present( QVideoFrame( buffer, QSize( last_image_->width, last_image_->height ), buffer->format()));
}

const QString &ImageTransportSubscriber::topic() const
{
  return topic_;
}

void ImageTransportSubscriber::setTopic( const QString &value )
{
  if ( topic_ == value ) return;
  topic_ = value;
  emit topicChanged();
}

const QString &ImageTransportSubscriber::defaultTransport() const
{
  return default_transport_;
}

void ImageTransportSubscriber::setDefaultTransport( const QString &value )
{
  if ( default_transport_ == value ) return;
  default_transport_ = value;
  emit defaultTransportChanged();
}
}
