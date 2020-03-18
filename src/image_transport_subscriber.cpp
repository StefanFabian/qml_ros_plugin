// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/image_transport_subscriber.h"
#include "qml_ros_plugin/image_buffer.h"
#include "qml_ros_plugin/ros.h"

namespace qml_ros_plugin
{

ImageTransportSubscriber::ImageTransportSubscriber( NodeHandle *nh, QString topic, quint32 queue_size )
  : topic_( std::move( topic )), default_transport_( "compressed" ), nh_( nh ), queue_size_( queue_size )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::processImage, Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
  subscribe();
}

ImageTransportSubscriber::ImageTransportSubscriber()
  : default_transport_( "compressed" ), nh_( new NodeHandle( RosQml::getInstance().backgroundQueue()), true )
    , queue_size_( 1 )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::processImage, Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
}

QAbstractVideoSurface *ImageTransportSubscriber::videoSurface() const { return surface_; }

void ImageTransportSubscriber::setVideoSurface( QAbstractVideoSurface *surface )
{
  if ( surface == surface_ ) return;

  bool subscribed = subscribed_;
  blockSignals( true );
  unsubscribe();
  surface_ = surface;
  subscribe();
  blockSignals( false );
  if ( subscribed != subscribed_ ) emit subscribedChanged();
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
  bool was_subscribed = subscribed_;
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
  if ( !was_subscribed ) emit subscribedChanged();
}

void ImageTransportSubscriber::unsubscribe()
{
  if ( !subscribed_ ) return;
  subscriber_.shutdown();
  if ( surface_ != nullptr && surface_->isActive())
    surface_->stop();
  subscribed_ = false;
  emit subscribedChanged();
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
  auto buffer = new ImageBuffer( img, surface_->supportedPixelFormats());
  {
    std::lock_guard<std::mutex> lock( image_lock_ );
    last_image_ = img;
    buffer_ = buffer;
  }
  QMetaObject::invokeMethod( this, "processImage", Qt::AutoConnection );
}

void ImageTransportSubscriber::processImage()
{
  if ( surface_ == nullptr ) return;
  std::lock_guard<std::mutex> lock( image_lock_ );
  // Show blank image after {timeout} seconds of no image
  ros::Time now = ros::Time::now();
  if ( buffer_ == nullptr )
  {
    int elapsed_time_ = static_cast<int>((now - last_frame_timestamp_).toNSec() / 1000000);

    if ( timeout_ == 0 ) return;
    if ( elapsed_time_ < timeout_ )
    {
      no_image_timer_.start( timeout_ - elapsed_time_ );
      return;
    }
    surface_->present( QVideoFrame());
    return;
  }
  last_frame_timestamp_ = now;
  if ( timeout_ != 0 )
    no_image_timer_.start( timeout_ );

  const QVideoSurfaceFormat &surface_format = surface_->surfaceFormat();
  if ( surface_format.frameWidth() != int( last_image_->width ) ||
       surface_format.frameHeight() != int( last_image_->height ) || surface_format.pixelFormat() != buffer_->format())
  {
    format_ = QVideoSurfaceFormat( QSize( last_image_->width, last_image_->height ), buffer_->format());
    surface_->stop();
  }
  if ( !surface_->isActive())
  {
    format_ = QVideoSurfaceFormat( QSize( last_image_->width, last_image_->height ), buffer_->format());
    if ( format_.pixelFormat() == QVideoFrame::Format_Invalid )
    {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Could not find compatible format for video surface." );
      unsubscribe();
      return;
    }
    if ( !surface_->start( format_ ))
    {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to start video surface: %s",
                       videoSurfaceErrorToString( surface_->error()));
      unsubscribe();
      return;
    }
  }
  surface_->present( QVideoFrame( buffer_, QSize( last_image_->width, last_image_->height ), buffer_->format()));
  buffer_ = nullptr;
}

QString ImageTransportSubscriber::topic() const
{
  if ( subscribed_ ) return QString::fromStdString( subscriber_.getTopic());
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
}
