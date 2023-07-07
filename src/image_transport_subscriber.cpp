// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/image_transport_subscriber.h"
#include "qml_ros_plugin/image_buffer.h"
#include "qml_ros_plugin/image_transport_manager.h"
#include "qml_ros_plugin/ros.h"

namespace qml_ros_plugin
{

ImageTransportSubscriber::ImageTransportSubscriber( NodeHandle::Ptr nh, QString topic,
                                                    quint32 queue_size )
    : topic_( std::move( topic ) ), default_transport_( "compressed" ), nh_( std::move( nh ) ),
      queue_size_( queue_size )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::onNoImageTimeout,
           Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
  initSubscriber();
}

ImageTransportSubscriber::ImageTransportSubscriber()
    : default_transport_( "compressed" ), nh_( std::make_shared<NodeHandle>() ), queue_size_( 1 )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscriber::onNoImageTimeout,
           Qt::AutoConnection );
  connect( nh_.get(), &NodeHandle::ready, this, &ImageTransportSubscriber::onNodeHandleReady );
}

QAbstractVideoSurface *ImageTransportSubscriber::videoSurface() const { return surface_; }

void ImageTransportSubscriber::setVideoSurface( QAbstractVideoSurface *surface )
{
  if ( surface == surface_ )
    return;
  if ( surface_ != nullptr && surface_->isActive() )
    surface_->stop();
  surface_ = surface;
  if ( surface_ == nullptr && subscribed_ ) {
    shutdownSubscriber( true );
    return;
  }
  if ( last_frame_.isValid() )
    presentFrame( last_frame_ );
  initSubscriber();
}

void ImageTransportSubscriber::onNodeHandleReady() { initSubscriber(); }

void ImageTransportSubscriber::onRosShutdown() { shutdownSubscriber( true ); }

void ImageTransportSubscriber::initSubscriber()
{
  // This makes sure we lazy subscribe and only subscribe if there is a surface to write to
  if ( surface_ == nullptr )
    return;
  if ( !enabled_ )
    return;
  if ( !nh_->isReady() )
    return;
  if ( topic_.isEmpty() )
    return;
  bool was_subscribed = subscribed_;
  // TODO Transport hints
  image_transport::TransportHints transport_hints( default_transport_.toStdString() );
  subscription_ = ImageTransportManager::getInstance().subscribe(
      nh_, topic_, queue_size_, transport_hints,
      [this]( const QVideoFrame &frame ) { presentFrame( frame ); }, surface_ );
  subscribed_ = subscription_ != nullptr;
  if ( was_subscribed != subscribed_ )
    emit subscribedChanged();
}

void ImageTransportSubscriber::shutdownSubscriber( bool stop_surface )
{
  subscription_.reset();
  if ( stop_surface && surface_ != nullptr && surface_->isActive() )
    surface_->stop();
  if ( !subscribed_ )
    return;
  subscribed_ = false;
  emit subscribedChanged();
}

void ImageTransportSubscriber::onNoImageTimeout()
{
  using namespace std::chrono_literals;
  if ( surface_ == nullptr || !surface_->isActive() )
    return;
  if ( paused_ ) {
    // Check again in 30ms
    no_image_timer_.start( 30ms );
    return;
  }
  int elapsed_time_milliseconds =
      static_cast<int>( ( ros::Time::now() - last_frame_timestamp_ ).toNSec() / 1000000 );

  if ( timeout_ == 0 )
    return;
  if ( elapsed_time_milliseconds < timeout_ ) {
    no_image_timer_.start( timeout_ - elapsed_time_milliseconds );
    return;
  }
  surface_->present( QVideoFrame() );
}

namespace
{
const char *videoSurfaceErrorToString( QAbstractVideoSurface::Error error )
{
  switch ( error ) {
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
} // namespace

void ImageTransportSubscriber::presentFrame( const QVideoFrame &frame )
{
  if ( surface_ == nullptr || paused_ )
    return;
  const QVideoSurfaceFormat &surface_format = surface_->surfaceFormat();
  if ( surface_format.frameWidth() != frame.width() ||
       surface_format.frameHeight() != frame.height() ||
       surface_format.pixelFormat() != frame.pixelFormat() ) {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat() );
    surface_->stop();
  }
  if ( !surface_->isActive() ) {
    format_ = QVideoSurfaceFormat( frame.size(), frame.pixelFormat() );
    if ( format_.pixelFormat() == QVideoFrame::Format_Invalid ) {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Could not find compatible format for video surface." );
      shutdownSubscriber( true );
      return;
    }
    if ( !surface_->start( format_ ) ) {
      ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to start video surface: %s",
                       videoSurfaceErrorToString( surface_->error() ) );
      shutdownSubscriber( true );
      return;
    }
  }
  last_frame_ = frame;
  surface_->present( frame );
  // Return if this is the last frame of our subscription.
  if ( subscription_ == nullptr )
    return;

  bool network_latency_changed = last_network_latency_ != subscription_->networkLatency();
  bool processing_latency_changed = last_processing_latency_ != subscription_->processingLatency();
  if ( network_latency_changed )
    emit networkLatencyChanged();
  if ( processing_latency_changed )
    emit processingLatencyChanged();
  if ( network_latency_changed || processing_latency_changed )
    emit latencyChanged();
  if ( std::abs( last_framerate_ - subscription_->framerate() ) > 0.1 )
    emit framerateChanged();
  last_framerate_ = subscription_->framerate();
  last_frame_timestamp_ = ros::Time::now();
  last_network_latency_ = subscription_->networkLatency();
  last_processing_latency_ = subscription_->processingLatency();
  if ( timeout_ != 0 ) {
    no_image_timer_.start( throttle_interval_ + timeout_ );
  }
}

QString ImageTransportSubscriber::topic() const
{
  if ( subscription_ )
    return QString::fromStdString( subscription_->getTopic() );
  return topic_;
}

void ImageTransportSubscriber::setTopic( const QString &value )
{
  if ( topic_ == value )
    return;
  shutdownSubscriber( true );
  topic_ = value;
  emit topicChanged();
  initSubscriber();
}

const QString &ImageTransportSubscriber::defaultTransport() const { return default_transport_; }

void ImageTransportSubscriber::setDefaultTransport( const QString &value )
{
  if ( default_transport_ == value )
    return;
  shutdownSubscriber( true );
  default_transport_ = value;
  emit defaultTransportChanged();
  initSubscriber();
}

int ImageTransportSubscriber::minimumWidth() const { return minimum_size_.width(); }

void ImageTransportSubscriber::setMinimumWidth( int value )
{
  minimum_size_.setWidth( value );
  if ( subscription_ != nullptr )
    subscription_->setMinimumSize( minimum_size_ );
  emit minimumWidthChanged();
}

int ImageTransportSubscriber::minimumHeight() const { return minimum_size_.height(); }

void ImageTransportSubscriber::setMinimumHeight( int value )
{
  minimum_size_.setHeight( value );
  if ( subscription_ != nullptr )
    subscription_->setMinimumSize( minimum_size_ );
  emit minimumHeightChanged();
}

double ImageTransportSubscriber::minimumFramerate() const { return minimum_framerate_; }
void ImageTransportSubscriber::setMinimumFramerate( double value )
{
  minimum_framerate_ = value;
  if ( subscription_ != nullptr )
    subscription_->setMinimumFramerate( value );
  emit minimumFramerateChanged();
}

bool ImageTransportSubscriber::subscribed() const { return subscribed_; }

int ImageTransportSubscriber::timeout() const { return timeout_; }

void ImageTransportSubscriber::setTimeout( int value )
{
  timeout_ = value;
  emit timeoutChanged();
}

bool ImageTransportSubscriber::enabled() const { return enabled_; }

void ImageTransportSubscriber::setEnabled( bool value )
{
  if ( enabled_ == value )
    return;
  enabled_ = value;
  if ( enabled_ )
    initSubscriber();
  else
    shutdownSubscriber( true );
  emit enabledChanged();
  emit playbackStateChanged( playbackState() );
}

double ImageTransportSubscriber::framerate() const
{
  return subscription_ == nullptr ? 0 : subscription_->framerate();
}

int ImageTransportSubscriber::latency() const
{
  return subscription_ == nullptr ? -1 : subscription_->latency();
}

int ImageTransportSubscriber::networkLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->networkLatency();
}

int ImageTransportSubscriber::processingLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->processingLatency();
}

QMediaPlayer::State ImageTransportSubscriber::playbackState() const
{
  return subscription_ == nullptr ? QMediaPlayer::StoppedState
         : paused_                ? QMediaPlayer::PausedState
                                  : QMediaPlayer::PlayingState;
}

void ImageTransportSubscriber::play()
{
  if ( subscription_ && !paused_ && enabled_ )
    return;
  paused_ = false;
  if ( !enabled_ ) {
    setEnabled( true );
  } else if ( !subscription_ ) {
    initSubscriber();
    emit playbackStateChanged( playbackState() );
  }
}

void ImageTransportSubscriber::pause()
{
  if ( paused_ )
    return;
  paused_ = true;
  emit playbackStateChanged( playbackState() );
}

void ImageTransportSubscriber::stop()
{
  if ( !subscription_ && !enabled_ )
    return;
  setEnabled( false );
}
} // namespace qml_ros_plugin
