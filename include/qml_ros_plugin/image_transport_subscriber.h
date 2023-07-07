// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H

#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/qobject_ros.h"

#include <QAbstractVideoSurface>
#include <QMediaPlayer>
#include <QTimer>
#include <QVideoSurfaceFormat>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <mutex>

namespace qml_ros_plugin
{

class ImageTransportSubscriptionHandle;

class ImageTransportSubscriber : public QObjectRos
{
  Q_OBJECT
  // @formatter:off
  //! Interface for QML. This is the surface the images are passed to.
  Q_PROPERTY( QAbstractVideoSurface *videoSurface READ videoSurface WRITE setVideoSurface )
  //! The image base topic (without image_raw etc.). This value may change once the subscriber is
  //! connected and private topic names or remappings were evaluated.
  Q_PROPERTY( QString topic READ topic WRITE setTopic NOTIFY topicChanged )
  //! The default transport passed as transport hint. May be overridden by a parameter. (Default: compressed)
  Q_PROPERTY( QString defaultTransport READ defaultTransport WRITE setDefaultTransport NOTIFY
                  defaultTransportChanged )
  //! Specifies the minimum width of the rtsp stream if you use the hector_camera_server package to serve (ROS) camera streams via RTSP.
  Q_PROPERTY( int minimumFrameWidth READ minimumWidth WRITE setMinimumWidth NOTIFY minimumWidthChanged )
  //! Specifies the minimum width of the rtsp stream if you use the hector_camera_server package to serve (ROS) camera streams via RTSP.
  Q_PROPERTY( int minimumFrameHeight READ minimumHeight WRITE setMinimumHeight NOTIFY minimumHeightChanged )
  //! Specifies the minimum framerate of the rtsp stream if you use the hector_camera_server package to serve (ROS) camera streams via RTSP.
  Q_PROPERTY( double minimumFramerate READ minimumFramerate WRITE setMinimumFramerate NOTIFY
                  minimumFramerateChanged )
  //! Whether or not this ImageTransportSubscriber is subscribed to the given topic
  Q_PROPERTY( bool subscribed READ subscribed NOTIFY subscribedChanged )
  //! The latency from the sender to the received time in ms not including the conversion latency before displaying.
  //! This latency is based on the ROS time of the sending and receiving machines, hence, they need to be synchronized.
  Q_PROPERTY( int networkLatency READ networkLatency NOTIFY networkLatencyChanged )
  //! The latency (in ms) from the reception of the image until it is in a displayable format.
  Q_PROPERTY( int processingLatency READ processingLatency NOTIFY processingLatencyChanged )
  //! The full latency (in ms) from the camera to your display excluding drawing time.
  Q_PROPERTY( int latency READ latency NOTIFY latencyChanged )
  //! The framerate of the received camera frames in frames per second.
  Q_PROPERTY( double framerate READ framerate NOTIFY framerateChanged )
  //! The timeout when no image is received until a blank frame is served. Set to 0 to disable and
  //! always show last frame. Default is 3000 ms.
  Q_PROPERTY( int timeout READ timeout WRITE setTimeout NOTIFY timeoutChanged )
  //! Whether the subscriber is active or not. Setting to false will shut down subscribers
  Q_PROPERTY( bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged )
  //! The playback state of this video source. Can be modified with play and pause
  Q_PROPERTY( QMediaPlayer::State playbackState READ playbackState NOTIFY playbackStateChanged )
  // @formatter:on
public:
  ImageTransportSubscriber( NodeHandle::Ptr nh, QString topic, quint32 queue_size );

  ImageTransportSubscriber();

  QAbstractVideoSurface *videoSurface() const;

  void setVideoSurface( QAbstractVideoSurface *surface );

  QString topic() const;

  void setTopic( const QString &value );

  const QString &defaultTransport() const;

  void setDefaultTransport( const QString &value );

  int minimumWidth() const;

  void setMinimumWidth( int value );

  int minimumHeight() const;

  void setMinimumHeight( int value );

  double minimumFramerate() const;

  void setMinimumFramerate( double value );

  bool subscribed() const;

  int timeout() const;

  void setTimeout( int value );

  bool enabled() const;

  void setEnabled( bool value );

  double framerate() const;

  int latency() const;

  int networkLatency() const;

  int processingLatency() const;

  QMediaPlayer::State playbackState() const;

  //! Starts playing the stream. If not enabled, will set enabled to true.
  Q_INVOKABLE void play();

  //! Pauses the stream. Will not shut down the subscriber and therefore be quicker to resume at
  //!   the cost of still consuming bandwidth.
  Q_INVOKABLE void pause();

  //! Shuts down the subscriber. Similar as setEnabled.
  Q_INVOKABLE void stop();

signals:

  void topicChanged();

  void defaultTransportChanged();

  void minimumWidthChanged();

  void minimumHeightChanged();

  void minimumFramerateChanged();

  void subscribedChanged();

  void timeoutChanged();

  void framerateChanged();

  void latencyChanged();

  void networkLatencyChanged();

  void processingLatencyChanged();

  void enabledChanged();

  void playbackStateChanged( QMediaPlayer::State playbackState );

private slots:

  void onNodeHandleReady();

  void onNoImageTimeout();

  void presentFrame( const QVideoFrame &frame );

private:
  void onRosShutdown() override;

  void initSubscriber();

  void shutdownSubscriber( bool stop_surface );

  QTimer no_image_timer_;
  QVideoSurfaceFormat format_;
  QString topic_;
  QString default_transport_;
  QSize minimum_size_;
  QVideoFrame last_frame_;
  NodeHandle::Ptr nh_;
  std::shared_ptr<ImageTransportSubscriptionHandle> subscription_;
  QAbstractVideoSurface *surface_ = nullptr;
  ros::Time last_frame_timestamp_;
  double minimum_framerate_ = 30;
  double last_framerate_ = 0;
  quint32 queue_size_;
  int throttle_interval_ = 0;
  int last_network_latency_ = -1;
  int last_processing_latency_ = -1;
  int timeout_ = 3000;
  bool subscribed_ = false;
  bool enabled_ = true;
  bool paused_ = false;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
