// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H

#include "qml_ros_plugin/qobject_ros.h"
#include "qml_ros_plugin/node_handle.h"

#include <QAbstractVideoSurface>
#include <QTimer>
#include <QVideoSurfaceFormat>

#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>
#include <mutex>

namespace qml_ros_plugin
{

class ImageTransportSubscriptionHandle;

class ImageTransportSubscriber : public QObjectRos
{
Q_OBJECT
  // @formatter:off
  //! Interface for QML. This is the surface the images are passed to.
  Q_PROPERTY(QAbstractVideoSurface *videoSurface READ videoSurface WRITE setVideoSurface)
  //! The image base topic (without image_raw etc.). This value may change once the subscriber is connected and private
  //! topic names or remappings were evaluated.
  Q_PROPERTY(QString topic READ topic WRITE setTopic NOTIFY topicChanged)
  //! The default transport passed as transport hint. May be overridden by a parameter. (Default: compressed)
  Q_PROPERTY(QString defaultTransport READ defaultTransport WRITE setDefaultTransport NOTIFY defaultTransportChanged)
  //! Whether or not this ImageTransportSubscriber is subscribed to the given topic
  Q_PROPERTY(bool subscribed READ subscribed NOTIFY subscribedChanged)
  //! The timeout when no image is received until a blank frame is served. Set to 0 to disable and always show last frame.
  //! Default is 3000 ms.
  Q_PROPERTY(int timeout READ timeout WRITE setTimeout NOTIFY timeoutChanged)
  //! The update rate to throttle image receiving in images per second. Set to 0 to disable throttling.
  //! Default is 0 (disabled).
  Q_PROPERTY(double throttleRate READ throttleRate WRITE setThrottleRate NOTIFY throttleRateChanged)
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

  bool subscribed() const;

  int timeout() const;

  void setTimeout( int value );

  double throttleRate() const;

  void setThrottleRate( double value );

signals:

  void topicChanged();

  void defaultTransportChanged();

  void subscribedChanged();

  void timeoutChanged();

  void throttleRateChanged();

private slots:

  void onNodeHandleReady();

  void onNoImageTimeout();

  void presentFrame( const QVideoFrame &frame );

private:
  void onRosShutdown() override;

  void initSubscriber();

  void shutdownSubscriber();

  QTimer no_image_timer_;
  QVideoSurfaceFormat format_;
  QString topic_;
  QString default_transport_;
  NodeHandle::Ptr nh_;
  std::shared_ptr<ImageTransportSubscriptionHandle> subscription_;
  QAbstractVideoSurface *surface_ = nullptr;
  ros::Time last_frame_timestamp_;
  quint32 queue_size_;
  int throttle_interval_ = 0;
  int timeout_ = 3000;
  bool subscribed_ = false;
};
}

#endif //QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
