// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H

#include "qml_ros_plugin/qobject_ros.h"
#include "qml_ros_plugin/node_handle.h"

#include <QAbstractVideoSurface>
#include <QVideoSurfaceFormat>

#include <image_transport/subscriber.h>
#include <image_transport/image_transport.h>

namespace qml_ros_plugin
{

class ImageTransportSubscriber : public QObjectRos
{
Q_OBJECT
  //! @formatter:off
  //! Interface for QML. This is the surface the images are passed to.
  Q_PROPERTY(QAbstractVideoSurface *videoSurface READ videoSurface WRITE setVideoSurface)
  //! The image base topic (without image_raw etc.)
  Q_PROPERTY(QString topic READ topic WRITE setTopic NOTIFY topicChanged)
  //! The default transport passed as transport hint. May be overridden by a parameter. (Default: compressed)
  Q_PROPERTY(QString defaultTransport READ defaultTransport WRITE setDefaultTransport NOTIFY defaultTransportChanged)
  //! @formatter:on
public:

  ImageTransportSubscriber( NodeHandle *nh, QString topic, quint32 queue_size );

  ImageTransportSubscriber();

  QAbstractVideoSurface *videoSurface() const;

  void setVideoSurface( QAbstractVideoSurface *surface );

  QString topic() const;

  void setTopic( const QString &value );

  const QString &defaultTransport() const;

  void setDefaultTransport( const QString &value );

signals:

  void topicChanged();

  void defaultTransportChanged();

private slots:

  void onNodeHandleReady();

private:
  void imageCallback( const sensor_msgs::ImageConstPtr &img );

  Q_INVOKABLE void processImage();

  void onRosShutdown() override;

  void subscribe();

  void unsubscribe();

  QVideoSurfaceFormat format_;
  QString topic_;
  QString default_transport_;
  image_transport::Subscriber subscriber_;
  sensor_msgs::ImageConstPtr last_image_;
  NodeHandleReference nh_;
  std::unique_ptr<image_transport::ImageTransport> transport_;
  QAbstractVideoSurface *surface_;
  quint32 queue_size_;
  bool subscribed_;
};
}

#endif //QML_ROS_PLUGIN_IMAGE_TRANSPORT_SUBSCRIBER_H
