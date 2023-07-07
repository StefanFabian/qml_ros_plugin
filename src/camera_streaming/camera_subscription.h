//
// Created by stefan on 28.06.23.
//

#ifndef QML_ROS_PLUGIN_CAMERA_SUBSCRIPTION_H
#define QML_ROS_PLUGIN_CAMERA_SUBSCRIPTION_H

#include <QObject>
#include <QList>
#include <QVideoFrame>

namespace qml_ros_plugin
{

class ImageBuffer;

class CameraSubscription : public QObject
{
  Q_OBJECT
public:
  virtual std::string topic() const = 0;

  virtual std::string transport() const = 0;

  virtual void setSupportedFormats( QList<QVideoFrame::PixelFormat> formats ) = 0;

  virtual double framerate() const = 0;

  virtual ImageBuffer *takeBuffer() = 0;

signals:
  void newImage();
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_CAMERA_SUBSCRIPTION_H
