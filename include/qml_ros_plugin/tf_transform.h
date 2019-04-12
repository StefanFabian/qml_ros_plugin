// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TF_TRANSFORM_H
#define QML_ROS_PLUGIN_TF_TRANSFORM_H

#include <QObject>
#include <QVariantMap>
#include <memory>

namespace boost
{
namespace signals2
{
class connection;
}
}

namespace qml_ros_plugin
{
class TfTransform : public QObject
{
Q_OBJECT
  Q_PROPERTY( QString sourceFrame
                READ sourceFrame
                WRITE setSourceFrame
                NOTIFY
                sourceFrameChanged )
  Q_PROPERTY( QString targetFrame
                READ targetFrame
                WRITE setTargetFrame
                NOTIFY
                targetFrameChanged )
  Q_PROPERTY( bool active
                READ active
                WRITE setActive
                NOTIFY
                activeChanged )
  Q_PROPERTY( QVariant message
                READ message
                NOTIFY
                messageChanged )
  Q_PROPERTY( QVariant translation
                READ translation
                NOTIFY
                translationChanged )
  Q_PROPERTY( QVariant rotation
                READ rotation
                NOTIFY
                rotationChanged )
public:
  TfTransform();

  ~TfTransform() override;

  const QString &sourceFrame() const;

  void setSourceFrame( const QString &value );

  const QString &targetFrame() const;

  void setTargetFrame( const QString &targetFrame );

  bool active() const;

  void setActive( bool value );

  const QVariant message() const;

  const QVariant &translation() const;

  const QVariant &rotation() const;

signals:
  void sourceFrameChanged();

  void targetFrameChanged();

  void activeChanged();

  void messageChanged();

  void translationChanged();

  void rotationChanged();

protected slots:

  void onTransformChanged();

protected:
  void subscribe();

  void shutdown();

  bool active_;
  QString source_frame_;
  QString target_frame_;
  QVariantMap message_;
};
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_TF_TRANSFORM_H
