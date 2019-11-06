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
/*!
 * Represents a tf transform between source and target frame.
 */
class TfTransform : public QObject
{
Q_OBJECT
  // @formatter:off
  //! The source frame of the tf transform, i.e., the frame where the data originated.
  Q_PROPERTY( QString sourceFrame READ sourceFrame WRITE setSourceFrame NOTIFY sourceFrameChanged )

  //! The target frame of the tf transform, i.e., the frame to which the data should be transformed.
  Q_PROPERTY( QString targetFrame READ targetFrame WRITE setTargetFrame NOTIFY targetFrameChanged )

  //! Whether this tf transform is active, i.e., receiving transform updates.
  Q_PROPERTY( bool active READ active WRITE setActive NOTIFY activeChanged )

  //! The last received transform as a geometry_msgs/TransformStamped with an added boolean valid field and optional
  //! error fields. See TfTransformListener::lookUpTransform
  Q_PROPERTY( QVariantMap message READ message NOTIFY messageChanged )

  //! The translation part of the tf transform as a vector with x, y, z fields. Zero if no valid transform available (yet).
  Q_PROPERTY( QVariant translation READ translation NOTIFY translationChanged )

  //! The rotation part of the tf transform as a quaternion with w, x, y, z fields. Identity if no valid transform available (yet).
  Q_PROPERTY( QVariant rotation READ rotation NOTIFY rotationChanged )

  //! Whether the current transform, i.e., the fields message, translation and rotation are valid.
  Q_PROPERTY( bool valid READ valid NOTIFY validChanged )
  // @formatter:on
public:
  TfTransform();

  ~TfTransform() override;

  const QString &sourceFrame() const;

  void setSourceFrame( const QString &value );

  const QString &targetFrame() const;

  void setTargetFrame( const QString &targetFrame );

  bool active() const;

  void setActive( bool value );

  const QVariantMap &message() const;

  const QVariant &translation() const;

  const QVariant &rotation() const;

  bool valid() const;

signals:

  void sourceFrameChanged();

  void targetFrameChanged();

  void activeChanged();

  void messageChanged();

  void translationChanged();

  void rotationChanged();

  void validChanged();

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
