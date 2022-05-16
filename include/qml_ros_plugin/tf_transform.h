// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TF_TRANSFORM_H
#define QML_ROS_PLUGIN_TF_TRANSFORM_H

#include <QObject>
#include <QTimer>
#include <QVariantMap>
#include <memory>

namespace boost
{
namespace signals2
{
class connection;
}
} // namespace boost

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

  //! Whether this tf transform is enabled, i.e., receiving transform updates.
  Q_PROPERTY( bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged )
  //! Alias for enabled
  Q_PROPERTY( bool active READ enabled WRITE setEnabled NOTIFY enabledChanged )

  //! The last received transform as a geometry_msgs/TransformStamped with an added boolean valid
  //! field and optional error fields. See TfTransformListener::lookUpTransform
  Q_PROPERTY( QVariantMap transform READ message NOTIFY messageChanged )
  //! An alias for transform.
  Q_PROPERTY( QVariantMap message READ message NOTIFY messageChanged )

  //! The translation part of the tf transform as a vector with x, y, z fields. Zero if no valid transform available (yet).
  Q_PROPERTY( QVariant translation READ translation NOTIFY translationChanged )

  //! The rotation part of the tf transform as a quaternion with w, x, y, z fields. Identity if no valid transform available (yet).
  Q_PROPERTY( QVariant rotation READ rotation NOTIFY rotationChanged )

  //! The maximum rate in Hz at which tf updates are processed and emitted as changed signals.
  //! Default: 60 Note: The rate can not exceed 1000. To disable rate limiting set to 0.
  Q_PROPERTY( qreal rate READ rate WRITE setRate NOTIFY rateChanged )

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

  bool enabled() const;

  void setEnabled( bool value );

  qreal rate() const;

  void setRate( qreal value );

  const QVariantMap &message();

  const QVariant &translation();

  const QVariant &rotation();

  bool valid();

signals:

  void sourceFrameChanged();

  void targetFrameChanged();

  void enabledChanged();

  void rateChanged();

  void messageChanged();

  void translationChanged();

  void rotationChanged();

  void validChanged();

protected slots:

  void onTransformChanged();

  void updateMessage();

protected:
  void subscribe();

  void shutdown();

  QTimer throttle_timer_;
  QVariantMap message_;
  QString source_frame_;
  QString target_frame_;
  std::chrono::system_clock::time_point last_transform_;
  std::chrono::milliseconds throttle_time_;
  bool enabled_;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_TF_TRANSFORM_H
