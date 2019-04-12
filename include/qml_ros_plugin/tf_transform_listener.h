// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TF_TRANSFORM_LISTENER_H
#define QML_ROS_PLUGIN_TF_TRANSFORM_LISTENER_H

#include <QObject>
#include <memory>

namespace qml_ros_plugin
{

class TfTransformListener : public QObject
{
Q_OBJECT
private:
  TfTransformListener();

public:
  static TfTransformListener &getInstance();

  TfTransformListener( const TfTransformListener & ) = delete;

  void operator=( const TfTransformListener & ) = delete;

  ~TfTransformListener() override;

  /*!
   * Checks if a transform is possible. Returns true if possible, otherwise either false or if available a message why
   * the transform failed.
   * @param target_frame The frame into which to transform
   * @param source_frame The frame from which to transform
   * @param time_sec The time at which to transform in seconds
   * @param timeout How long to block before failing
   * @return True if the transform is possible, otherwise an error message if available, false if not.
   */
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QString &source_frame, double time_sec = 0,
                                     double timeout = 0 ) const;

  /*!
   *
   * Checks if a transform is possible. Returns true if possible, otherwise either false or if available a message why
   * the transform failed.
   * @param target_frame The frame into which to transform
   * @param target_time_sec The time into which to transform
   * @param source_frame The frame from which to transform
   * @param source_time_sec The time from which to transform
   * @param fixed_frame The frame in which to treat the transform as constant in time
   * @param timeout How long to block before failing
   * @return True if the transform is possible, otherwise an error message if available, false if not.
   */
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, double target_time_sec,
                                     const QString &source_frame, double source_time_sec,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                                           double time_sec = 0, double timeout = 0 );

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, double target_time_sec,
                                           const QString &source_frame, double source_time_sec,
                                           const QString &fixed_frame, double timeout = 0 );

signals:

  void transformChanged();

protected:
  void onTransformChanged();

  struct State;
  std::shared_ptr<State> state_;
};

/*!
 * A wraper around the TfTransformListener singleton to allow the QML engine to create and destroy it.
 */
class TfTransformListenerWrapper : public QObject
{
Q_OBJECT
public:
  TfTransformListenerWrapper();

  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QString &source_frame, double time_sec = 0,
                                     double timeout = 0 ) const;

  Q_INVOKABLE QVariant canTransform( const QString &target_frame, double target_time_sec,
                                     const QString &source_frame, double source_time_sec,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                                           double time_sec = 0, double timeout = 0 );

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, double target_time_sec,
                                           const QString &source_frame, double source_time_sec,
                                           const QString &fixed_frame, double timeout = 0 );

signals:
  void transformChanged();
};
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_TF_TRANSFORM_LISTENER_H
