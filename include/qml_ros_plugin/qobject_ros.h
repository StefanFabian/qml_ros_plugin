// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_QOBJECT_ROS_H
#define QML_ROS_PLUGIN_QOBJECT_ROS_H

#include <QObject>

namespace qml_ros_plugin
{

/*!
 * Base class for QObjects that require ROS functionality.
 * Provides virtual methods that are called once ROS was initialized and once it was shutdown to enable initialization
 * and clean-up of of functionality that requires ROS.
 */
class QObjectRos : public QObject
{
Q_OBJECT
public:
  explicit QObjectRos( QObject *parent = nullptr );

  //! @return Whether or not this object is initialized.
  bool isInitialized() const;

protected:
  /*!
   * Called once ROS was initialized in this application.
   *
   * Override to perform initialization of functionality that depends on ROS.
   */
  virtual void onRosInitialized() { }

  /*!
   * Called once ROS or the application shuts down.
   *
   * Override to perform clean-up of functionality that depends on ROS, e.g., if the destruction order does not
   * guarantee destruction before required ROS objects are destructed.
   */
  virtual void onRosShutdown() { }

public slots:
  void _initialize();

  void _shutdown();

private:
  bool is_initialized_;
};
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_QOBJECT_ROS_H
