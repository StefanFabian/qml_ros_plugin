// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/qobject_ros.h"
#include "qml_ros_plugin/ros.h"

#include <QCoreApplication>

namespace qml_ros_plugin
{

QObjectRos::QObjectRos( QObject *parent ) : QObject( parent ), is_initialized_( false )
{
  if ( RosQml::getInstance().isInitialized())
  {
    // Invoke initialize after object was constructed
    QMetaObject::invokeMethod( this, "_initialize", Qt::QueuedConnection );
  }
  else
  {
    QObject::connect( &RosQml::getInstance(), &RosQml::initialized, this, &QObjectRos::_initialize );
  }
  // These allow for safe clean-up if the application exits since the order of the singleton destructors is undefined
  // and this might lead to dependency issues.
  QObject::connect( &RosQml::getInstance(), &RosQml::shutdown, this, &QObjectRos::_shutdown );
  QObject::connect( QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this, &QObjectRos::_shutdown );
  // This object needs ROS communication, so it's safe to assume it wants the spinner to be running during its lifetime.
  RosQml::getInstance().startSpinning();
}

QObjectRos::~QObjectRos()
{
  RosQml::getInstance().stopSpinning();
}

bool QObjectRos::isInitialized() const { return is_initialized_; }

void QObjectRos::_initialize()
{
  if ( is_initialized_ ) return;
  onRosInitialized();
  is_initialized_ = true;
}

void QObjectRos::_shutdown()
{
  if ( !is_initialized_ ) return;
  onRosShutdown();
  is_initialized_ = false;
}
} // qml_ros_plugin
