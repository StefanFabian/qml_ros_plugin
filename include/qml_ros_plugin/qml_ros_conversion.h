// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_QML_ROS_CONVERSION_H
#define QML_ROS_PLUGIN_QML_ROS_CONVERSION_H

#include <QDateTime>
#include <ros/duration.h>
#include <ros/time.h>

namespace qml_ros_plugin
{

inline ros::Duration qmlToRosDuration( double milliseconds )
{
  return ros::Duration( milliseconds * 1E-3 );
}

inline ros::Time qmlToRosTime( double milliseconds_since_epoch )
{
  return ros::Time( milliseconds_since_epoch * 1E-3 );
}

inline ros::Time qmlToRosTime( const QDateTime &time )
{
  if ( !time.isValid()) return ros::Time( 0 );
  qint64 msecs = time.toMSecsSinceEpoch();
  uint32_t secs = msecs / 1000;
  return { secs, static_cast<uint32_t>(msecs - 1000 * secs) * 1000 * 1000 };
}

inline double rosToQmlDuration( const ros::Duration &duration )
{
  return duration.sec * 1E3 + duration.nsec * 1E-6;
}

inline QDateTime rosToQmlTime( const ros::Time &time )
{
  // Always round down because otherwise high precision stuff like tf might fail due to, e.g., look up into future
  return QDateTime::fromMSecsSinceEpoch( time.sec * 1E3 + time.nsec * 1E-6 );
}

inline QDateTime rosToQmlTime( const ros::WallTime &time )
{
  return QDateTime::fromMSecsSinceEpoch( time.sec * 1E3 + time.nsec * 1E-6 );
}
}

#endif //QML_ROS_PLUGIN_QML_ROS_CONVERSION_H
