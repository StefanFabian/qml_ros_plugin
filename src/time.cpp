// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/time.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

#include <ros/time.h>
#include <ros/ros.h>

namespace qml_ros_plugin
{

QVariant Time::now()
{
  if ( !isInitialized() || !ros::Time::isValid())
    return QVariant();
  return QVariant::fromValue( rosToQmlTime( ros::Time::now()));
}

bool Time::isSimTime()
{
  return ros::Time::isSimTime();
}

bool Time::isSystemTime()
{
  return ros::Time::isSystemTime();
}

bool Time::isValid()
{
  return ros::Time::isValid();
}

QDateTime WallTime::now()
{
  return rosToQmlTime( ros::WallTime::now());
}
}
