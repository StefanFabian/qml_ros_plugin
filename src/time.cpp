// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/time.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

namespace qml_ros_plugin
{

// ============== TIME SINGLETON ==============
QVariant TimeSingleton::now()
{
  if ( !ros::Time::isValid())
    return QVariant::fromValue( Time());
  return QVariant::fromValue( Time( ros::Time::now()));
}

QVariant TimeSingleton::create( double t )
{
  return QVariant::fromValue( Time( ros::Time( t )));
}

QVariant TimeSingleton::create( quint32 sec, quint32 nsec )
{
  return QVariant::fromValue( Time( ros::Time( sec, nsec )));
}

bool TimeSingleton::isSimTime()
{
  return ros::Time::isSimTime();
}

bool TimeSingleton::isSystemTime()
{
  return ros::Time::isSystemTime();
}

bool TimeSingleton::isValid()
{
  return ros::Time::isValid();
}

QVariant WallTimeSingleton::now()
{
  return QVariant::fromValue( WallTime( ros::WallTime::now()));
}

QVariant WallTimeSingleton::create( double t )
{
  return QVariant::fromValue( WallTime( ros::WallTime( t )));
}

QVariant WallTimeSingleton::create( quint32 sec, quint32 nsec )
{
  return QVariant::fromValue( WallTime( ros::WallTime( sec, nsec )));
}
}
