// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TIME_H
#define QML_ROS_PLUGIN_TIME_H

#include "qml_ros_plugin/qobject_ros.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

#include <QVariant>
#include <ros/time.h>

namespace qml_ros_plugin
{

template<typename RosTime>
class TimeWrapper
{
public:
  explicit TimeWrapper( const RosTime &time = RosTime()) : time_( time ) { }

  quint32 sec() const { return time_.sec; }

  void setSec( quint32 value ) { time_.sec = value; }

  quint32 nsec() const { return time_.nsec; }

  void setNSec( quint32 value ) { time_.nsec = value; }

  RosTime getRosTime() const { return time_; }

protected:
  RosTime time_;
};

/*!
 * Represents a point in time of the robot or simulation.
 *
 * Properties:
 *   - sec: unsigned integer containing the seconds passed since 1970
 *   - nsec: unsigned integer containing the nanoseconds since the last second
 */
class Time : public TimeWrapper<ros::Time>
{
Q_GADGET
  Q_PROPERTY( quint32 sec READ sec WRITE setSec )
  Q_PROPERTY( quint32 nsec READ nsec WRITE setNSec )
public:
  explicit Time( const ros::Time &time = ros::Time()) : TimeWrapper<ros::Time>( time ) { }

  //! The time in seconds (since 1970) as a decimal value. (Possible loss in precision)
  Q_INVOKABLE double toSec() const { return time_.toSec(); }

  //! The time in nanoseconds (since 1970) as an unsigned integer.
  Q_INVOKABLE quint64 toNSec() const { return time_.toNSec(); }

  //! Whether the time represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return time_.isZero(); }

  //!  A JS Date representing the value stored in this instance.
  //!  Since JS Dates only have millisecond accuracy, information about microseconds and nanoseconds are lost.
  //!  The time is always rounded down to prevent the JS Date from being in the future.
  Q_INVOKABLE QVariant toJSDate() const { return QVariant::fromValue( rosToQmlTime( time_ )); }
};


/*!
 * Represents a point in time of the current system.
 *
 * Properties:
 *   - sec: unsigned integer containing the seconds passed since 1970
 *   - nsec: unsigned integer containing the nanoseconds since the last second
 */
class WallTime : public TimeWrapper<ros::WallTime>
{
Q_GADGET
  Q_PROPERTY( quint32 sec READ sec WRITE setSec )
  Q_PROPERTY( quint32 nsec READ nsec WRITE setNSec )
public:
  explicit WallTime( const ros::WallTime &time = ros::WallTime()) : TimeWrapper<ros::WallTime>( time ) { }

  //! The time in seconds (since 1970) as a decimal value. (Possible loss in precision)
  Q_INVOKABLE double toSec() const { return time_.toSec(); }

  //! The time in nanoseconds (since 1970) as an unsigned integer.
  Q_INVOKABLE quint64 toNSec() const { return time_.toNSec(); }

  //! Whether the time represented by this instance is zero.
  Q_INVOKABLE bool isZero() const { return time_.isZero(); }

  //!  A JS Date representing the value stored in this instance.
  //!  Since JS Dates only have millisecond accuracy, information about microseconds and nanoseconds are lost.
  //!  The time is always rounded down to prevent the JS Date from being in the future.
  Q_INVOKABLE QVariant toJSDate() const { return QVariant::fromValue( rosToQmlTime( time_ )); }
};

class TimeSingleton : public QObjectRos
{
Q_OBJECT
public:
  /*!
   * Returns the ros::Time as Time.
   * This can be either the simulation time or the system time.
   *
   * Before the ros::Time got valid, this method returns a zero Time.
   */
  Q_INVOKABLE QVariant now();

  //! Creates a Time instance from the given time in seconds since 1970.
  Q_INVOKABLE QVariant create( double t );

  //! Creates a Time instance from the given time in seconds since 1970 and nanoseconds since the last full second.
  Q_INVOKABLE QVariant create( quint32 sec, quint32 nsec );

  //! Whether the time obtained using now() is the simulation time
  Q_INVOKABLE bool isSimTime();

  //! Whether the time obtained using now() is the system time
  Q_INVOKABLE bool isSystemTime();

  //! Whether the time obtained with now() is currently valid.
  Q_INVOKABLE bool isValid();
};

class WallTimeSingleton : public QObject
{
Q_OBJECT
public:
  /*!
   * Returns the ros::WallTime as WallTime.
   * This is always the system time.
   */
  Q_INVOKABLE QVariant now();

  //! Creates a WallTime instance from the given time in seconds since 1970.
  Q_INVOKABLE QVariant create( double t );

  //! Creates a WallTime instance from the given time in seconds since 1970 and nanoseconds since the last full second.
  Q_INVOKABLE QVariant create( quint32 sec, quint32 nsec );
};
}

// Register Time types
Q_DECLARE_METATYPE( qml_ros_plugin::Time );

Q_DECLARE_METATYPE( qml_ros_plugin::WallTime );

#endif //QML_ROS_PLUGIN_TIME_H
