// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H
#define QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H

#include <QVariantMap>
#include <geometry_msgs/TransformStamped.h>
#include <ros_babel_fish/babel_fish.h>

namespace qml_ros_plugin
{

/*!
 * @brief Converts between QVariant and ROS messages.
 *
 * Most types have a clear correspondence, e.g., all int fields smaller or equal to 32bit are mapped to int, all uint
 * smaller or equal to 32bit are mapped to uint. Larger are mapped to qlonglong and qulonglong (64bit).
 *
 * Noteworthy, however, are ros::Time and ros::Duration which are somewhat of a special case.
 * Whereas ros::Time is mapped to QDateTime, the difference of two datetimes in JS and QML is stored as double
 * representing the milliseconds.
 */
namespace conversion
{
template<typename T>
T &obtainValueAsReference( QVariant &value ) { return *reinterpret_cast<T *>(value.data()); }

template<typename T>
const T &obtainValueAsConstReference( const QVariant &value ) { return *reinterpret_cast<const T *>(value.data()); }

QVariantMap msgToMap( const std_msgs::Header &msg );

QVariantMap msgToMap( const geometry_msgs::Vector3 &msg );

QVariantMap msgToMap( const geometry_msgs::Quaternion &msg );

QVariantMap msgToMap( const geometry_msgs::Transform &msg );

QVariantMap msgToMap( const geometry_msgs::TransformStamped &msg );

QVariant msgToMap( const ros_babel_fish::TranslatedMessage::ConstPtr &msg );

QVariant msgToMap( const ros_babel_fish::TranslatedMessage::ConstPtr &storage, const ros_babel_fish::Message &msg );

bool fillMessage( ros_babel_fish::Message &msg, const QVariant &value );

bool fillMessage( ros_babel_fish::BabelFish &fish, ros_babel_fish::Message &msg, const QVariant &value );
}
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H
