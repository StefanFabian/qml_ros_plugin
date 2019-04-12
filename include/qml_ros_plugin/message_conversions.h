// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H
#define QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H

#include <QVariantMap>
#include <geometry_msgs/TransformStamped.h>

namespace qml_ros_plugin
{

QVariantMap msgToMap(const std_msgs::Header &msg);

QVariantMap msgToMap(const geometry_msgs::Vector3 &msg);

QVariantMap msgToMap(const geometry_msgs::Quaternion &msg);

QVariantMap msgToMap(const geometry_msgs::Transform &msg);

QVariantMap msgToMap(const geometry_msgs::TransformStamped &msg);

} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_MESSAGE_CONVERSIONS_H
