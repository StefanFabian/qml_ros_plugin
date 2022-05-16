// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_TOPIC_INFO_H
#define QML_ROS_PLUGIN_TOPIC_INFO_H

#include <QArgument>
#include <QMetaType>
#include <QString>

namespace qml_ros_plugin
{

class TopicInfo
{
  Q_GADGET
  // @formatter:off
  //! The name of the topic, e.g., /front_camera/image_raw
  Q_PROPERTY( QString name READ name )
  //! The datatype of the topic, e.g., sensor_msgs/Image
  Q_PROPERTY( QString datatype READ datatype )
  // @formatter:on
public:
  TopicInfo() = default;

  TopicInfo( QString name, QString datatype )
      : name_( std::move( name ) ), datatype_( std::move( datatype ) )
  {
  }

  const QString &name() const { return name_; }

  const QString &datatype() const { return datatype_; }

private:
  QString name_;
  QString datatype_;
};
} // namespace qml_ros_plugin

Q_DECLARE_METATYPE( qml_ros_plugin::TopicInfo );

#endif // QML_ROS_PLUGIN_TOPIC_INFO_H
