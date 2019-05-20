// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_ARRAY_H
#define QML_ROS_PLUGIN_ARRAY_H

#include <QObject>
#include <QVariant>

#include <ros_babel_fish/message_types/array_message.h>
#include <ros_babel_fish/babel_fish.h>

namespace qml_ros_plugin
{

class Array : public QObject
{
Q_OBJECT
public:
  Array();

  Array( ros_babel_fish::TranslatedMessage::ConstPtr translated_message,
         const ros_babel_fish::ArrayMessageBase *message );

//  Array( const Array &other );

  ~Array() override = default;

  Q_INVOKABLE qulonglong length() const;

  Q_INVOKABLE QVariant get( uint index ) const;

  Q_INVOKABLE QVariant set( uint index, const QVariant &value );

private:
  ros_babel_fish::TranslatedMessage::ConstPtr translated_message_;
  const ros_babel_fish::ArrayMessageBase *message_;
  mutable QVariantList cache_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_ARRAY_H
