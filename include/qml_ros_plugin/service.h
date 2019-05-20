// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_SERVICE_H
#define QML_ROS_PLUGIN_SERVICE_H

#include <QObject>
#include <QVariant>

#include <ros_babel_fish/babel_fish.h>

namespace qml_ros_plugin
{

class Service : public QObject
{
  Q_OBJECT
public:

  Q_INVOKABLE QVariant call(const QString &service, const QString &type, const QVariantMap &req);

protected:
  ros_babel_fish::BabelFish babel_fish_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_SERVICE_H
