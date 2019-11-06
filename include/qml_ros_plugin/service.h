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
  Service();

  /*!
   * Calls a service and returns the result.
   *
   * @param service The service topic.
   * @param type The type of the service, e.g., "roscpp_tutorials/TwoInts"
   * @param req The service request, i.e., a filled request message of the service type, e.g., "roscpp_tutorials/TwoIntsRequest"
   * @return False, if request was not successful, true if the response message is empty and the translated service
   *   response, e.g., "roscpp_tutorials/TwoIntsResponse", otherwise.
   */
  Q_INVOKABLE QVariant call( const QString &service, const QString &type, const QVariantMap &req );

protected:
  ros_babel_fish::BabelFish babel_fish_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_SERVICE_H
