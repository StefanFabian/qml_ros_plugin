// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_SERVICE_H
#define QML_ROS_PLUGIN_SERVICE_H

#include <QJSValue>
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

  /*!
   * Calls a service asynchronously returning immediately.
   * Once the service call finishes, the optional callback is called with the result if provided.
   *
   * @param service The service topic.
   * @param type The type of the service, e.g., "roscpp_tutorials/TwoInts"
   * @param req The service request, i.e., a filled request message of the service type, e.g., "roscpp_tutorials/TwoIntsRequest"
   * @param callback The callback that is called once the service has finished.
   */
  Q_INVOKABLE void callAsync( const QString &service, const QString &type, const QVariantMap &req, const QJSValue &callback = QJSValue());

private slots:
  void invokeCallback(QJSValue value, QVariant result);

private:
  ros_babel_fish::BabelFish babel_fish_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_SERVICE_H
