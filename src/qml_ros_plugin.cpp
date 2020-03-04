// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/console.h"
#include "qml_ros_plugin/image_transport_subscriber.h"
#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/publisher.h"
#include "qml_ros_plugin/ros.h"
#include "qml_ros_plugin/service.h"
#include "qml_ros_plugin/subscriber.h"
#include "qml_ros_plugin/tf_transform.h"
#include "qml_ros_plugin/tf_transform_listener.h"
#include "qml_ros_plugin/time.h"

#include <QtQml>
#include <QQmlExtensionPlugin>

namespace qml_ros_plugin
{

class QmlRosPlugin : public QQmlExtensionPlugin
{
Q_OBJECT
  // @formatter:off
  Q_PLUGIN_METADATA( IID QQmlExtensionInterface_iid )
  // @formatter:on
public:
  void registerTypes( const char *uri ) override
  {
    Q_ASSERT( uri == QLatin1String( "Ros" ));
    qmlRegisterType<qml_ros_plugin::Array>();
    qmlRegisterUncreatableMetaObject( ros_init_options::staticMetaObject, "Ros", 1, 0, "RosInitOptions",
                                      "Error: Can not create enum object." );
    qmlRegisterUncreatableMetaObject( ros_console_levels::staticMetaObject, "Ros", 1, 0, "RosConsoleLevels",
                                      "Error: Can not create enum object." );
    qmlRegisterType<Console>();
    qmlRegisterSingletonType<RosQmlSingletonWrapper>( "Ros", 1, 0, "Ros",
                                                      []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                                      {
                                                        Q_UNUSED( engine );
                                                        Q_UNUSED( scriptEngine );
                                                        return new RosQmlSingletonWrapper;
                                                      } );
    qmlRegisterType<NodeHandle>();
    qmlRegisterType<Publisher>();

    qmlRegisterType<Subscriber>( "Ros", 1, 0, "Subscriber" );
    qmlRegisterSingletonType<Service>( "Ros", 1, 0, "Service",
                                       []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                       {
                                         Q_UNUSED( engine );
                                         Q_UNUSED( scriptEngine );
                                         return new Service;
                                       } );
    qmlRegisterSingletonType<TfTransformListenerWrapper>( "Ros", 1, 0, "TfTransformListener",
                                                          []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                                          {
                                                            Q_UNUSED( engine );
                                                            Q_UNUSED( scriptEngine );
                                                            return new TfTransformListenerWrapper;
                                                          } );
    qmlRegisterType<TfTransform>( "Ros", 1, 0, "TfTransform" );
    qmlRegisterSingletonType<Time>( "Ros", 1, 0, "Time",
                                    []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                    {
                                      Q_UNUSED( engine );
                                      Q_UNUSED( scriptEngine );
                                      return new Time();
                                    } );
    qmlRegisterSingletonType<WallTime>( "Ros", 1, 0, "WallTime",
                                        []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                        {
                                          Q_UNUSED( engine );
                                          Q_UNUSED( scriptEngine );
                                          return new WallTime();
                                        } );

    // Image transport
    qmlRegisterType<ImageTransportSubscriber>( "Ros", 1, 0, "ImageTransportSubscriber" );
  }
};
}

#include "qml_ros_plugin.moc"
