// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/array.h"
#include "qml_ros_plugin/service.h"
#include "qml_ros_plugin/subscriber.h"
#include "qml_ros_plugin/tf_transform.h"
#include "qml_ros_plugin/tf_transform_listener.h"

#include <QtQml>
#include <QQmlExtensionPlugin>

namespace qml_ros_plugin
{

class QmlRosPlugin : public QQmlExtensionPlugin
{
Q_OBJECT
  Q_PLUGIN_METADATA( IID
                       QQmlExtensionInterface_iid )
public:
  void registerTypes( const char *uri ) override
  {
    Q_ASSERT( uri == QLatin1String( "RosPlugin" ));
//    qRegisterMetaType<qml_ros_plugin::Array>();
    qmlRegisterType<qml_ros_plugin::Array>();
    qmlRegisterType<Subscriber>( "RosPlugin", 1, 0, "Subscriber" );
    qmlRegisterSingletonType<Service>( "RosPlugin", 1, 0, "Service",
                                                   []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                                   {
                                                     Q_UNUSED( engine );
                                                     Q_UNUSED( scriptEngine );
                                                     return new Service;
                                                   } );
    qmlRegisterSingletonType<TfTransformListener>( "RosPlugin", 1, 0, "TfTransformListener",
                                                   []( QQmlEngine *engine, QJSEngine *scriptEngine ) -> QObject *
                                                   {
                                                     Q_UNUSED( engine );
                                                     Q_UNUSED( scriptEngine );
                                                     return new TfTransformListenerWrapper;
                                                   } );
    qmlRegisterType<TfTransform>( "RosPlugin", 1, 0, "TfTransform" );
  }
};

}

#include "qml_ros_plugin.moc"
