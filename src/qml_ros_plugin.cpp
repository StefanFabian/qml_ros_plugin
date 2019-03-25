//
// Created by Stefan Fabian on 25.03.19.
//

#include "qml_ros_plugin/subscriber.h"

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
    qmlRegisterType<Subscriber>( "RosPlugin", 1, 0, "Subscriber" );
  }
};

}

#include "qml_ros_plugin.moc"
