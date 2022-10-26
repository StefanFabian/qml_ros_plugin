// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/service.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"

#include <QJSEngine>
#include <thread>

using namespace ros_babel_fish;
using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{
Service::Service() : id_counter_( 0 ) { babel_fish_ = BabelFishDispenser::getBabelFish(); }

QVariant Service::call( const QString &service, const QString &type, const QVariantMap &req )
{
  try {
    Message::Ptr message = babel_fish_.createServiceRequest( type.toStdString() );
    fillMessage( *message, req );
    TranslatedMessage::Ptr response;
    bool result = babel_fish_.callService( service.toStdString(), message, response );
    if ( !result )
      return QVariant( false );
    if ( response->input_message->size() == 0 )
      return QVariant( true );
    return msgToMap( response, *response->translated_message );
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to call service: %s", ex.what() );
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to call service: %s", ex.what() );
  }
  return QVariant( false );
}

void Service::callAsync( const QString &service, const QString &type, const QVariantMap &req,
                         const QJSValue &callback )
{
  std::string std_service = service.toStdString();
  Message::Ptr message;
  try {
    message = babel_fish_.createServiceRequest( type.toStdString() );
    fillMessage( *message, req );
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to call service: %s", ex.what() );
    QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                               Q_ARG( QJSValue, callback ), Q_ARG( QVariant, QVariant( false ) ) );
    return;
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to call service: %s", ex.what() );
    QMetaObject::invokeMethod( this, "invokeCallback", Qt::AutoConnection,
                               Q_ARG( QJSValue, callback ), Q_ARG( QVariant, QVariant( false ) ) );
    return;
  }
  // Insert the callback into a container and pass the id to the service call thread
  // Passing the callback to the thread would lead to random segfaults because the QJSValue is not
  // supposed to leave the thread of the QJSEngine
  int id = ++id_counter_;
  callbacks_.insert( id, callback );
  std::thread thread( [std_service, this, message, id]() -> void {
    TranslatedMessage::Ptr response;
    bool result = babel_fish_.callService( std_service, message, response );
    if ( !result ) {
      QMetaObject::invokeMethod( this, "invokeCallback", Qt::QueuedConnection, Q_ARG( int, id ),
                                 Q_ARG( QVariant, QVariant( false ) ) );
      return;
    }
    if ( response->input_message->size() == 0 ) {
      QMetaObject::invokeMethod( this, "invokeCallback", Qt::QueuedConnection, Q_ARG( int, id ),
                                 Q_ARG( QVariant, QVariant( true ) ) );
      return;
    }
    QMetaObject::invokeMethod(
        this, "invokeCallback", Qt::QueuedConnection, Q_ARG( int, id ),
        Q_ARG( QVariant, msgToMap( response, *response->translated_message ) ) );
  } );
  thread.detach();
}

void Service::invokeCallback( int id, QVariant result )
{
  QJSValue callback = callbacks_[id];
  callbacks_.remove( id );
  if ( !callback.isCallable() )
    return;
  QJSEngine *engine = qjsEngine( this );
  callback.call( { engine->toScriptValue( result ) } );
}
} // namespace qml_ros_plugin
