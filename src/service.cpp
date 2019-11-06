// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/service.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"

using namespace ros_babel_fish;
using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{
Service::Service()
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

QVariant Service::call( const QString &service, const QString &type, const QVariantMap &req )
{
  Message::Ptr message = babel_fish_.createServiceRequest( type.toStdString());
  fillMessage( *message, req );
  TranslatedMessage::Ptr response;
  bool result = babel_fish_.callService( service.toStdString(), message, response );
  if ( !result ) return QVariant( false );
  if ( response->input_message->size() == 0 ) return QVariant( true );
  return msgToMap( response, *response->translated_message );
}
} // qml_ros_plugin
