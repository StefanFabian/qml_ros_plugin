// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/action_client.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/goal_handle.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/qml_ros_conversion.h"

#include <QJSEngine>

using namespace ros_babel_fish;
using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{

ActionClient::ActionClient( NodeHandle::Ptr nh, const QString &action_type, const QString &name )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  nh_ = std::move( nh );
  action_type_ = action_type;
  name_ = name;
}

void ActionClient::onRosInitialized()
{
  if ( !action_type_.endsWith( "Action" ) ) {
    ROS_ERROR_NAMED( "qml_ros_plugin",
                     "The action type did not end with 'Action'. Are you sure '%s' is an action?",
                     action_type_.toStdString().c_str() );
    return;
  }
  std::string action_goal_type = action_type_.toStdString() + "Goal";
  auto description = babel_fish_.descriptionProvider()->getMessageDescription( action_goal_type );
  if ( description == nullptr ) {
    ROS_ERROR_NAMED(
        "qml_ros_plugin", "Failed to look up action message of type '%s'. ActionClient not initialized and will not connect.",
        action_goal_type.c_str() );
    return;
  }
  try {
    client_ = std::make_shared<actionlib::ActionClient<BabelFishAction>>(
        nh_->nodeHandle(), description, name_.toStdString() );
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Could not create ActionClient: %s", ex.what() );
    client_ = nullptr;
    return;
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Could not create ActionClient: %s", ex.what() );
    client_ = nullptr;
    return;
  }
  connect_timer_.setInterval( 16 );
  connect( &connect_timer_, &QTimer::timeout, this, &ActionClient::checkServerConnected );
  connect_timer_.start();
}

void ActionClient::onRosShutdown() { client_.reset(); }

bool ActionClient::isServerConnected() const
{
  return client_ != nullptr && client_->isServerConnected();
}

QString ActionClient::actionType() const { return action_type_; }

void ActionClient::checkServerConnected()
{
  if ( !isServerConnected() )
    return;
  connect_timer_.stop();
  disconnect( &connect_timer_, &QTimer::timeout, this, &ActionClient::checkServerConnected );
  emit connectedChanged();
}

void ActionClient::invokeTransitionCallback(
    QJSValue callback, actionlib::ActionClient<BabelFishAction>::GoalHandle handle )
{
  QJSEngine *engine = qjsEngine( this );
  QJSValue js_goal_handle = engine->newQObject( new GoalHandle( client_, handle ) );
  callback.call( { js_goal_handle } );
}

void ActionClient::invokeFeedbackCallback(
    QJSValue callback, actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle handle,
    ros_babel_fish::BabelFishMessage::ConstPtr feedback )
{
  QJSEngine *engine = qjsEngine( this );
  QJSValue js_goal_handle = engine->newQObject( new GoalHandle( client_, handle ) );
  try {
    TranslatedMessage::ConstPtr translated_feedback = babel_fish_.translateMessage( feedback );
    QJSValue js_feedback = engine->toScriptValue<QVariant>( msgToMap( translated_feedback ) );
    callback.call( { js_goal_handle, js_feedback } );
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to translate Action feedback: %s", ex.what() );
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to translate Action feedback: %s", ex.what() );
  }
}

QObject *ActionClient::sendGoal( const QVariantMap &goal, QJSValue transition_cb,
                                 QJSValue feedback_cb )
{
  if ( client_ == nullptr ) {
    ROS_ERROR( "Tried to send goal when ActionClient was not connected!" );
    return nullptr;
  }

  std::string goal_type = action_type_.toStdString();
  goal_type = goal_type.substr( 0, goal_type.length() - strlen( "Action" ) ) + "Goal";
  try {
    Message::Ptr message = babel_fish_.createMessage( goal_type );
    if ( message == nullptr )
      return nullptr;
    if ( !fillMessage( *message, goal ) )
      return nullptr;
    BabelFishMessage::Ptr bf_message = babel_fish_.translateMessage( message );
    auto goal_handle = client_->sendGoal(
        *bf_message,
        [transition_cb,
         this]( const actionlib::ActionClient<BabelFishAction>::GoalHandle &goal_handle ) mutable {
          if ( !transition_cb.isCallable() )
            return;
          // Make sure this is called on the main thread, because the objects created in this method
          // have to be on the same thread as QQmlEngine or there may be random crashes
          QMetaObject::invokeMethod(
              this, "invokeTransitionCallback", Qt::AutoConnection, Q_ARG( QJSValue, transition_cb ),
              Q_ARG( actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle,
                     goal_handle ) );
        },
        [feedback_cb, this]( const actionlib::ActionClient<BabelFishAction>::GoalHandle &goal_handle,
                             const BabelFishMessage::ConstPtr &feedback ) mutable {
          if ( !feedback_cb.isCallable() )
            return;
          QMetaObject::invokeMethod(
              this, "invokeFeedbackCallback", Qt::AutoConnection, Q_ARG( QJSValue, feedback_cb ),
              Q_ARG( actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle,
                     goal_handle ),
              Q_ARG( ros_babel_fish::BabelFishMessage::ConstPtr, feedback ) );
        } );
    return new GoalHandle( client_, goal_handle );
  } catch ( BabelFishMessageException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to send Action goal: %s", ex.what() );
  } catch ( BabelFishException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Failed to send Action goal: %s", ex.what() );
  }
  return nullptr;
}

void ActionClient::cancelAllGoals()
{
  if ( client_ == nullptr )
    return;
  client_->cancelAllGoals();
}

void ActionClient::cancelGoalsAtAndBeforeTime( const QDateTime &time )
{
  if ( client_ == nullptr )
    return;
  client_->cancelGoalsAtAndBeforeTime( qmlToRosTime( time ) );
}
} // namespace qml_ros_plugin
