// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_ACTION_CLIENT_H
#define QML_ROS_PLUGIN_ACTION_CLIENT_H

#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/qobject_ros.h"

#include <ros_babel_fish/actionlib/babel_fish_action.h>
#include <ros_babel_fish/babel_fish.h>

#include <QJSValue>
#include <QTimer>

namespace qml_ros_plugin
{
class NodeHandle;

class GoalHandle;

class ActionClient : public QObjectRos
{
  Q_OBJECT
  // @formatter:off
  //! True if the ActionClient is connected to the ActionServer, false otherwise.
  Q_PROPERTY( bool connected READ isServerConnected NOTIFY connectedChanged )
  //! The type of the action. Example: actionlib_tutorials/FibonacciAction
  Q_PROPERTY( QString actionType READ actionType CONSTANT )
  // @formatter:on
public:
  ActionClient( NodeHandle::Ptr nh, const QString &action_type, const QString &name );

  Q_INVOKABLE bool isServerConnected() const;

  QString actionType() const;

  /*!
   * Sends a goal to the action server if it is connected.
   *
   * @param goal The goal that is sent to the action server.
   * @param transition_cb A callback that is called on every client state transition.
   * @param feedback_cb A callback that is called whenever feedback for this goal is received.
   * @return null if the action server is not connected, otherwise a GoalHandle keeping track of the state of the goal.
   */
  Q_INVOKABLE QObject *sendGoal( const QVariantMap &goal, QJSValue transition_cb = QJSValue(),
                                 QJSValue feedback_cb = QJSValue() );

  //! Cancels all goals that are currently tracked by this client.
  Q_INVOKABLE void cancelAllGoals();

  //! Cancels all goals that were sent at and before the given ROS time by this client.
  //! Use Time.now() to obtain the current ROS time which can differ from the actual time.
  Q_INVOKABLE void cancelGoalsAtAndBeforeTime( const QDateTime &time );

signals:

  //! Emitted when the connected status changes, e.g., when the client connected to the server.
  void connectedChanged();

private slots:

  void checkServerConnected();

  void invokeTransitionCallback(
      QJSValue callback, actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle handle );

  void invokeFeedbackCallback(
      QJSValue callback, actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle handle,
      ros_babel_fish::BabelFishMessage::ConstPtr feedback );

private:
  void onRosInitialized() override;

  void onRosShutdown() override;

  ros_babel_fish::BabelFish babel_fish_;
  NodeHandle::Ptr nh_;
  QString action_type_;
  QString name_;
  std::shared_ptr<actionlib::ActionClient<ros_babel_fish::BabelFishAction>> client_;
  QTimer connect_timer_;
};
} // namespace qml_ros_plugin

Q_DECLARE_METATYPE( actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle );

Q_DECLARE_METATYPE( ros_babel_fish::BabelFishMessage::ConstPtr );

#endif // QML_ROS_PLUGIN_ACTION_CLIENT_H
