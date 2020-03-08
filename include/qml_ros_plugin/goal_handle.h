// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_GOAL_HANDLE_H
#define QML_ROS_PLUGIN_GOAL_HANDLE_H

#include "qml_ros_plugin/qobject_ros.h"

#include <ros_babel_fish/actionlib/babel_fish_action.h>
#include <ros_babel_fish/babel_fish.h>

#include <utility>

namespace qml_ros_plugin
{

namespace action_comm_states
{
Q_NAMESPACE

enum CommState
{
  WAITING_FOR_GOAL_ACK = actionlib::CommState::WAITING_FOR_GOAL_ACK,
  PENDING = actionlib::CommState::PENDING,
  ACTIVE = actionlib::CommState::ACTIVE,
  WAITING_FOR_RESULT = actionlib::CommState::WAITING_FOR_RESULT,
  WAITING_FOR_CANCEL_ACK = actionlib::CommState::WAITING_FOR_CANCEL_ACK,
  RECALLING = actionlib::CommState::RECALLING,
  PREEMPTING = actionlib::CommState::PREEMPTING,
  DONE = actionlib::CommState::DONE
};

Q_ENUM_NS( CommState )
}

namespace action_terminal_states
{
Q_NAMESPACE

enum TerminalState
{
  RECALLED = actionlib::TerminalState::RECALLED,
  REJECTED = actionlib::TerminalState::REJECTED,
  PREEMPTED = actionlib::TerminalState::PREEMPTED,
  ABORTED = actionlib::TerminalState::ABORTED,
  SUCCEEDED = actionlib::TerminalState::SUCCEEDED,
  LOST = actionlib::TerminalState::LOST,

  UNKNOWN
};

Q_ENUM_NS( TerminalState )
}

class TerminalState
{
Q_GADGET
  // @formatter:off
  //! The terminal state in form of an ActionTerminalStates enum value:
  //! RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
  Q_PROPERTY( qml_ros_plugin::action_terminal_states::TerminalState state READ state )
  //! An optional text that was returned by the ActionServer in combination with the terminal state.
  Q_PROPERTY( QString text READ text )
  // @formatter:on
public:
  TerminalState() : state_( action_terminal_states::UNKNOWN ) { }

  TerminalState( action_terminal_states::TerminalState state, QString text )
    : text_( std::move( text )), state_( state ) { }

  const QString &text() const { return text_; }

  qml_ros_plugin::action_terminal_states::TerminalState state() const { return state_; }

private:

  QString text_;
  action_terminal_states::TerminalState state_;
};

class GoalHandle : public QObjectRos
{
Q_OBJECT
  // @formatter:off
  //! True if this handle is not tracking a goal, false otherwise
  Q_PROPERTY( bool expired READ expired )
  //! The terminal state of this goal. Only valid if commstate == ActionCommStates.DONE.
  Q_PROPERTY( qml_ros_plugin::TerminalState terminalState READ terminalState )
  //! The state of this goal's communication state machine.
  //! Possible states:
  //! WAITING_FOR_GOAL_ACK, PENDING, ACTIVE, WAITING_FOR_RESULT, WAITING_FOR_CANCEL_ACK, RECALLING, PREEMPTING, DONE
  Q_PROPERTY( qml_ros_plugin::action_comm_states::CommState commState READ commState )
  // @formatter:on
public:
  explicit GoalHandle( std::shared_ptr<actionlib::ActionClient<ros_babel_fish::BabelFishAction>> client,
                       const actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle &handle );

  bool expired() const;

  qml_ros_plugin::TerminalState terminalState() const;

  qml_ros_plugin::action_comm_states::CommState commState() const;

  //! Sends a cancellation request to the ActionServer. Transitions to WAITING_FOR_CANCEL_ACK.
  Q_INVOKABLE void cancel();

  //! Resends this goal with the same id to the ActionServer.
  //! Useful if you have reason to think that the goal was lost in transit.
  Q_INVOKABLE void resend();

  //! Can be used to obtain the result returned by the ActionServer. Empty if no result received.
  Q_INVOKABLE QVariant getResult();

private:
  ros_babel_fish::BabelFish babel_fish_;
  // Store the client to make sure its destructed after the goal handles
  std::shared_ptr<actionlib::ActionClient<ros_babel_fish::BabelFishAction>> client_;
  actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle goal_handle_;
};
}

Q_DECLARE_METATYPE( qml_ros_plugin::action_comm_states::CommState );

Q_DECLARE_METATYPE( qml_ros_plugin::TerminalState );

Q_DECLARE_METATYPE( qml_ros_plugin::action_terminal_states::TerminalState );


#endif //QML_ROS_PLUGIN_GOAL_HANDLE_H
