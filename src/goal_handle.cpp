// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/goal_handle.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"

using namespace ros_babel_fish;
using namespace qml_ros_plugin::conversion;

namespace qml_ros_plugin
{

GoalHandle::GoalHandle(
    std::shared_ptr<actionlib::ActionClient<ros_babel_fish::BabelFishAction>> client,
    const actionlib::ActionClient<ros_babel_fish::BabelFishAction>::GoalHandle &handle )
    : client_( std::move( client ) ), goal_handle_( handle )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
}

void GoalHandle::cancel() { goal_handle_.cancel(); }

bool GoalHandle::expired() const { return goal_handle_.isExpired(); }

void GoalHandle::resend() { goal_handle_.resend(); }

qml_ros_plugin::action_comm_states::CommState GoalHandle::commState() const
{
  return static_cast<qml_ros_plugin::action_comm_states::CommState>(
      goal_handle_.getCommState().state_ );
}

TerminalState GoalHandle::terminalState() const
{
  const actionlib::TerminalState &state = goal_handle_.getTerminalState();
  return { static_cast<action_terminal_states::TerminalState>( state.state_ ),
           QString::fromStdString( state.text_ ) };
}

QVariant GoalHandle::getResult()
{
  BabelFishMessage::ConstPtr result = goal_handle_.getResult();
  if ( result == nullptr )
    return {};
  TranslatedMessage::Ptr translated = babel_fish_.translateMessage( result );
  return msgToMap( translated, translated->translated_message->as<CompoundMessage>() );
}
} // namespace qml_ros_plugin
