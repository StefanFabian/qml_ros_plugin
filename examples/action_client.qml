import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import Ros 1.0

ApplicationWindow {
  id: page
  width: 620
  height: 400

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros
    onShutdown: Qt.quit()
  }

  // Arguments are: Message Type, Topic, Queue Size
  property var fibonacciClient: Ros.createActionClient("actionlib_tutorials/FibonacciAction", "fibonacci")
  property var goal_handle
  property string status: "DONE"
  property string terminalStatus: "Not in DONE yet"
  property string terminalStatusText: ""
  property string feedback: "[]"
  property string result: "[]"

  function updateStatus(handle) {
    switch (handle.commState) {
      case ActionCommStates.WAITING_FOR_GOAL_ACK:
        status = "WAITING_FOR_GOAL_ACK"
        break
      case ActionCommStates.PENDING:
        status = "PENDING"
        break
      case ActionCommStates.ACTIVE:
        status = "ACTIVE"
        break
      case ActionCommStates.WAITING_FOR_RESULT:
        status = "WAITING_FOR_RESULT"
        break
      case ActionCommStates.WAITING_FOR_CANCEL_ACK:
        status = "WAITING_FOR_CANCEL_ACK"
        break
      case ActionCommStates.RECALLING:
        status = "RECALLING"
        break
      case ActionCommStates.PREEMPTING:
        status = "PREEMPTING"
        break
      case ActionCommStates.DONE:
        status = "DONE"
        // The terminalState should only be used if the comm state is DONE.
        switch (handle.terminalState.state) {
          case ActionTerminalStates.RECALLED:
            terminalStatus = "RECALLED"
            break
          case ActionTerminalStates.REJECTED:
            terminalStatus = "REJECTED"
            break
          case ActionTerminalStates.PREEMPTED:
            terminalStatus = "PREEMPTED"
            break
          case ActionTerminalStates.ABORTED:
            terminalStatus = "ABORTED"
            break
          case ActionTerminalStates.SUCCEEDED:
            terminalStatus = "SUCCEEDED"
            break
          case ActionTerminalStates.LOST:
            terminalStatus = "LOST"
            break
        }
        // Empty in the example but that's how you'd get the goal text
        terminalStatusText = handle.terminalState.text
        break
    }
  }

  GridLayout {
    columns: 2
    anchors.fill: parent
    anchors.margins: 12
    Text {
      Layout.fillWidth: true
      Layout.columnSpan: 2
      text: "Enter an integer to send it to the fibonacci tutorial action server:"
    }

    SpinBox {
      id: numberInput
      Layout.margins: 12
    }

    GridLayout {
      Layout.fillWidth: true
      Layout.preferredWidth: parent.width * 2 / 3
      columns: 2
      Text {
        text: "Status:"
      }
      Text {
        text: status || ""
      }
      Text {
        text: "Terminal Status:"
      }
      Text {
        text: terminalStatus || ""
      }
      Text {
        text: "Terminal Status Text:"
      }
      Text {
        text: terminalStatusText || ""
      }
      Text {
        text: "Feedback:"
      }
      Text {
        Layout.fillWidth: true
        text: feedback || ""
      }
      Text {
        text: "Result:"
      }
      Text {
        text: result || ""
      }
    }

    Button {
      id: activeButton
      state: "ready"
      onClicked: {
        if (state == "sending") {
          goal_handle.cancel();
          return
        }
        state = "sending"
        terminalStatus = "Not in DONE yet"
        terminalStatusText = ""
        goal_handle = fibonacciClient.sendGoal({ order: numberInput.value }, function (goal_handle) {
          updateStatus(goal_handle)
          if (goal_handle.commState == ActionCommStates.DONE) {
            activeButton.state = "ready"
            page.result = "["
            var result = goal_handle.getResult()
            for (var i = 0; i < result.sequence.length; ++i) {
              page.result += result.sequence.at(i)
              if (i != result.sequence.length - 1) page.result += ", "
            }
            page.result += "]"
          }
        }, function (goal_handle, feedback) {
          var newFeedback = "["
          for (var i = 0; i < feedback.sequence.length; ++i)
            newFeedback += feedback.sequence.at(i) + (i == feedback.sequence.length - 1 ? "]" : ", ")
          page.feedback = newFeedback
        })
      }
      states: [
        State {
          name: "ready"
          PropertyChanges {
            target: activeButton
            text: "Send"
          }
        },
        State {
          name: "sending"
          PropertyChanges {
            target: activeButton
            text: "Cancel"
          }
        }
      ]
    }

    Text {
      text: "Is connected: " + (fibonacciClient.connected ? "true" : "false")
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_action_client_demo")
  }
}
