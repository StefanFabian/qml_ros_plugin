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

  Subscriber {
    id: mySubscriber
    topic: "/intval"
  }

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    Text {
      text: "Paste the following in a terminal and run it:"
    }
    TextInput {
      Layout.margins: 12
      readOnly: true
      selectByMouse: true
      text: "i=0; while [ 1 ]; do rostopic pub -1 /intval std_msgs/Int32 \"data: $i\"; sleep 1; let i++; done"
    }

    Text {
      text: "Received:\n" +
            "  Message type: " + mySubscriber.messageType + "\n" +
            "  Message content: " + (mySubscriber.message ? mySubscriber.message.data : "No message received yet") + "\n" +
            "  Running: " + mySubscriber.running
    }

    Button {
      id: activeButton
      state: "active"
      onClicked: {
        mySubscriber.running = !mySubscriber.running
        state = state == "active" ? "paused" : "active"
      }
      states: [
        State {
          name: "active"
          PropertyChanges {
            target: activeButton
            text: "Unsubscribe"
          }
        },
        State {
          name: "paused"
          PropertyChanges {
            target: activeButton
            text: "Subscribe"
          }
        }
      ]
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_subscriber_demo")
  }

}
