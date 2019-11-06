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

  GridLayout {
    anchors.fill: parent
    anchors.margins: 12
    columns: 2
    Text {
      text: "For this example,\n run the following in a separate terminal:"
    }

    TextInput {
      readOnly: true
      selectByMouse: true
      text: "rosrun roscpp_tutorials add_two_ints_server"
    }

    Text {
      Layout.alignment: Qt.AlignRight
      text: "A:"
    }

    SpinBox {
      id: inputA
      Layout.margins: 12
    }

    Text {
      Layout.alignment: Qt.AlignRight
      text: "B:"
    }

    SpinBox {
      id: inputB
      Layout.margins: 12
    }

    Button {
      id: activeButton
      state: "ready"
      onClicked: {
        state = "sending"
        var result = Service.call("/add_two_ints", "roscpp_tutorials/TwoInts",
                                  { a: inputA.value, b: inputB.value })
        textResult.text = !!result ? ("Result: " + result.sum) : "Failed"
        state = "ready"
      }
      states: [
        State {
          name: "ready"
          PropertyChanges {
            target: activeButton
            text: "Call Service"
          }
        },
        State {
          name: "sending"
          PropertyChanges {
            target: activeButton
            text: "Processing"
          }
        }
      ]
    }

    Text {
      id: textResult
      text: "No result yet"
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_service_demo")
  }

}
