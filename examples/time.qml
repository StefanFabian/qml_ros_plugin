import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import Ros 1.0

ApplicationWindow {
  id: page
  width: 1200
  height: 400

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros
    onShutdown: Qt.quit()
  }

  property var time: Time.now()
  property var wallTime: WallTime.now()

  GridLayout {
    anchors.fill: parent
    anchors.margins: 12
    columns: 2

    Text {      text: "Time"    }
    Text {      text: "WallTime"    }

    Text { text: "sec: " + page.time.sec }
    Text { text: "sec: " + page.wallTime.sec }

    Text { text: "nsec: " + page.time.nsec }
    Text { text: "nsec: " + page.wallTime.nsec }

    Text { text: "Date: " + page.time.toJSDate() }
    Text { text: "Date: " + page.wallTime.toJSDate() }

    Button {
      text: "Update"
      onClicked: {
        page.time = Time.now()
        page.wallTime = WallTime.now()
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_time_demo")
  }

}
