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

  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12

    Text {
      id: timeText
      text: "Time: " + Time.now()
    }

    Text {
      
    }

    Text {
      id: wallTimeText
      text: "WallTime: " + WallTime.now()
    }

    Button {
      text: "Update"
      onClicked: {
        timeText.text = "Time: " + Time.now()
        wallTimeText.text = "WallTime: " + WallTime.now()
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
