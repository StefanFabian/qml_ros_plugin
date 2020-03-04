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
    onShutdown: {
      Qt.quit()
    }
  }

  ButtonGroup {
    id: outputLoggingLevelGroup
    onCheckedButtonChanged: {
        switch (checkedButton.text) {
          case 'Debug':
            Ros.console.setLoggerLevel(Ros.console.defaultName, RosConsoleLevels.Debug);
            break;
          case 'Info':
            Ros.console.setLoggerLevel(Ros.console.defaultName, RosConsoleLevels.Info);
            break;
          case 'Warn':
            Ros.console.setLoggerLevel(Ros.console.defaultName, RosConsoleLevels.Warn);
            break;
          case 'Error':
            Ros.console.setLoggerLevel(Ros.console.defaultName, RosConsoleLevels.Error);
            break;
          case 'Fatal':
            Ros.console.setLoggerLevel(Ros.console.defaultName, RosConsoleLevels.Fatal);
            break;
        }
    }
  }
  ButtonGroup { id: messageLoggingLevelGroup }

  GridLayout {
    anchors.fill: parent
    columns: 5

    // Output logging level
    Text {
      Layout.columnSpan: parent.columns
      text: "Output logging level"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Debug"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Info"
      checked: true
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Warn"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Error"
    }

    RadioButton {
      ButtonGroup.group: outputLoggingLevelGroup
      text: "Fatal"
    }

    // Message logging level
    Text {
      Layout.columnSpan: parent.columns
      text: "Message logging level"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Debug"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Info"
      checked: true
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Warn"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Error"
    }

    RadioButton {
      ButtonGroup.group: messageLoggingLevelGroup
      text: "Fatal"
    }


    TextField {
      id: logMessageField
      Layout.columnSpan: 4
      Layout.fillWidth: true
      Layout.margins: 4
      placeholderText: "Enter a message to log"
    }

    Button {
      text: "Log"

      onClicked: {
        switch (messageLoggingLevelGroup.checkedButton.text) {
          case 'Debug':
            Ros.debug(logMessageField.text);
            break;
          case 'Info':
            Ros.info(logMessageField.text);
            break;
          case 'Warn':
            Ros.warn(logMessageField.text);
            break;
          case 'Error':
            Ros.error(logMessageField.text);
            break;
          case 'Fatal':
            Ros.fatal(logMessageField.text);
            break;
          default:
            Ros.fatal("Unexpected logging level")
        }
      }
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_logging_demo")
  }

}
