import QtQuick 2.3
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2
import QtQuick.Layouts 1.0
import QtMultimedia 5.4
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
  
  ImageTransportSubscriber {
    id: imageSubscriber
    // Enter a valid image topic here
    topic: "/front_rgbd_cam/color/image_rect_color"
    // This is the default transport, change if compressed is not available
    defaultTransport: "compressed"
  }

  VideoOutput {
    anchors.fill: parent
    // Can be used in increments of 90 to rotate the video
    orientation: 90
    source: imageSubscriber
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] and the name to use those
    // args instead of the ones supplied by the command line.
    Ros.init("qml_image_subscriber_demo")
  }

}
