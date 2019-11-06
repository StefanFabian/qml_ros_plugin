# Examples
Examples on how to use the QML ROS Plugin

| Dependencies | |
| --- | --- |
| qmlscene | `sudo apt install qmlscene`|
| QtQuick 2 | `sudo apt install qml-module-qtquick2` |
| QtQuick.Controls 2 | `sudo apt install qml-module-qtquick-controls2` |
| QtQuick.Layouts | `sudo apt install qml-module-qtquick-layouts` |
| QtQuick.Window 2 | `sudo apt install qml-module-qtquick-window2` |

Run using: `qmlscene FILE`

**Example:**
```
qmlscene publisher.qml
```
----------
**Note:**
For the `tf_transforms.qml` and `urdf_tutorial_combined.qml`, you can use the following simulation:
```
roslaunch urdf_sim_tutorial 13-diffdrive.launch
```
