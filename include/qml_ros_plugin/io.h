// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_IO_H
#define QML_ROS_PLUGIN_IO_H

#include <QJSValue>

namespace qml_ros_plugin
{

class IO
{
  Q_GADGET
public:
  /*!
   * Writes the given value to the given path in the yaml format.
   * @param path The path to the file.
   * @param value The value to write.
   * @return True if successful, false otherwise.
   */
  Q_INVOKABLE bool writeYaml( QString path, const QVariant &value );

  /*!
   * Reads a yaml file and returns the content in a QML compatible structure of 'QVariantMap's and 'QVariantList's.
   * @param path The path to the file.
   * @return A variant containing the file content or false.
   */
  Q_INVOKABLE QVariant readYaml( QString path );
};
} // namespace qml_ros_plugin

Q_DECLARE_METATYPE( qml_ros_plugin::IO );

#endif // QML_ROS_PLUGIN_IO_H
