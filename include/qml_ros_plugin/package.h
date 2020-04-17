// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_PACKAGE_H
#define QML_ROS_PLUGIN_PACKAGE_H

#include <QtCore>

namespace qml_ros_plugin
{

/*!
 * A wrapper for ros::package
 */
class Package
{
Q_GADGET

public:
  /*!
   * Runs a command of the form 'rospack <cmd>'. (This does not make a subprocess call!)
   * @param cmd The command passed to rospack.
   * @return The output of the command as a string.
   */
  Q_INVOKABLE QString command( const QString &cmd );

  /*!
   * Queries the path to a package.
   * @param package_name The name of the package.
   * @return The fully-qualified path to the given package or an empty string if the package is not found.
   */
  Q_INVOKABLE QString getPath( const QString &package_name );

  /*!
   * @return A list of all packages.
   */
  Q_INVOKABLE QStringList getAll();

  /*!
   * Queries for all plugins exported for a given package.
   *
   * @code{.xml}
   * <export>
   *   <name attribute="value"/>
   *   <rviz plugin="${prefix}/plugin_description.xml"/>
   * </export>
   * @endcode
   *
   * To query for rviz plugins you would pass 'rviz' as the name and 'plugin' as the attribute.
   *
   * @param name The name of the export tag.
   * @param attribute The name of the attribute for the value is obtained.
   * @param force_recrawl Forces rospack to rediscover everything on the system before running the search.
   * @return A map with the name of the package exporting something for name as the key (string) and a QStringList
   *   containing all exported values as the value.
   */
  Q_INVOKABLE QVariantMap getPlugins( const QString &name, const QString &attribute, bool force_recrawl = false );
};
}

Q_DECLARE_METATYPE( qml_ros_plugin::Package );

#endif //QML_ROS_PLUGIN_PACKAGE_H
