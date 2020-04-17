// Copyright (c) 2020 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/package.h"
#include "qml_ros_plugin/message_conversions.h"

#include <ros/package.h>
#include <ros/ros.h>

namespace qml_ros_plugin
{

QString Package::command( const QString &cmd )
{
  return QString::fromStdString( ros::package::command( cmd.toStdString()));
}

QString Package::getPath( const QString &package_name )
{
  return QString::fromStdString( ros::package::getPath( package_name.toStdString()));
}

QStringList Package::getAll()
{
  QStringList result;
  ros::V_string result_std;
  if ( !ros::package::getAll( result_std ))
  {
    ROS_WARN_NAMED( "qml_ros_plugin", "Failed to get packages!" );
    return {};
  }
  result.reserve( result_std.size());
  for ( const auto &s : result_std )
    result.append( QString::fromStdString( s ));
  return result;
}

QVariantMap Package::getPlugins( const QString &name, const QString &attribute, bool force_recrawl )
{
  std::vector<std::pair<std::string, std::string>> exports;
  ros::package::getPlugins( name.toStdString(), attribute.toStdString(), exports, force_recrawl );
  QVariantMap result;
  for ( const auto &pair : exports )
  {
    const QString &package = QString::fromStdString( pair.first );
    const QString &value = QString::fromStdString( pair.second );
    if ( result.contains( package ))
    {
      conversion::obtainValueAsReference<QStringList>( result[package] ).append( value );
      continue;
    }
    result.insert( package, QStringList{ value } );
  }
  return result;
}
}
