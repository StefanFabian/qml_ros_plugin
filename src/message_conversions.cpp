// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/message_conversions.h"

namespace qml_ros_plugin
{

QVariantMap msgToMap( const std_msgs::Header &msg )
{
  QVariantMap result;
  result.insert( "frame_id", QString::fromStdString( msg.frame_id ));
  result.insert( "stamp", msg.stamp.toSec());
  result.insert( "seq", msg.seq );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Transform &msg )
{
  QVariantMap result;
  result.insert( "translation", QVariant::fromValue(msgToMap( msg.translation )));
  result.insert( "rotation", QVariant::fromValue(msgToMap( msg.rotation )));
  return result;
}

QVariantMap msgToMap( const geometry_msgs::TransformStamped &msg )
{
  QVariantMap result;
  result.insert( "header", QVariant::fromValue(msgToMap( msg.header )));
  result.insert( "child_frame_id", QString::fromStdString( msg.child_frame_id ));
  result.insert( "transform", QVariant::fromValue(msgToMap( msg.transform )));
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Vector3 &msg )
{
  QVariantMap result;
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}

QVariantMap msgToMap( const geometry_msgs::Quaternion &msg )
{
  QVariantMap result;
  result.insert( "w", msg.w );
  result.insert( "x", msg.x );
  result.insert( "y", msg.y );
  result.insert( "z", msg.z );
  return result;
}
}
