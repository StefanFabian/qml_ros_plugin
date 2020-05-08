//
// Created by Stefan Fabian on 02.10.19.
//

#ifndef QML_ROS_PLUGIN_MESSAGE_COMPARISON_H
#define QML_ROS_PLUGIN_MESSAGE_COMPARISON_H

#include <gtest/gtest.h>

#include <qml_ros_plugin/array.h>
#include <qml_ros_plugin/qml_ros_conversion.h>

#include <geometry_msgs/TransformStamped.h>
#include <ros_babel_fish/message_types.h>
#include <ros_babel_fish_test_msgs/TestArray.h>
#include <ros_babel_fish_test_msgs/TestMessage.h>

#include <QDateTime>
#include <QVariant>

using namespace qml_ros_plugin;
using namespace ros_babel_fish;
using namespace ros_babel_fish_test_msgs;

template<typename T>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, T msg, const std::string &path = "msg", double = 1E-10 )
{
  if ( map.value<T>() == msg ) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Map at " << path << " differed!" << std::endl
                                       << "Map: " << map.value<T>() << std::endl
                                       << "Message: " << msg;
}

template<typename ContainerTypeA, typename ContainerTypeB>
::testing::AssertionResult
mapAndMessageEqualArray( const ContainerTypeA &a, const ContainerTypeB &b, const std::string &path,
                         double precision = 1E-10 )
{
  if ( static_cast<size_t>(a.size()) != b.size())
    return ::testing::AssertionFailure() << "Size of " << path << " differed!" << std::endl
                                         << "Map size: " << a.size() << std::endl
                                         << "Message size: " << b.size();
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for ( size_t i = 0; i < b.size(); ++i )
  {
    if ( !(result = mapAndMessageEqual( a.at( i ), b[i], path + "[" + std::to_string( i ) + "]", precision )))
      return result;
  }
  return result;
}

template<typename ElementType>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const std::vector<ElementType> msg,
                                               const std::string &path = "msg", double precision = 1E-10 )
{
  if ( map.type() == QVariant::List )
  {
    return mapAndMessageEqualArray( map.toList(), msg, path, precision );
  }
  return mapAndMessageEqualArray( map.value<Array>(), msg, path, precision );
}

template<typename ElementType, unsigned long N>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const boost::array<ElementType, N> &msg,
                                               const std::string &path = "msg", double precision = 1E-10 )
{
  if ( map.type() == QVariant::List )
  {
    return mapAndMessageEqualArray( map.toList(), msg, path, precision );
  }
  return mapAndMessageEqualArray( map.value<Array>(), msg, path, precision );
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, std::string msg, const std::string &path, double )
{

  if ( map.toString().toStdString() == msg ) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Map at " << path << " differed!" << std::endl
                                       << "Map: " << map.toString().toStdString() << std::endl
                                       << "Message: " << msg;
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, ros::Time msg, const std::string &path, double )
{
  qint64 msec_since_epoch = msg.sec * 1000L + msg.nsec / 1000000;
  if ( map.toDateTime() == QDateTime::fromMSecsSinceEpoch( msec_since_epoch )) return ::testing::AssertionSuccess();
  return ::testing::AssertionFailure() << "Map at " << path << " differed!" << std::endl
                                       << "Map: " << map.toDateTime().toMSecsSinceEpoch() << std::endl
                                       << "Message: " << msec_since_epoch;
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, ros::Duration msg, const std::string &path, double precision )
{
  bool ok;
  if ( std::abs( map.toDouble( &ok ) - msg.toSec() * 1000 ) <= precision && ok ) return ::testing::AssertionSuccess();
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << " not ok!";
  return ::testing::AssertionFailure() << "Map at " << path << " differed!" << std::endl
                                       << "Map: " << map.toDouble() << std::endl
                                       << "Message: " << msg.toSec() * 1000;
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, std_msgs::Header msg, const std::string &path, double )
{
  if ( map.toMap()["frame_id"].toString().toStdString() != msg.frame_id )
    return ::testing::AssertionFailure() << "Map at " << path << ".frame_id differed!" << std::endl
                                         << "Map: " << map.toMap()["frame_id"].toString().toStdString() << std::endl
                                         << "Message: " << msg.frame_id;
  if ( map.toMap()["stamp"].toDateTime() != rosToQmlTime( msg.stamp ))
    return ::testing::AssertionFailure() << "Map at " << path << ".stamp differed!" << std::endl
                                         << "Map: " << map.toMap()["stamp"].toDateTime().toMSecsSinceEpoch()
                                         << std::endl
                                         << "Message: " << msg.stamp.toSec() * 1000;
  return ::testing::AssertionSuccess();
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, geometry_msgs::Point msg, const std::string &path, double precision )
{
  bool ok;
  if ( std::abs( map.toHash()["x"].toDouble( &ok ) - msg.x ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".x differed!" << std::endl
                                         << "Map: " << map.toHash()["x"].toDouble() << std::endl
                                         << "Message: " << msg.x;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".x not ok!";
  if ( std::abs( map.toHash()["y"].toDouble( &ok ) - msg.y ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".y differed!" << std::endl
                                         << "Map: " << map.toHash()["y"].toDouble() << std::endl
                                         << "Message: " << msg.y;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".y not ok!";
  if ( std::abs( map.toHash()["z"].toDouble( &ok ) - msg.z ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".z differed!" << std::endl
                                         << "Map: " << map.toHash()["z"].toDouble() << std::endl
                                         << "Message: " << msg.z;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".z not ok!";
  return ::testing::AssertionSuccess();
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, TestMessage msg, const std::string &path, double precision )
{
  if ( map.toHash()["header"].toHash()["frame_id"].toString().toStdString() != msg.header.frame_id )
    return ::testing::AssertionFailure() << "Map at " << path << ".header.frame_id  differed!" << std::endl
                                         << "Map: "
                                         << map.toHash()["header"].toHash()["frame_id"].toString().toStdString()
                                         << std::endl
                                         << "Message: " << msg.header.frame_id;
  if ( map.toHash()["header"].toHash()["stamp"].toDateTime() != rosToQmlTime( msg.header.stamp ))
    return ::testing::AssertionFailure() << "Map at " << path << ".header.stamp differed!" << std::endl
                                         << "Map: "
                                         << map.toHash()["header"].toHash()["stamp"].toDateTime().toMSecsSinceEpoch()
                                         << std::endl
                                         << "Message: " << msg.header.stamp.toSec() * 1000;
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = mapAndMessageEqual( map.toHash()["b"], msg.b, path + ".b", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["ui8"], msg.ui8, path + ".ui8", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["ui16"], msg.ui16, path + ".ui16", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["ui32"], msg.ui32, path + ".ui32", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["ui64"], msg.ui64, path + ".ui64", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["i8"], msg.i8, path + ".i8", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["i16"], msg.i16, path + ".i16", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["i32"], msg.i32, path + ".i32", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["i64"], msg.i64, path + ".i64", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["f32"], msg.f32, path + ".f32", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["f64"], msg.f64, path + ".f64", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["str"], msg.str, path + ".str", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["d"], msg.d, path + ".d", precision )))return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["t"], msg.t, path + ".t", precision )))return result;
  return mapAndMessageEqual( map.toHash()["point_arr"], msg.point_arr, ".point_arr" );
}

template<>
::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, TestArray msg, const std::string &path, double precision )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = mapAndMessageEqual( map.toHash()["bools"], msg.bools, path + ".bools", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["durations"], msg.durations, path + ".durations", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["times"], msg.times, path + ".times", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["float32s"], msg.float32s, path + ".float32s", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["float64s"], msg.float64s, path + ".float64s", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["int8s"], msg.int8s, path + ".int8s", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["int16s"], msg.int16s, path + ".int16s", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["int32s"], msg.int32s, path + ".int32s", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["int64s"], msg.int64s, path + ".int64s", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["uint8s"], msg.uint8s, path + ".uint8s", precision ))) return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["uint16s"], msg.uint16s, path + ".uint16s", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["uint32s"], msg.uint32s, path + ".uint32s", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["uint64s"], msg.uint64s, path + ".uint64s", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["strings"], msg.strings, path + ".strings", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["subarrays_fixed"], msg.subarrays_fixed,
                                      path + ".subarrays_fixed", precision )))
    return result;
  return mapAndMessageEqual( map.toHash()["subarrays"], msg.subarrays, path + ".subarrays", precision );
}

::testing::AssertionResult
mapAndMessageEqual( const QVariant &map, const TestSubArray &msg, const std::string &path, double precision )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = mapAndMessageEqual( map.toHash()["ints"], msg.ints, path + ".ints", precision )))
    return result;
  if ( !(result = mapAndMessageEqual( map.toHash()["strings"], msg.strings, path + ".strings", precision )))
    return result;
  return mapAndMessageEqual( map.toHash()["times"], msg.times, path + ".times", precision );
}

template<>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const geometry_msgs::Vector3 msg,
                                               const std::string &path, double precision )
{
  bool ok;
  if ( std::abs( map.toHash()["x"].toDouble( &ok ) - msg.x ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".x differed!" << std::endl
                                         << "Map: " << map.toHash()["x"].toDouble() << std::endl
                                         << "Message: " << msg.x;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".x not ok!";
  if ( std::abs( map.toHash()["y"].toDouble( &ok ) - msg.y ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".y differed!" << std::endl
                                         << "Map: " << map.toHash()["y"].toDouble() << std::endl
                                         << "Message: " << msg.y;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".y not ok!";
  if ( std::abs( map.toHash()["z"].toDouble( &ok ) - msg.z ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".z differed!" << std::endl
                                         << "Map: " << map.toHash()["z"].toDouble() << std::endl
                                         << "Message: " << msg.z;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".z not ok!";
  return ::testing::AssertionSuccess();
}

template<>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const geometry_msgs::Quaternion msg,
                                               const std::string &path, double precision )
{
  bool ok;
  if ( std::abs( map.toHash()["w"].toDouble( &ok ) - msg.w ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".w differed!" << std::endl
                                         << "Map: " << map.toHash()["w"].toDouble() << std::endl
                                         << "Message: " << msg.w;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".w not ok!";
  if ( std::abs( map.toHash()["x"].toDouble( &ok ) - msg.x ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".x differed!" << std::endl
                                         << "Map: " << map.toHash()["x"].toDouble() << std::endl
                                         << "Message: " << msg.x;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".x not ok!";
  if ( std::abs( map.toHash()["y"].toDouble( &ok ) - msg.y ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".y differed!" << std::endl
                                         << "Map: " << map.toHash()["y"].toDouble() << std::endl
                                         << "Message: " << msg.y;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".y not ok!";
  if ( std::abs( map.toHash()["z"].toDouble( &ok ) - msg.z ) > precision )
    return ::testing::AssertionFailure() << "Map at " << path << ".z differed!" << std::endl
                                         << "Map: " << map.toHash()["z"].toDouble() << std::endl
                                         << "Message: " << msg.z;
  if ( !ok ) return ::testing::AssertionFailure() << "Double cast at " << path << ".z not ok!";
  return ::testing::AssertionSuccess();
}

template<>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const geometry_msgs::Transform msg,
                                               const std::string &path, double precision )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = mapAndMessageEqual( map.toMap()["translation"], msg.translation, path + ".translation", precision )))
    return result;
  return mapAndMessageEqual( map.toMap()["rotation"], msg.rotation, path + ".rotation", precision );
}

template<>
::testing::AssertionResult mapAndMessageEqual( const QVariant &map, const geometry_msgs::TransformStamped msg,
                                               const std::string &path, double precision )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = mapAndMessageEqual( map.toMap()["header"], msg.header, path + ".header", precision )))
    return result;
  if ( map.toMap()["child_frame_id"].toString() != QString::fromStdString( msg.child_frame_id ))
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".child_frame_id !"
                                         << std::endl
                                         << "Map: " << map.toMap()["child_frame_id"].toString().toStdString()
                                         << std::endl
                                         << "Message: " << msg.child_frame_id;
  return mapAndMessageEqual( map.toMap()["transform"], msg.transform, path + ".transform", precision );
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const TestMessage &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  if ( compound["header"]["frame_id"].value<std::string>() != msg.header.frame_id )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".header.frame_id !"
                                         << std::endl
                                         << "Map: " << compound["header"]["frame_id"].value<std::string>() << std::endl
                                         << "Message: " << msg.header.frame_id;
  if ( compound["header"]["stamp"].value<ros::Time>() != msg.header.stamp )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".header.frame_id!"
                                         << std::endl
                                         << "Map: " << compound["header"]["stamp"].value<ros::Time>() << std::endl
                                         << "Message: " << msg.header.stamp;
  if ( compound["b"].value<bool>() != msg.b )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".b !" << std::endl
                                         << "Map: " << compound["b"].value<bool>() << std::endl
                                         << "Message: " << msg.b;
  if ( compound["ui8"].value<uint8_t>() != msg.ui8 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".ui8 !" << std::endl
                                         << "Map: " << compound["ui8"].value<uint8_t>() << std::endl
                                         << "Message: " << msg.ui8;
  if ( compound["ui16"].value<uint16_t>() != msg.ui16 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".ui16 !" << std::endl
                                         << "Map: " << compound["ui16"].value<uint16_t>() << std::endl
                                         << "Message: " << msg.ui16;
  if ( compound["ui32"].value<uint32_t>() != msg.ui32 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".ui32 !" << std::endl
                                         << "Map: " << compound["ui32"].value<uint32_t>() << std::endl
                                         << "Message: " << msg.ui32;
  if ( compound["ui64"].value<uint64_t>() != msg.ui64 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".ui64 !" << std::endl
                                         << "Map: " << compound["ui64"].value<uint64_t>() << std::endl
                                         << "Message: " << msg.ui64;
  if ( compound["i8"].value<int8_t>() != msg.i8 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".i8 !" << std::endl
                                         << "Map: " << compound["i8"].value<int8_t>() << std::endl
                                         << "Message: " << msg.i8;
  if ( compound["i16"].value<int16_t>() != msg.i16 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".i16 !" << std::endl
                                         << "Map: " << compound["i16"].value<int16_t>() << std::endl
                                         << "Message: " << msg.i16;
  if ( compound["i32"].value<int32_t>() != msg.i32 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".i32 !" << std::endl
                                         << "Map: " << compound["i32"].value<int32_t>() << std::endl
                                         << "Message: " << msg.i32;
  if ( compound["i64"].value<int64_t>() != msg.i64 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".i64 !" << std::endl
                                         << "Map: " << compound["i64"].value<int64_t>() << std::endl
                                         << "Message: " << msg.i64;
  if ( compound["f32"].value<float>() != msg.f32 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".f32 !" << std::endl
                                         << "Map: " << compound["f32"].value<float>() << std::endl
                                         << "Message: " << msg.f32;
  if ( compound["f64"].value<double>() != msg.f64 )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".f64 !" << std::endl
                                         << "Map: " << compound["f64"].value<double>() << std::endl
                                         << "Message: " << msg.f64;
  if ( compound["str"].value<std::string>() != msg.str )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".str !" << std::endl
                                         << "Map: " << compound["str"].value<std::string>() << std::endl
                                         << "Message: " << msg.str;
  // Make sure time is accurate to the millisecond which is the supported resolution by QDateTime
  if ((compound["t"].value<ros::Time>().toSec() - msg.t.toSec()) >= 1E-3 )
    return ::testing::AssertionFailure() << "Map at " << path << ".t differed!" << std::endl
                                         << "Map: " << compound["t"].value<ros::Time>() << std::endl
                                         << "Message: " << msg.t;
  if ( std::abs( compound["d"].value<ros::Duration>().toSec() - msg.d.toSec()) >= 1E-3 )
    return ::testing::AssertionFailure() << "Map at " << path << ".d differed!" << std::endl
                                         << "Map: " << compound["d"].value<ros::Duration>() << std::endl
                                         << "Message: " << msg.d;

  const auto &list = compound["point_arr"].as<CompoundArrayMessage>();
  if ( list.length() != msg.point_arr.size())
    return ::testing::AssertionFailure() << "Size of msg.point_arr differed!" << std::endl
                                         << "Map size: " << list.length() << std::endl
                                         << "Message size: " << msg.point_arr.size();
  for ( size_t i = 0; i < list.length(); ++i )
  {
    if ( std::abs( list[i]["x"].value<double>() - msg.point_arr[i].x ) > precision )
      return ::testing::AssertionFailure() << "Map at " << path << ".point_arr[" << i << "].x differed!" << std::endl
                                           << "Map: " << list[i]["x"].value<double>() << std::endl
                                           << "Message: " << msg.point_arr[i].x;
    if ( std::abs( list[i]["y"].value<double>() - msg.point_arr[i].y ) > precision )
      return ::testing::AssertionFailure() << "Map at " << path << ".point_arr[" << i << "].y differed!" << std::endl
                                           << "Map: " << list[i]["y"].value<double>() << std::endl
                                           << "Message: " << msg.point_arr[i].y;
    if ( std::abs( list[i]["z"].value<double>() - msg.point_arr[i].z ) > precision )
      return ::testing::AssertionFailure() << "Map at " << path << ".point_arr[" << i << "].z differed!" << std::endl
                                           << "Map: " << list[i]["z"].value<double>() << std::endl
                                           << "Message: " << msg.point_arr[i].z;
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const TestSubArray &msg,
                                         const std::string &path = "msg", double precision = 1E-10 );

template<typename T1, typename T2>
bool valueEqual( const T1 &a, const T2 &b )
{
  return a == b;
}

template<>
bool valueEqual( const ros::Time &a, const ros::Time &b )
{
  return std::abs( a.toSec() - b.toSec()) < 1E-3; // Should differ at most by less than a millisecond
}

template<typename T, typename ContainerType>
typename std::enable_if<!std::is_same<T, Message>::value, ::testing::AssertionResult>::type
messageEqual( const ArrayMessage<T> &arr, const ContainerType &msg, const std::string &path = "msg",
              double = 1E-10 )
{
  if ( arr.length() != msg.size())
    return ::testing::AssertionFailure() << "Size of " << path << " differed!" << std::endl
                                         << "Map size: " << arr.length() << std::endl
                                         << "Message size: " << msg.size();
  for ( size_t i = 0; i < arr.length(); ++i )
  {
    if ( !valueEqual( arr[i], msg[i] ))
      return ::testing::AssertionFailure() << "Message at " << path << "[" << i << "] differed!" << std::endl
                                           << "Map: " << arr[i] << std::endl
                                           << "Message: " << msg[i];
  }
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult messageEqual( const CompoundArrayMessage &arr, const std::vector<TestSubArray> &msg,
                                         const std::string &path, double precision = 1E-10 )
{
  if ( arr.length() != msg.size())
    return ::testing::AssertionFailure() << "Size of " << path << " differed!" << std::endl
                                         << "Map size: " << arr.length() << std::endl
                                         << "Message size: " << msg.size();
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for ( size_t i = 0; i < arr.length(); ++i )
  {
    if ( !(result = messageEqual( arr[i].as<CompoundMessage>(), msg[i], path + "[" + std::to_string( i ) + "]",
                                  precision )))
      return result;
  }
  return result;
}

template<unsigned long N>
::testing::AssertionResult messageEqual( const CompoundArrayMessage &arr, const boost::array<TestSubArray, N> &msg,
                                         const std::string &path, double precision = 1E-10 )
{
  if ( arr.length() != msg.size())
    return ::testing::AssertionFailure() << "Size of " << path << " differed!" << std::endl
                                         << "Map size: " << arr.length() << std::endl
                                         << "Message size: " << msg.size();
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  for ( size_t i = 0; i < arr.length(); ++i )
  {
    if ( !(result = messageEqual( arr[i].as<CompoundMessage>(), msg[i], path + "[" + std::to_string( i ) + "]",
                                  precision )))
      return result;
  }
  return result;
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const std_msgs::Header &msg,
                                         const std::string &path, double = 1E-10 )
{
  if ( compound["frame_id"].value<std::string>() != msg.frame_id )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".frame_id !" << std::endl
                                         << "Map: " << compound["frame_id"].value<std::string>() << std::endl
                                         << "Message: " << msg.frame_id;
  if ( compound["stamp"].value<ros::Time>() != msg.stamp )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".stamp !" << std::endl
                                         << "Map: " << compound["stamp"].value<ros::Time>() << std::endl
                                         << "Message: " << msg.stamp;
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const TestArray &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = messageEqual( compound["bools"].as<ArrayMessage<bool>>(), msg.bools, path + ".bools", precision )))
    return result;
  if ( !(result = messageEqual( compound["durations"].as<ArrayMessage<ros::Duration>>(),
                                msg.durations, path + ".durations", precision )))
    return result;
  if ( !(result = messageEqual( compound["times"].as<ArrayMessage<ros::Time>>(), msg.times, path + ".times",
                                precision )))
    return result;
  if ( !(result = messageEqual( compound["float32s"].as<ArrayMessage<float>>(), msg.float32s,
                                path + ".float32s", precision )))
    return result;
  if ( !(result = messageEqual( compound["float64s"].as<ArrayMessage<double>>(), msg.float64s,
                                path + ".float64s", precision )))
    return result;
  if ( !(result = messageEqual( compound["int8s"].as<ArrayMessage<int8_t >>(), msg.int8s,
                                path + ".int8s", precision )))
    return result;
  if ( !(result = messageEqual( compound["int16s"].as<ArrayMessage<int16_t >>(), msg.int16s, path + ".int16s",
                                precision )))
    return result;
  if ( !(result = messageEqual( compound["int32s"].as<ArrayMessage<int32_t >>(), msg.int32s, path + ".int32s",
                                precision )))
    return result;
  if ( !(result = messageEqual( compound["int64s"].as<ArrayMessage<int64_t >>(), msg.int64s, path + ".int64s",
                                precision )))
    return result;
  if ( !(result = messageEqual( compound["uint8s"].as<ArrayMessage<uint8_t >>(), msg.uint8s, path + ".uint8s",
                                precision )))
    return result;
  if ( !(result = messageEqual( compound["uint16s"].as<ArrayMessage<uint16_t >>(), msg.uint16s,
                                path + ".uint16s", precision )))
    return result;
  if ( !(result = messageEqual( compound["uint32s"].as<ArrayMessage<uint32_t >>(), msg.uint32s,
                                path + ".uint32s", precision )))
    return result;
  if ( !(result = messageEqual( compound["uint64s"].as<ArrayMessage<uint64_t >>(), msg.uint64s,
                                path + ".uint64s", precision )))
    return result;
  if ( !(result = messageEqual( compound["strings"].as<ArrayMessage<std::string>>(), msg.strings,
                                path + ".strings", precision )))
    return result;
  if ( !(result = messageEqual( compound["subarrays_fixed"].as<CompoundArrayMessage>(), msg.subarrays_fixed,
                                path + ".subarrays_fixed", precision )))
    return result;
  return messageEqual( compound["subarrays"].as<CompoundArrayMessage>(), msg.subarrays, path + ".subarrays",
                       precision );
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const TestSubArray &msg,
                                         const std::string &path, double precision )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = messageEqual( compound["ints"].as<ArrayMessage<int32_t>>(), msg.ints, path + ".ints", precision )))
    return result;
  if ( !(result = messageEqual( compound["strings"].as<ArrayMessage<std::string>>(), msg.strings, path + ".strings",
                                precision )))
    return result;
  return messageEqual( compound["times"].as<ArrayMessage<ros::Time>>(), msg.times, path + ".times", precision );
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const geometry_msgs::Vector3 &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  if ( std::abs( compound["x"].value<double>() - msg.x ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".x !" << std::endl
                                         << "Map: " << compound["x"].value<double>() << std::endl
                                         << "Message: " << msg.x;
  if ( std::abs( compound["y"].value<double>() - msg.y ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".y !" << std::endl
                                         << "Map: " << compound["y"].value<double>() << std::endl
                                         << "Message: " << msg.y;
  if ( std::abs( compound["z"].value<double>() - msg.z ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".z !" << std::endl
                                         << "Map: " << compound["z"].value<double>() << std::endl
                                         << "Message: " << msg.z;
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const geometry_msgs::Quaternion &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  if ( std::abs( compound["w"].value<double>() - msg.w ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".w !" << std::endl
                                         << "Map: " << compound["w"].value<double>() << std::endl
                                         << "Message: " << msg.w;
  if ( std::abs( compound["x"].value<double>() - msg.x ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".x !" << std::endl
                                         << "Map: " << compound["x"].value<double>() << std::endl
                                         << "Message: " << msg.x;
  if ( std::abs( compound["y"].value<double>() - msg.y ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".y !" << std::endl
                                         << "Map: " << compound["y"].value<double>() << std::endl
                                         << "Message: " << msg.y;
  if ( std::abs( compound["z"].value<double>() - msg.z ) < precision )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".z !" << std::endl
                                         << "Map: " << compound["z"].value<double>() << std::endl
                                         << "Message: " << msg.z;
  return ::testing::AssertionSuccess();
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const geometry_msgs::Transform &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = messageEqual( compound["translation"].as<CompoundMessage>(), msg.translation,
                                path + ".translation", precision )))
    return result;
  return messageEqual( compound["rotation"].as<CompoundMessage>(), msg.rotation, path + ".rotation", precision );
}

::testing::AssertionResult messageEqual( const CompoundMessage &compound, const geometry_msgs::TransformStamped &msg,
                                         const std::string &path = "msg", double precision = 1E-10 )
{
  ::testing::AssertionResult result = ::testing::AssertionSuccess();
  if ( !(result = messageEqual( compound["header"].as<CompoundMessage>(), msg.header, path + ".header", precision )))
    return result;
  if ( compound["child_frame_id"].value<std::string>() != msg.child_frame_id )
    return ::testing::AssertionFailure() << "Retranslated message differed at " << path << ".child_frame_id !"
                                         << std::endl
                                         << "Map: " << compound["child_frame_id"].value<std::string>() << std::endl
                                         << "Message: " << msg.child_frame_id;
  return messageEqual( compound["transform"].as<CompoundMessage>(), msg.transform, path + ".transform", precision );
}

#endif //QML_ROS_PLUGIN_MESSAGE_COMPARISON_H
