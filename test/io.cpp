//
// Created by Stefan Fabian on 07.05.20.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/message_conversions.h>
#include <qml_ros_plugin/ros.h>

#include <QQmlComponent>
#include <QQmlEngine>

#include <ros/package.h>
#include <ros/ros.h>

using namespace qml_ros_plugin;

TEST( IO, yaml )
{
  // Manually register converter which would normally be done by plugin registration function
  QMetaType::registerConverter<Array, QVariantList>( &Array::toVariantList );
  BabelFish fish;
  RosQmlSingletonWrapper wrapper;
  IO io = wrapper.io();

  ros_babel_fish_test_msgs::TestArray test_array;
  unsigned SEED = 42;
  fillBoolArray( test_array.bools, SEED++ );
  fillArray( test_array.uint8s, SEED++ );
  fillArray( test_array.uint16s, SEED++ );
  fillArray( test_array.uint32s, SEED++ );
  fillArray( test_array.uint64s, SEED++ );
  fillArray( test_array.int8s, SEED++ );
  fillArray( test_array.int16s, SEED++ );
  fillArray( test_array.int32s, SEED++ );
  fillArray( test_array.int64s, SEED++ );
  fillArray( test_array.float32s, SEED++ );
  fillArray( test_array.float64s, SEED++ );
  fillArray( test_array.times, SEED++ );
  fillArray( test_array.durations, SEED++ );
  fillArray( test_array.strings, SEED++ );
  fillArray( test_array.subarrays_fixed, SEED++ );
  fillArray( test_array.subarrays, SEED++ );

  ros::SerializedMessage serialized_msg = ros::serialization::serializeMessage( test_array );
  BabelFishMessage::Ptr bf_msg = boost::make_shared<BabelFishMessage>();
  bf_msg->morph(
      fish.descriptionProvider()->getMessageDescription( "ros_babel_fish_test_msgs/TestArray" ) );
  ros::serialization::deserializeMessage( serialized_msg, *bf_msg );
  TranslatedMessage::ConstPtr translated = fish.translateMessage( bf_msg );
  QVariant map = conversion::msgToMap( translated );

  std::string path = ros::package::getPath( ROS_PACKAGE_NAME ) + "/test/test_io/array_message.yaml";
  ASSERT_TRUE( io.writeYaml( QString::fromStdString( path ), map ) );

  QVariant file = io.readYaml( QString::fromStdString( path ) );

  Message::Ptr file_msg = fish.createMessage( "ros_babel_fish_test_msgs/TestArray" );
  conversion::fillMessage( *file_msg, file );
  ASSERT_TRUE( messageEqual( file_msg->as<CompoundMessage>(), test_array ) );

  ASSERT_FALSE( io.writeYaml(
      "/usr/bin/i_really_hope_this_fails_on_your_machine_for_your_own_sake.mission", map ) );
  EXPECT_FALSE( io.writeYaml( "https://somewebserver.org/mission.mission", map ) );
  file = io.readYaml( "https://somewebserver.org/mission.mission" );
  EXPECT_EQ( file.type(), QVariant::Bool );
  EXPECT_FALSE( file.toBool() );
  file =
      io.readYaml( "/usr/bin/i_really_hope_this_fails_on_your_machine_for_your_own_sake.mission" );
  EXPECT_EQ( file.type(), QVariant::Bool );
  EXPECT_FALSE( file.toBool() );

  QQmlEngine engine;
  QQmlComponent component( &engine );
  component.setData( R"(
import QtQuick 2.0

QtObject {
  property bool b: false
  property int ui8: 42
  property var string_arr: ["first", "second"]
  property ListModel point_arr: ListModel {
    ListElement { x: 1; y: 2; z: 3 }
    ListElement { x: 3; y: 2; z: 3 }
    ListElement { x: 1; y: 3.14; z: 1 }
    ListElement { x: 10000; y: -2; z: -300 }
  }
}
)",
                     QUrl() );
  QObject *obj = component.create();
  path = ros::package::getPath( ROS_PACKAGE_NAME ) + "/test/test_io/qobject.yaml";
  ASSERT_TRUE( io.writeYaml( QString::fromStdString( path ), QVariant::fromValue( obj ) ) );

  file = io.readYaml( QString::fromStdString( path ) );
  ASSERT_EQ( file.type(), QVariant::Map );
  QVariantMap content = file.toMap();
  ASSERT_TRUE( content.contains( "b" ) );
  ASSERT_EQ( content["b"].type(), QVariant::Bool );
  EXPECT_FALSE( content["b"].toBool() );
  ASSERT_TRUE( content.contains( "ui8" ) );
  bool ok = false;
  EXPECT_EQ( content["ui8"].toULongLong( &ok ), 42ULL );
  EXPECT_TRUE( ok );
  ASSERT_TRUE( content.contains( "point_arr" ) );
  ASSERT_EQ( content["point_arr"].type(), QVariant::List );
  QVariantList arr = content["point_arr"].toList();
  ASSERT_EQ( arr.length(), 4 );
  std::vector<std::tuple<QVariant::Type, double, QVariant::Type, double, QVariant::Type, double>>
      expected_points = {
          { QVariant::ULongLong, 1, QVariant::ULongLong, 2, QVariant::ULongLong, 3 },
          { QVariant::ULongLong, 3, QVariant::ULongLong, 2, QVariant::ULongLong, 3 },
          { QVariant::ULongLong, 1, QVariant::Double, 3.14, QVariant::ULongLong, 1 },
          { QVariant::ULongLong, 10000, QVariant::LongLong, -2, QVariant::LongLong, -300 },
      };
  for ( int i = 0; i < 4; ++i ) {
    ASSERT_EQ( arr[i].type(), QVariant::Map );
    QVariantMap entry = arr[i].toMap();
    ASSERT_TRUE( entry.contains( "x" ) );
    ASSERT_EQ( entry["x"].type(), std::get<0>( expected_points[i] ) );
    ASSERT_DOUBLE_EQ( entry["x"].toDouble(), std::get<1>( expected_points[i] ) );
    ASSERT_TRUE( entry.contains( "y" ) );
    ASSERT_EQ( entry["y"].type(), std::get<2>( expected_points[i] ) );
    ASSERT_DOUBLE_EQ( entry["y"].toDouble(), std::get<3>( expected_points[i] ) );
    ASSERT_TRUE( entry.contains( "z" ) );
    ASSERT_EQ( entry["z"].type(), std::get<4>( expected_points[i] ) );
    ASSERT_DOUBLE_EQ( entry["z"].toDouble(), std::get<5>( expected_points[i] ) );
  }

  ASSERT_TRUE( content.contains( "string_arr" ) );
  ASSERT_EQ( content["string_arr"].type(), QVariant::List );
  arr = content["string_arr"].toList();
  ASSERT_EQ( arr.length(), 2 );
  ASSERT_EQ( arr[0].type(), QVariant::String );
  ASSERT_EQ( arr[0].toString(), "first" );
  ASSERT_EQ( arr[1].type(), QVariant::String );
  ASSERT_EQ( arr[1].toString(), "second" );

  path = ros::package::getPath( ROS_PACKAGE_NAME ) + "/test/test_io/test.yaml";
  file = io.readYaml( QString::fromStdString( path ) );
  ASSERT_EQ( file.type(), QVariant::Map );
  content = file.toMap();
  ASSERT_TRUE( content.contains( "first" ) );
  EXPECT_EQ( content["first"], QVariant() );
  ASSERT_TRUE( content.contains( "second" ) );
  ASSERT_EQ( content["second"].type(), QVariant::String );
  EXPECT_EQ( content["second"].toString(), "A string without tag" );

  EXPECT_FALSE( io.writeYaml( "", QVariant() ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_package" );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}
