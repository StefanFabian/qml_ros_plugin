//
// Created by stefan on 06.11.19.
//


#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/publisher.h>
#include <qml_ros_plugin/ros.h>
#include <qml_ros_plugin/subscriber.h>
#include <qml_ros_plugin/tf_transform_listener.h>
#include <qml_ros_plugin/time.h>

#include <QCoreApplication>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

void processSomeEvents( int n = 10, int sleep_duration_us = 5000 )
{
  for ( int i = 0; i < n; ++i )
  {
    usleep( sleep_duration_us );
    QCoreApplication::processEvents();
  }
}

TEST( Spinning, testSpinning )
{
  qml_ros_plugin::RosQmlSingletonWrapper ros_wrapper;
  processSomeEvents();

  // Initialize ROS
  ros_wrapper.init( { "name", "test:=pose" }, "test_ros_life_cycle" );
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>( "/pose", 10 );
  geometry_msgs::Pose pose_msg;
  pose_msg.position.y = 1.12;

  auto subscriber = dynamic_cast<qml_ros_plugin::Subscriber *>(ros_wrapper.subscribe( "/test", 1 ));
  processSomeEvents();
  pub.publish( pose_msg );
  processSomeEvents();
  ASSERT_TRUE( subscriber->message().isValid());
  EXPECT_EQ( subscriber->message().toMap()["position"].toMap()["y"].toDouble(), 1.12 );

  ros_wrapper.setThreads( 0 ); // Disable async spinner
  pose_msg.position.y = 2.34;
  pub.publish( pose_msg );
  processSomeEvents(100);
  EXPECT_NE( subscriber->message().toMap()["position"].toMap()["y"].toDouble(), 2.34 ) << "Shouldn't have received that yet!";
  ros_wrapper.spinOnce();
  processSomeEvents();
  EXPECT_EQ( subscriber->message().toMap()["position"].toMap()["y"].toDouble(), 2.34 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}