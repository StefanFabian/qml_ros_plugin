//
// Created by stefan on 31.10.19.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/publisher.h>
#include <qml_ros_plugin/ros.h>
#include <qml_ros_plugin/tf_transform_listener.h>
#include <qml_ros_plugin/time.h>

#include <QCoreApplication>
#include <ros/ros.h>

class MockRos : public QObjectRos
{
public:
  void onRosInitialized() override
  {
    initialized = true;
  }

  void onRosShutdown() override
  {
    initialized = false;
  }

  bool initialized = false;
};

TEST( RosLifeCycle, testLifeCycle )
{
  int argc = 0;
  QCoreApplication app( argc, nullptr );
  qml_ros_plugin::RosQmlSingletonWrapper ros_wrapper;
  qml_ros_plugin::TimeSingleton time;
  qml_ros_plugin::TfTransformListenerWrapper tf_wrapper;
  qml_ros_plugin::NodeHandle::Ptr qml_nh = std::make_shared<qml_ros_plugin::NodeHandle>( "test_ns" );
  EXPECT_FALSE( ros_wrapper.isInitialized());
  EXPECT_FALSE( RosQml::getInstance().isInitialized());
  EXPECT_FALSE( qml_ros_plugin::TfTransformListener::getInstance().isInitialized());
  EXPECT_EQ(
    tf_wrapper.lookUpTransform( "frame2", "frame1" )["exception"].toString(),
    QString( "Uninitialized" ));
  EXPECT_EQ(
    tf_wrapper.lookUpTransform( "frame2", QDateTime(), "frame1", QDateTime(), "frame1" )["exception"].toString(),
    QString( "Uninitialized" ));
  auto publisher_before_init = dynamic_cast<qml_ros_plugin::Publisher *>(qml_nh->advertise( "geometry_msgs/PoseStamped",
                                                                                            "/pose_stamped", 10 ));
  ASSERT_NE( publisher_before_init, nullptr );
  MockRos mock_ros;
  QCoreApplication::processEvents();
  EXPECT_FALSE( time.isValid());
  EXPECT_FALSE( qml_nh->isInitialized());
  EXPECT_FALSE( qml_nh->isReady());
  EXPECT_FALSE( publisher_before_init->isAdvertised());
  QCoreApplication::processEvents();
  EXPECT_FALSE( mock_ros.initialized );

  // Initialize ROS
  ros_wrapper.init( "test_ros_life_cycle" );
  ros::NodeHandle nh;
  int wait_count = 0;
  while ( true )
  {
    if ( ++wait_count > 10 ) FAIL() << "Initialization did not finish in time!";
    if ( time.isValid()) break;
    QCoreApplication::processEvents();
    ros::Duration( 0.05 ).sleep();
  }
  ASSERT_TRUE( ros::isInitialized());
  EXPECT_TRUE( ros_wrapper.isInitialized());
  EXPECT_TRUE( RosQml::getInstance().isInitialized());
  EXPECT_TRUE( ros_wrapper.ok());
  EXPECT_TRUE( qml_ros_plugin::TfTransformListener::getInstance().isInitialized());
  EXPECT_FALSE( time.isSimTime());
  EXPECT_TRUE( time.isSystemTime());
  EXPECT_TRUE( qml_nh->isInitialized());
  QCoreApplication::processEvents();

  EXPECT_TRUE( time.isValid());
  EXPECT_TRUE( time.now().value<Time>().toJSDate().isValid());
  EXPECT_TRUE( qml_nh->isReady());
  EXPECT_TRUE( publisher_before_init->isAdvertised());
  auto publisher_after_init = dynamic_cast<qml_ros_plugin::Publisher *>(qml_nh->advertise( "geometry_msgs/PoseStamped",
                                                                                           "/pose_stamped2", 10 ));
  EXPECT_TRUE( publisher_after_init->isAdvertised());
  QCoreApplication::processEvents();
  EXPECT_EQ( qml_nh->nodeHandle().getNamespace(), "/test_ns" );
  EXPECT_EQ( qml_nh->ns(), QString( "/test_ns" ));
  EXPECT_TRUE( mock_ros.initialized );
  ros::shutdown();
  for ( int i = 0; i < 5; ++i )
  {
    QCoreApplication::processEvents();
    ros::Duration( 0.05 ).sleep();
  }
  EXPECT_FALSE( mock_ros.initialized );
  EXPECT_FALSE( qml_ros_plugin::TfTransformListener::getInstance().isInitialized());
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
