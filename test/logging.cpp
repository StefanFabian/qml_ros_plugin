//
// Created by Stefan Fabian on 04.03.20.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/ros.h>

#include <rosgraph_msgs/Log.h>

#include <QCoreApplication>
#include <QJSEngine>
#include <ros/ros.h>

using namespace qml_ros_plugin;

bool waitFor( const std::function<bool()> &pred )
{
  int wait_count = 0;
  while ( ++wait_count < 10 )
  {
    if ( pred()) return true;
    QCoreApplication::processEvents();
    ros::spinOnce();
    ros::Duration( 0.05 ).sleep();
  }
  return false;
}

TEST( Logging, log )
{
  ros::NodeHandle nh;
  QJSEngine engine;
  auto *wrapper = new RosQmlSingletonWrapper;
  engine.newQObject( wrapper );

  EXPECT_EQ( static_cast<ros::console::Level>(ros_console_levels::Debug), ros::console::levels::Debug );
  EXPECT_EQ( static_cast<ros::console::Level>(ros_console_levels::Info), ros::console::levels::Info );
  EXPECT_EQ( static_cast<ros::console::Level>(ros_console_levels::Warn), ros::console::levels::Warn );
  EXPECT_EQ( static_cast<ros::console::Level>(ros_console_levels::Error), ros::console::levels::Error );
  EXPECT_EQ( static_cast<ros::console::Level>(ros_console_levels::Fatal), ros::console::levels::Fatal );

  std::vector<std::pair<std::string, int>> log;
  ros::Subscriber rosout_sub = nh.subscribe<rosgraph_msgs::Log>( "/rosout", 0,
                                                                 [ &log ]( const rosgraph_msgs::LogConstPtr &msg )
                                                                 {
                                                                   if ( msg->line != 0 ) return;
                                                                   log.emplace_back( msg->msg, msg->level );
                                                                 } );
  ASSERT_TRUE( wrapper->debug().isCallable());
  wrapper->debug().call( { QJSValue( "Debug Message" ) } );
  EXPECT_FALSE( waitFor( [ &log ]() { return !log.empty(); } )) << "Default level should not show debug messages.";
  ASSERT_TRUE( wrapper->console().setLoggerLevel( wrapper->console().defaultName(), ros_console_levels::Debug ))
            << "Failed to set logging level.";
  waitFor( []() { return false; } );

  QJSValue result = wrapper->debug().call( { QJSValue( "Debug Message" ) } );
  ASSERT_FALSE( result.isError());
  ASSERT_TRUE( waitFor( [ &log ]() { return !log.empty(); } ));
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Debug Message" );
  EXPECT_EQ( log[0].second, rosgraph_msgs::Log::DEBUG );
  log.clear();

  result = wrapper->info().call( { QJSValue( "Info Message" ) } );
  ASSERT_FALSE( result.isError());
  ASSERT_TRUE( waitFor( [ &log ]() { return !log.empty(); } ));
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Info Message" );
  EXPECT_EQ( log[0].second, rosgraph_msgs::Log::INFO );
  log.clear();

  result = wrapper->warn().call( { QJSValue( "Warn Message" ) } );
  ASSERT_FALSE( result.isError());
  ASSERT_TRUE( waitFor( [ &log ]() { return !log.empty(); } ));
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Warn Message" );
  EXPECT_EQ( log[0].second, rosgraph_msgs::Log::WARN );
  log.clear();

  result = wrapper->error().call( { QJSValue( "Error Message" ) } );
  ASSERT_FALSE( result.isError());
  ASSERT_TRUE( waitFor( [ &log ]() { return !log.empty(); } ));
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Error Message" );
  EXPECT_EQ( log[0].second, rosgraph_msgs::Log::ERROR );
  log.clear();

  result = wrapper->fatal().call( { QJSValue( "Fatal Message" ) } );
  ASSERT_FALSE( result.isError());
  ASSERT_TRUE( waitFor( [ &log ]() { return !log.empty(); } ));
  ASSERT_EQ( log.size(), 1U );
  EXPECT_EQ( log[0].first, "Fatal Message" );
  EXPECT_EQ( log[0].second, rosgraph_msgs::Log::FATAL );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_logging" );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}
