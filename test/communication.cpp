//
// Created by stefan on 05.11.19.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/publisher.h>
#include <qml_ros_plugin/ros.h>
#include <qml_ros_plugin/service.h>
#include <qml_ros_plugin/subscriber.h>
#include <qml_ros_plugin/tf_transform.h>
#include <qml_ros_plugin/tf_transform_listener.h>
#include <qml_ros_plugin/time.h>

#include <geometry_msgs/Pose.h>
#include <roscpp_tutorials/TwoInts.h>

#include <QCoreApplication>
#include <tf2_ros/transform_broadcaster.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

template<typename T>
struct MessageStorage
{
  std::vector<T> messages;

  void callback( T msg )
  {
    messages.push_back( msg );
  }
};

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

TEST( Communication, publisher )
{
  RosQml::getInstance().setThreads( 0 ); // Disable spinner
  RosQmlSingletonWrapper wrapper;
  ros::NodeHandle nh;
  auto pub_singleton_private_ns = dynamic_cast<qml_ros_plugin::Publisher *>(wrapper.advertise( "~private_ns",
                                                                                               "geometry_msgs/Pose",
                                                                                               "test",
                                                                                               10 ));
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_private_ns->topic(), QString( "/communication/private_ns/test" ))
          << pub_singleton_private_ns->topic().toStdString();
  EXPECT_EQ( pub_singleton_private_ns->queueSize(), 10U );
  EXPECT_EQ( pub_singleton_private_ns->type(), QString( "geometry_msgs/Pose" ));
  EXPECT_EQ( pub_singleton_private_ns->isLatched(), false );
  EXPECT_EQ( pub_singleton_private_ns->getNumSubscribers(), 0U );
  MessageStorage<geometry_msgs::Pose> pub_singleton_private_ns_storage;
  ros::Subscriber subscriber_private_ns = nh.subscribe<geometry_msgs::Pose>( "/communication/private_ns/test", 10,
                                                                             &MessageStorage<geometry_msgs::Pose>::callback,
                                                                             &pub_singleton_private_ns_storage );
  QCoreApplication::processEvents();
  ros::spinOnce();
  EXPECT_EQ( pub_singleton_private_ns->getNumSubscribers(), 1U );
  pub_singleton_private_ns->publish( {{ "position", QVariantMap{{ "x", 1.2 }}}} );
  if ( !waitFor( [ & ]() { return !pub_singleton_private_ns_storage.messages.empty(); } ))
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_private_ns_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_private_ns_storage.messages[0].position.x, 1.2 );
  delete pub_singleton_private_ns;

  auto pub_singleton_private_ns_glob = dynamic_cast<qml_ros_plugin::Publisher *>(wrapper.advertise( "~private_ns",
                                                                                                    "geometry_msgs/Pose",
                                                                                                    "/pose",
                                                                                                    10 ));
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_private_ns_glob->topic(), QString( "/pose" ))
          << pub_singleton_private_ns_glob->topic().toStdString();
  MessageStorage<geometry_msgs::Pose> pub_singleton_private_ns_glob_storage;
  ros::Subscriber subscriber_private_ns_glob = nh.subscribe<geometry_msgs::Pose>( "/pose", 10,
                                                                                  &MessageStorage<geometry_msgs::Pose>::callback,
                                                                                  &pub_singleton_private_ns_glob_storage );
  ros::Duration( 0.5 ).sleep();
  EXPECT_TRUE( pub_singleton_private_ns_glob_storage.messages.empty());
  pub_singleton_private_ns_glob->publish( {{ "position", QVariantMap{{ "y", 1.3 }}}} );
  if ( !waitFor( [ & ]() { return !pub_singleton_private_ns_glob_storage.messages.empty(); } ))
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_private_ns_glob_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_private_ns_glob_storage.messages[0].position.y, 1.3 );
  delete pub_singleton_private_ns_glob;

  auto pub_singleton_ns_glob = dynamic_cast<qml_ros_plugin::Publisher *>(wrapper.advertise( "geometry_msgs/Pose",
                                                                                            "other_pose",
                                                                                            10 ));
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_ns_glob->topic(), QString( "/other_pose" ))
          << pub_singleton_ns_glob->topic().toStdString();
  MessageStorage<geometry_msgs::Pose> pub_singleton_ns_glob_storage;
  ros::Subscriber subscriber_ns_glob = nh.subscribe<geometry_msgs::Pose>( "/other_pose", 10,
                                                                          &MessageStorage<geometry_msgs::Pose>::callback,
                                                                          &pub_singleton_ns_glob_storage );
  ros::Duration( 0.5 ).sleep();
  EXPECT_TRUE( pub_singleton_ns_glob_storage.messages.empty());
  pub_singleton_ns_glob->publish( {{ "position", QVariantMap{{ "y", 1.3 }}}} );
  if ( !waitFor( [ & ]() { return !pub_singleton_ns_glob_storage.messages.empty(); } ))
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_ns_glob_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_ns_glob_storage.messages[0].position.y, 1.3 );
  delete pub_singleton_ns_glob;
}

TEST( Communication, subscriber )
{
  RosQmlSingletonWrapper wrapper;
  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~private_ns" );
  ros::Publisher pub_pns = pnh.advertise<geometry_msgs::Pose>( "test", 10 );
  EXPECT_EQ( pub_pns.getTopic(), "/communication/private_ns/test" );
  auto subscriber_pns = dynamic_cast<qml_ros_plugin::Subscriber *>(wrapper.subscribe( "~private_ns", "test", 1 ));
  QCoreApplication::processEvents();
  ros::spinOnce();
  EXPECT_TRUE( subscriber_pns->isInitialized());
  EXPECT_TRUE( subscriber_pns->running());
  EXPECT_EQ( subscriber_pns->getNumPublishers(), 1U );
  EXPECT_EQ( subscriber_pns->ns(), QString( "/communication/private_ns" )) << subscriber_pns->ns().toStdString();
  EXPECT_EQ( subscriber_pns->queueSize(), 1U );
  EXPECT_EQ( subscriber_pns->topic(), QString( "/communication/private_ns/test" ))
          << subscriber_pns->topic().toStdString();
  if ( !waitFor( [ & ]() { return pub_pns.getNumSubscribers() > 0; } ))
    FAIL() << "Timout while waiting for subscriber num increasing.";
  geometry_msgs::Pose pose;
  pose.position.x = 2.34;
  pub_pns.publish( pose );
  if ( !waitFor( [ & ]() { return subscriber_pns->message().isValid(); } ))
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x, subscriber_pns->message().toMap()["position"].toMap()["x"].toDouble());
  delete subscriber_pns;

  ros::Publisher pub_pns_glob = pnh.advertise<geometry_msgs::Pose>( "/pose", 10 );
  EXPECT_EQ( pub_pns_glob.getTopic(), "/pose" );
  qml_ros_plugin::Subscriber subscriber_pns_glob;
  subscriber_pns_glob.setNs( "~private_ns" );
  subscriber_pns_glob.setTopic( "/pose" );
  subscriber_pns_glob.setQueueSize( 5 );
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_pns_glob.isInitialized());
  EXPECT_EQ( subscriber_pns_glob.queueSize(), 5U );
  EXPECT_EQ( subscriber_pns_glob.topic(), QString( "/pose" )) << subscriber_pns_glob.topic().toStdString();
  if ( !waitFor( [ & ]() { return pub_pns_glob.getNumSubscribers() > 0; } ))
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.y = 3.44;
  pub_pns_glob.publish( pose );
  if ( !waitFor( [ & ]() { return subscriber_pns_glob.message().isValid(); } ))
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x, subscriber_pns_glob.message().toMap()["position"].toMap()["x"].toDouble());
  EXPECT_DOUBLE_EQ( pose.position.y, subscriber_pns_glob.message().toMap()["position"].toMap()["y"].toDouble());

  ros::Publisher pub_ns = nh.advertise<geometry_msgs::Pose>( "/other_pose", 10 );
  EXPECT_EQ( pub_ns.getTopic(), "/other_pose" );
  auto subscriber_ns = dynamic_cast<qml_ros_plugin::Subscriber *>(wrapper.subscribe( "/other_pose", 0 ));
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_ns->isInitialized());
  EXPECT_EQ( subscriber_ns->ns(), QString( "/" )) << subscriber_ns->ns().toStdString();
  EXPECT_EQ( subscriber_ns->queueSize(), 0U );
  EXPECT_EQ( subscriber_ns->topic(), QString( "/other_pose" )) << subscriber_ns->topic().toStdString();
  if ( !waitFor( [ & ]() { return pub_ns.getNumSubscribers() > 0; } ))
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.z = 5.16;
  pub_ns.publish( pose );
  if ( !waitFor( [ & ]() { return subscriber_ns->message().isValid(); } ))
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x, subscriber_ns->message().toMap()["position"].toMap()["x"].toDouble());
  EXPECT_DOUBLE_EQ( pose.position.y, subscriber_ns->message().toMap()["position"].toMap()["y"].toDouble());
  EXPECT_DOUBLE_EQ( pose.position.z, subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble());
  EXPECT_EQ( subscriber_ns->messageType(), QString( "geometry_msgs/Pose" ))
          << subscriber_ns->messageType().toStdString();
  subscriber_ns->setRunning( false );
  EXPECT_FALSE( subscriber_ns->running());
  pose.position.z = 1.0;
  pub_ns.publish( pose );
  subscriber_ns->setRunning( true );
  EXPECT_TRUE( subscriber_ns->running());
  if ( waitFor( [ & ]()
                {
                  return std::abs( subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() - 1.0 ) < 1E-4;
                } ))
    FAIL() << "Shouldn't have received the message that was published while the subscriber wasn't running.";
}

TEST( Communication, service )
{
  qml_ros_plugin::Service service;
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue( &queue );
  ros::AsyncSpinner spinner( 1, &queue );
  bool service_called = false;
  ros::ServiceServer server = nh.advertiseService<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>(
    "/service",
    boost::function<bool( roscpp_tutorials::TwoIntsRequest &, roscpp_tutorials::TwoIntsResponse & )>(
      [ & ]( roscpp_tutorials::TwoIntsRequest &req, roscpp_tutorials::TwoIntsResponse &resp )
      {
        service_called = true;
        resp.sum = req.a + req.b;
        return true;
      } ));
  spinner.start();
  QVariant result = service.call( "/service", "roscpp_tutorials/TwoInts", {{ "a", 1 },
                                                                           { "b", 3 }} );
  EXPECT_TRUE( service_called ) << "Service was not called!";
  ASSERT_EQ( result.type(), QVariant::Map )
            << "Result was not world. Did the request fail? "
            << (result.type() == QVariant::Bool && !result.toBool() ? "Yes" : "No");
  EXPECT_EQ( result.toMap()["sum"].toInt(), 4 )
          << "Contains 'sum'? " << (result.toMap().contains( "sum" ) ? "Yes" : "No");
}

TEST( Communication, tfTransform )
{
  qml_ros_plugin::TfTransformListenerWrapper wrapper;
  qml_ros_plugin::TfTransform transform;
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = "world";
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.child_frame_id = "base";
  transform_stamped.transform.translation.x = 1;
  transform_stamped.transform.translation.y = 2.4;
  transform_stamped.transform.translation.z = 0.3;
  transform_stamped.transform.rotation.w = 0.8432;
  transform_stamped.transform.rotation.x = 0.068;
  transform_stamped.transform.rotation.y = 0.4821;
  transform_stamped.transform.rotation.z = -0.2281;
  tf2_ros::TransformBroadcaster broadcaster;
  EXPECT_TRUE( transform.active());
  ASSERT_TRUE( transform.message().contains( "valid" ));
  EXPECT_FALSE( transform.message()["valid"].toBool());
  EXPECT_FALSE( transform.valid());
  broadcaster.sendTransform( transform_stamped );
  ros::spinOnce();
  transform.setSourceFrame( "base" );
  EXPECT_EQ( transform.sourceFrame(), QString( "base" )) << transform.sourceFrame().toStdString();
  transform.setTargetFrame( "world" );
  EXPECT_EQ( transform.targetFrame(), QString( "world" )) << transform.targetFrame().toStdString();
  if ( !waitFor( [ & ]()
                 {
                   broadcaster.sendTransform( transform_stamped );
                   return transform.valid();
                 } ))
    FAIL() << "Failed to get transform in time!";
  ASSERT_TRUE( transform.valid());
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["x"].toDouble(), 1 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["y"].toDouble(), 2.4 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["z"].toDouble(), 0.3 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["w"].toDouble(), 0.8432 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["x"].toDouble(), 0.068 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["y"].toDouble(), 0.4821 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["z"].toDouble(), -0.2281 );

  transform.setActive( false );
  EXPECT_FALSE( transform.active());
  QCoreApplication::processEvents();
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.transform.translation.x = 3.14;
  broadcaster.sendTransform( transform_stamped );
  waitFor( []() { return false; } );
  EXPECT_NE( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(), 3.14 )
          << "Shouldn't have received that transform!";
  QDateTime last_transform_datetime = QDateTime::currentDateTime();
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.transform.translation.x = 0.577;
  broadcaster.sendTransform( transform_stamped );
  transform.setActive( true );
  EXPECT_TRUE( transform.active());
  if ( !waitFor( [ & ]()
                 {
                   broadcaster.sendTransform( transform_stamped );
                   return std::abs( transform.translation().toMap()["x"].toDouble() - 0.577 ) < 1E-4;
                 } ))
    FAIL() << "Did not receive transform in time: " << transform.translation().toMap()["x"].toDouble();
  EXPECT_DOUBLE_EQ( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(), 0.577 );

  QVariant can_transform = wrapper.canTransform( "base", "world" ).toBool();
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool());
  can_transform = wrapper.canTransform( "millionaire", "inheritance", QDateTime(), 500 );
  EXPECT_TRUE( can_transform.type() != QVariant::Bool || !can_transform.toBool())
          << "Inheritance shouldn't be able to transform to millionaire!";
  can_transform = wrapper.canTransform( "base", last_transform_datetime,
                                        "world", last_transform_datetime, "world" );
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool());

  EXPECT_TRUE( mapAndMessageEqual( wrapper.lookUpTransform( "world", "base", QDateTime(), 500 ), transform_stamped ));
  QVariantMap lookup_result = wrapper.lookUpTransform( "world", rosToQmlTime( transform_stamped.header.stamp ),
                                                       "base", rosToQmlTime( transform_stamped.header.stamp ),
                                                       "world" );
  ASSERT_TRUE( lookup_result["valid"].toBool())
            << lookup_result["exception"].toString().toStdString() << std::endl
            << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ));
  lookup_result = wrapper.lookUpTransform( "world", rosToQmlTime( transform_stamped.header.stamp ),
                                           "base", rosToQmlTime( transform_stamped.header.stamp ), "world", 500 );
  ASSERT_TRUE( lookup_result["valid"].toBool())
            << lookup_result["exception"].toString().toStdString() << std::endl
            << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ));

  EXPECT_EQ( wrapper.canTransform( "world", QDateTime::currentDateTime(),
                                   "base", QDateTime::currentDateTime(), "world" ).type(),
             QVariant::String );
  EXPECT_EQ( wrapper.canTransform( "world", QDateTime::currentDateTime(),
                                   "base", QDateTime::currentDateTime(), "world", 500 ).type(),
             QVariant::String );
  EXPECT_EQ( wrapper.lookUpTransform( "world", QDateTime::currentDateTime(),
                                      "base", QDateTime::currentDateTime(), "world" )["exception"].toString(),
             QString( "ExtrapolationException" ));
  EXPECT_EQ( wrapper.lookUpTransform( "world", "base", QDateTime::currentDateTime())["exception"].toString(),
             QString( "ExtrapolationException" ));
  EXPECT_EQ( wrapper.lookUpTransform( "world", QDateTime(), "equality", QDateTime(), "world" )["exception"].toString(),
             QString( "LookupException" ));
  EXPECT_EQ( wrapper.lookUpTransform( "world", "equality" )["exception"].toString(), QString( "LookupException" ));
  EXPECT_EQ(
    wrapper.lookUpTransform( "world", QDateTime(), "billionaires", QDateTime(), "world" )["exception"].toString(),
    QString( "ConnectivityException" ));
  EXPECT_EQ( wrapper.lookUpTransform( "world", "politics" )["exception"].toString(),
             QString( "ConnectivityException" ));
  // Currently no idea how to test InvalidArgumentException since invalid quaternions get rejected when publishing already
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  ros::init( argc, argv, "communication" );
  return RUN_ALL_TESTS();
}

