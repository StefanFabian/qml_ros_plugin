//
// Created by stefan on 05.11.19.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/action_client.h>
#include <qml_ros_plugin/goal_handle.h>
#include <qml_ros_plugin/publisher.h>
#include <qml_ros_plugin/ros.h>
#include <qml_ros_plugin/service.h>
#include <qml_ros_plugin/subscriber.h>
#include <qml_ros_plugin/tf_transform.h>
#include <qml_ros_plugin/tf_transform_listener.h>
#include <qml_ros_plugin/time.h>

#include <geometry_msgs/Pose.h>
#include <roscpp_tutorials/TwoInts.h>
#include <std_srvs/Empty.h>

#include <QCoreApplication>
#include <QJSEngine>
#include <QSignalSpy>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

template<typename T>
struct MessageStorage {
  std::vector<T> messages;

  void callback( T msg ) { messages.push_back( msg ); }
};

void processEvents()
{
  QCoreApplication::processEvents();
  ros::spinOnce();
  RosQml::getInstance().spinOnce();
}

//! @param wait_count Max time to wait in increments of 33 ms
bool waitFor( const std::function<bool()> &pred, int wait_count = 10 )
{
  while ( --wait_count > 0 ) {
    if ( pred() )
      return true;
    processEvents();
    ros::Duration( 0.033 ).sleep();
  }
  return false;
}

TEST( Communication, publisher )
{
  RosQmlSingletonWrapper wrapper;
  ros::NodeHandle nh;
  auto pub_singleton_private_ns = dynamic_cast<qml_ros_plugin::Publisher *>(
      wrapper.advertise( "~private_ns", "geometry_msgs/Pose", "test", 10 ) );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_private_ns->topic(), QString( "/communication/private_ns/test" ) )
      << pub_singleton_private_ns->topic().toStdString();
  EXPECT_EQ( pub_singleton_private_ns->queueSize(), 10U );
  EXPECT_EQ( pub_singleton_private_ns->type(), QString( "geometry_msgs/Pose" ) );
  EXPECT_EQ( pub_singleton_private_ns->isLatched(), false );
  EXPECT_EQ( pub_singleton_private_ns->getNumSubscribers(), 0U );
  MessageStorage<geometry_msgs::Pose> pub_singleton_private_ns_storage;
  ros::Subscriber subscriber_private_ns = nh.subscribe<geometry_msgs::Pose>(
      "/communication/private_ns/test", 10, &MessageStorage<geometry_msgs::Pose>::callback,
      &pub_singleton_private_ns_storage );
  QCoreApplication::processEvents();
  ros::spinOnce();
  EXPECT_EQ( pub_singleton_private_ns->getNumSubscribers(), 1U );
  pub_singleton_private_ns->publish( { { "position", QVariantMap{ { "x", 1.2 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_private_ns_storage.messages.empty(); } ) )
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_private_ns_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_private_ns_storage.messages[0].position.x, 1.2 );
  delete pub_singleton_private_ns;

  auto pub_singleton_private_ns_glob = dynamic_cast<qml_ros_plugin::Publisher *>(
      wrapper.advertise( "~private_ns", "geometry_msgs/Pose", "/pose", 10 ) );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_private_ns_glob->topic(), QString( "/pose" ) )
      << pub_singleton_private_ns_glob->topic().toStdString();
  MessageStorage<geometry_msgs::Pose> pub_singleton_private_ns_glob_storage;
  ros::Subscriber subscriber_private_ns_glob =
      nh.subscribe<geometry_msgs::Pose>( "/pose", 10, &MessageStorage<geometry_msgs::Pose>::callback,
                                         &pub_singleton_private_ns_glob_storage );
  ros::Duration( 0.5 ).sleep();
  EXPECT_TRUE( pub_singleton_private_ns_glob_storage.messages.empty() );
  pub_singleton_private_ns_glob->publish( { { "position", QVariantMap{ { "y", 1.3 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_private_ns_glob_storage.messages.empty(); } ) )
    FAIL() << "Timeout while waiting for message!";
  ASSERT_EQ( pub_singleton_private_ns_glob_storage.messages.size(), 1UL );
  EXPECT_EQ( pub_singleton_private_ns_glob_storage.messages[0].position.y, 1.3 );
  delete pub_singleton_private_ns_glob;

  auto pub_singleton_ns_glob = dynamic_cast<qml_ros_plugin::Publisher *>(
      wrapper.advertise( "geometry_msgs/Pose", "other_pose", 10 ) );
  QCoreApplication::processEvents();
  EXPECT_EQ( pub_singleton_ns_glob->topic(), QString( "/other_pose" ) )
      << pub_singleton_ns_glob->topic().toStdString();
  MessageStorage<geometry_msgs::Pose> pub_singleton_ns_glob_storage;
  ros::Subscriber subscriber_ns_glob = nh.subscribe<geometry_msgs::Pose>(
      "/other_pose", 10, &MessageStorage<geometry_msgs::Pose>::callback,
      &pub_singleton_ns_glob_storage );
  ros::Duration( 0.5 ).sleep();
  EXPECT_TRUE( pub_singleton_ns_glob_storage.messages.empty() );
  pub_singleton_ns_glob->publish( { { "position", QVariantMap{ { "y", 1.3 } } } } );
  if ( !waitFor( [&]() { return !pub_singleton_ns_glob_storage.messages.empty(); } ) )
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
  auto subscriber_pns =
      dynamic_cast<qml_ros_plugin::Subscriber *>( wrapper.subscribe( "~private_ns", "test", 1 ) );
  QCoreApplication::processEvents();
  ros::spinOnce();
  EXPECT_TRUE( subscriber_pns->isInitialized() );
  EXPECT_TRUE( subscriber_pns->running() );
  EXPECT_EQ( subscriber_pns->getNumPublishers(), 1U );
  EXPECT_EQ( subscriber_pns->ns(), QString( "/communication/private_ns" ) )
      << subscriber_pns->ns().toStdString();
  EXPECT_EQ( subscriber_pns->queueSize(), 1U );
  EXPECT_EQ( subscriber_pns->topic(), QString( "/communication/private_ns/test" ) )
      << subscriber_pns->topic().toStdString();
  if ( !waitFor( [&]() { return pub_pns.getNumSubscribers() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  geometry_msgs::Pose pose;
  pose.position.x = 2.34;
  pub_pns.publish( pose );
  if ( !waitFor( [&]() { return subscriber_pns->message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_pns->message().toMap()["position"].toMap()["x"].toDouble() );
  delete subscriber_pns;

  ros::Publisher pub_pns_glob = pnh.advertise<geometry_msgs::Pose>( "/pose", 10 );
  EXPECT_EQ( pub_pns_glob.getTopic(), "/pose" );
  qml_ros_plugin::Subscriber subscriber_pns_glob;
  subscriber_pns_glob.setNs( "~private_ns" );
  subscriber_pns_glob.setTopic( "/pose" );
  subscriber_pns_glob.setQueueSize( 5 );
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_pns_glob.isInitialized() );
  EXPECT_EQ( subscriber_pns_glob.queueSize(), 5U );
  EXPECT_EQ( subscriber_pns_glob.topic(), QString( "/pose" ) )
      << subscriber_pns_glob.topic().toStdString();
  if ( !waitFor( [&]() { return pub_pns_glob.getNumSubscribers() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.y = 3.44;
  pub_pns_glob.publish( pose );
  if ( !waitFor( [&]() { return subscriber_pns_glob.message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_pns_glob.message().toMap()["position"].toMap()["x"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.y,
                    subscriber_pns_glob.message().toMap()["position"].toMap()["y"].toDouble() );

  ros::Publisher pub_ns = nh.advertise<geometry_msgs::Pose>( "/other_pose", 10 );
  EXPECT_EQ( pub_ns.getTopic(), "/other_pose" );
  auto subscriber_ns =
      dynamic_cast<qml_ros_plugin::Subscriber *>( wrapper.subscribe( "/other_pose", 0 ) );
  QCoreApplication::processEvents();
  EXPECT_TRUE( subscriber_ns->isInitialized() );
  EXPECT_EQ( subscriber_ns->ns(), QString( "/" ) ) << subscriber_ns->ns().toStdString();
  EXPECT_EQ( subscriber_ns->queueSize(), 0U );
  EXPECT_EQ( subscriber_ns->topic(), QString( "/other_pose" ) )
      << subscriber_ns->topic().toStdString();
  if ( !waitFor( [&]() { return pub_ns.getNumSubscribers() > 0; } ) )
    FAIL() << "Timout while waiting for subscriber num increasing.";
  pose.position.z = 5.16;
  pub_ns.publish( pose );
  if ( !waitFor( [&]() { return subscriber_ns->message().isValid(); } ) )
    FAIL() << "Did not receive message in time.";
  EXPECT_DOUBLE_EQ( pose.position.x,
                    subscriber_ns->message().toMap()["position"].toMap()["x"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.y,
                    subscriber_ns->message().toMap()["position"].toMap()["y"].toDouble() );
  EXPECT_DOUBLE_EQ( pose.position.z,
                    subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() );
  EXPECT_EQ( subscriber_ns->messageType(), QString( "geometry_msgs/Pose" ) )
      << subscriber_ns->messageType().toStdString();
  subscriber_ns->setRunning( false );
  EXPECT_FALSE( subscriber_ns->running() );
  pose.position.z = 1.0;
  pub_ns.publish( pose );
  subscriber_ns->setRunning( true );
  EXPECT_TRUE( subscriber_ns->running() );
  if ( waitFor( [&]() {
         return std::abs( subscriber_ns->message().toMap()["position"].toMap()["z"].toDouble() -
                          1.0 ) < 1E-4;
       } ) )
    FAIL() << "Shouldn't have received the message that was published while the subscriber wasn't "
              "running.";
}

TEST( Communication, waitForMessage )
{
  QJSEngine engine;
  RosQmlSingletonWrapper *pwrapper = new RosQmlSingletonWrapper();
  QJSValue jswrapper = engine.newQObject(pwrapper);
  RosQmlSingletonWrapper &wrapper = *pwrapper;
  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~private_ns" );
  ros::Publisher pub_pns = pnh.advertise<geometry_msgs::Pose>( "test", 10 );
  EXPECT_EQ( pub_pns.getTopic(), "/communication/private_ns/test" );
  geometry_msgs::Pose pose;
  pose.position.x = 2.34;
  QJSValue obj = engine.newObject();
  // Check that callback is called using a watcher object that will hold the passed response
  QJSValue callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  wrapper.waitForMessageAsync( "/communication/private_ns/test", callback );
  ASSERT_TRUE( waitFor( [&pub_pns]() { return pub_pns.getNumSubscribers() > 0; } ) );
  ASSERT_FALSE( obj.hasProperty( "result" ) );
  // Publish message
  pub_pns.publish( pose );
  waitFor( [&obj]() { return obj.hasProperty( "result" ); }, 60 );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  QVariant result = obj.property( "result" ).toVariant();
  EXPECT_DOUBLE_EQ( pose.position.x, result.toMap()["position"].toMap()["x"].toDouble() );

  // With timeout but receiving a message within timeout
  pose.position.x = 3.14;
  obj = engine.newObject();
  callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );
  wrapper.waitForMessageAsync( "/communication/private_ns/test", 500, callback );
  ASSERT_TRUE( waitFor( [&pub_pns]() { return pub_pns.getNumSubscribers() > 0; } ) );
  ASSERT_FALSE( obj.hasProperty( "result" ) );
  // Publish message
  pub_pns.publish( pose );
  waitFor( [&obj]() { return obj.hasProperty( "result" ); }, 60 );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  result = obj.property( "result" ).toVariant();
  EXPECT_DOUBLE_EQ( pose.position.x, result.toMap()["position"].toMap()["x"].toDouble() );

  // Timeouting
  obj = engine.newObject();
  callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );
  wrapper.waitForMessageAsync( "/communication/private_ns/test", 500, callback );
  processEvents();
  ASSERT_FALSE( obj.hasProperty( "result" ) );
  ASSERT_TRUE( waitFor( [&obj]() { return obj.hasProperty( "result" ); }, 60 ) );
  result = obj.property( "result" ).toVariant();
  ASSERT_FALSE( result.isValid() );
}

TEST( Communication, queryTopics )
{
  ros::NodeHandle nh;
  ros::Publisher pub1 = nh.advertise<geometry_msgs::Pose>( "/query_topics/pose1", 10 );
  ros::Publisher pub2 = nh.advertise<geometry_msgs::Vector3>( "/query_topics/vector3", 10 );
  ros::Publisher pub3 = nh.advertise<geometry_msgs::Point>( "/query_topics/point1", 10 );
  ros::Publisher pub4 = nh.advertise<geometry_msgs::Point>( "/query_topics/point2", 10 );
  ros::Publisher pub5 = nh.advertise<geometry_msgs::Pose>( "/query_topics/pose2", 10 );
  ros::Publisher pub6 = nh.advertise<geometry_msgs::Pose>( "/query_topics/pose3", 10 );
  RosQmlSingletonWrapper wrapper;
  waitFor( []() { return false; } ); // Wait some time
  QStringList topics = wrapper.queryTopics();
  std::string debug_topics;
  for ( const QString &topic : topics ) debug_topics += '\n' + topic.toStdString();
  for ( const QString &topic :
        QStringList{ "/query_topics/pose1", "/query_topics/vector3", "/query_topics/point1",
                     "/query_topics/point2", "/query_topics/pose2", "/query_topics/pose3" } ) {
    EXPECT_TRUE( topics.contains( topic ) )
        << topic.toStdString() << " is not in topics." << std::endl
        << "Declared topics:" << debug_topics;
  }
  topics = wrapper.queryTopics( "geometry_msgs/Point" );
  for ( const QString &topic : QStringList{ "/query_topics/point1", "/query_topics/point2" } ) {
    EXPECT_TRUE( topics.contains( topic ) )
        << topic.toStdString() << " is not in topics of type Point";
  }
  topics = wrapper.queryTopics( "geometry_msgs/Pose" );
  for ( const QString &topic :
        QStringList{ "/query_topics/pose1", "/query_topics/pose2", "/query_topics/pose3" } ) {
    EXPECT_TRUE( topics.contains( topic ) )
        << topic.toStdString() << " is not in topics of type Pose";
  }
  QList<TopicInfo> topic_info = wrapper.queryTopicInfo();
  for ( const QPair<QString, QString> &pair :
        QList<QPair<QString, QString>>{ { "/query_topics/pose1", "geometry_msgs/Pose" },
                                        { "/query_topics/vector3", "geometry_msgs/Vector3" },
                                        { "/query_topics/point1", "geometry_msgs/Point" },
                                        { "/query_topics/point2", "geometry_msgs/Point" },
                                        { "/query_topics/pose2", "geometry_msgs/Pose" },
                                        { "/query_topics/pose3", "geometry_msgs/Pose" } } ) {
    bool found = false;
    debug_topics = "";
    for ( const TopicInfo &info : topic_info ) {
      debug_topics += '\n' + info.name().toStdString();
      if ( info.name() != pair.first )
        continue;
      EXPECT_EQ( info.datatype(), pair.second );
      found = true;
    }
    EXPECT_TRUE( found ) << "Did not find " << pair.first.toStdString() << " in topic types."
                         << std::endl
                         << "Topics:" << debug_topics;
  }

  EXPECT_EQ( wrapper.queryTopicType( "/query_topics/point1" ), "geometry_msgs/Point" );
  EXPECT_EQ( wrapper.queryTopicType( "/query_topics/pose2" ), "geometry_msgs/Pose" );
  EXPECT_TRUE( wrapper.queryTopicType( "/query_topics/pose20" ).isEmpty() )
      << wrapper.queryTopicType( "/query_topics/pose20" ).toStdString();
}

TEST( Communication, serviceCall )
{
  qml_ros_plugin::Service service;
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue( &queue );
  ros::AsyncSpinner spinner( 1, &queue );
  bool service_called = false;
  ros::ServiceServer server =
      nh.advertiseService<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>(
          "/service",
          boost::function<bool( roscpp_tutorials::TwoIntsRequest &, roscpp_tutorials::TwoIntsResponse & )>(
              [&]( roscpp_tutorials::TwoIntsRequest &req, roscpp_tutorials::TwoIntsResponse &resp ) {
                service_called = true;
                resp.sum = req.a + req.b;
                return true;
              } ) );
  spinner.start();
  QVariant result =
      service.call( "/service", "roscpp_tutorials/TwoInts", { { "a", 1 }, { "b", 3 } } );
  EXPECT_TRUE( service_called ) << "Service was not called!";
  ASSERT_EQ( result.type(), QVariant::Map )
      << "Result was not map. Did the request fail? "
      << ( result.type() == QVariant::Bool && !result.toBool() ? "Yes" : "No" );
  EXPECT_EQ( result.toMap()["sum"].toInt(), 4 )
      << "Contains 'sum'? " << ( result.toMap().contains( "sum" ) ? "Yes" : "No" );
}

TEST( Communication, serviceCallAsync )
{
  QJSEngine engine;
  QJSValue service_js = engine.newQObject( new qml_ros_plugin::Service );
  auto *service = dynamic_cast<qml_ros_plugin::Service *>( service_js.toQObject() );
  ASSERT_NE( service, nullptr );
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  nh.setCallbackQueue( &queue );
  ros::AsyncSpinner spinner( 1, &queue );
  bool service_called = false;
  bool returned = false;
  ros::ServiceServer server =
      nh.advertiseService<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>(
          "/service",
          boost::function<bool( roscpp_tutorials::TwoIntsRequest &, roscpp_tutorials::TwoIntsResponse & )>(
              [&]( roscpp_tutorials::TwoIntsRequest &req, roscpp_tutorials::TwoIntsResponse &resp ) {
                service_called = true;
                ros::Duration( 1 ).sleep();
                resp.sum = req.a + req.b;
                returned = true;
                return true;
              } ) );
  spinner.start();
  QJSValue obj = engine.newObject();
  // Check that callback is called using a watcher object that will hold the passed response
  QJSValue callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  service->callAsync( "/service", "roscpp_tutorials/TwoInts", { { "a", 1 }, { "b", 3 } }, callback );
  ASSERT_TRUE( !returned );
  waitFor( [&returned]() { return returned; }, 60 );
  ASSERT_TRUE( returned );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  QVariant result = obj.property( "result" ).toVariant();
  EXPECT_TRUE( service_called ) << "Service was not called!";
  ASSERT_EQ( result.type(), QVariant::Map )
      << "Result was not map. Did the request fail? "
      << ( result.type() == QVariant::Bool && !result.toBool() ? "Yes" : "No" ) << std::endl
      << "Typename: " << result.typeName();
  EXPECT_EQ( result.toMap()["sum"].toInt(), 4 )
      << "Contains 'sum'? " << ( result.toMap().contains( "sum" ) ? "Yes" : "No" );

  service_called = false;
  returned = false;
  ros::ServiceServer server_false =
      nh.advertiseService<roscpp_tutorials::TwoIntsRequest, roscpp_tutorials::TwoIntsResponse>(
          "/service_false",
          boost::function<bool( roscpp_tutorials::TwoIntsRequest &, roscpp_tutorials::TwoIntsResponse & )>(
              [&]( roscpp_tutorials::TwoIntsRequest &, roscpp_tutorials::TwoIntsResponse & ) {
                service_called = true;
                returned = true;
                return false;
              } ) );
  obj = engine.newObject();
  callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  service->callAsync( "/service_false", "roscpp_tutorials/TwoInts", { { "a", 1 }, { "b", 3 } },
                      callback );
  ASSERT_TRUE( !returned );
  waitFor( [&returned]() { return returned; } );
  ASSERT_TRUE( returned );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  result = obj.property( "result" ).toVariant();
  ASSERT_EQ( result.type(), QVariant::Bool )
      << "Result was not bool. Typename: " << result.typeName();
  EXPECT_FALSE( result.toBool() );

  service_called = false;
  returned = false;
  ros::ServiceServer server_empty =
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
          "/service_empty",
          boost::function<bool( std_srvs::EmptyRequest &, std_srvs::EmptyResponse & )>(
              [&]( std_srvs::EmptyRequest &, std_srvs::EmptyResponse & ) {
                service_called = true;
                returned = true;
                return true;
              } ) );
  obj = engine.newObject();
  callback =
      engine
          .evaluate( "(function (watcher) { return function (resp) { watcher.result = resp; }; })" )
          .call( { obj } );

  service->callAsync( "/service_empty", "std_srvs/Empty", {}, callback );
  ASSERT_TRUE( !returned );
  waitFor( [&returned]() { return returned; } );
  ASSERT_TRUE( returned );
  processEvents();
  ASSERT_TRUE( obj.hasProperty( "result" ) );
  result = obj.property( "result" ).toVariant();
  ASSERT_EQ( result.type(), QVariant::Bool )
      << "Result was not bool. Typename: " << result.typeName();
  EXPECT_TRUE( result.toBool() );
}

class ActionClientCallback : public QObject
{
  Q_OBJECT
public:
  Q_INVOKABLE void feedbackCalled( int fb ) { this->feedback = fb; }

  Q_INVOKABLE void transitionCalled( int s ) { this->state = s; }

  int feedback = -1;
  int state = -1;
};

TEST( Communication, actionClient )
{
  RosQml::getInstance().setThreads( 8 ); // Enable spinner for this test
  NodeHandle::Ptr node_handle = std::make_shared<NodeHandle>();
  QJSEngine engine;
  auto *client_ptr =
      new ActionClient( node_handle, "ros_babel_fish_test_msgs/SimpleTestAction", "action" );
  engine.newQObject( client_ptr );
  ActionClient &client = *client_ptr;
  EXPECT_FALSE( client.isServerConnected() );
  EXPECT_EQ( client.sendGoal( { { "goal", 8 } } ), nullptr );

  auto *callback_watcher = new ActionClientCallback;
  QJSValue callback_watcher_js = engine.newQObject( callback_watcher );
  QJSValue transition_callback =
      engine
          .evaluate( "(function (watcher) { return function (handle) { "
                     "watcher.transitionCalled(handle.commState); }; })" )
          .call( { callback_watcher_js } );
  QJSValue feedback_callback =
      engine
          .evaluate( "(function (watcher) { return function (handle, feedback) { "
                     "watcher.feedbackCalled(feedback.feedback); }; })" )
          .call( { callback_watcher_js } );
  ASSERT_TRUE(
      waitFor( [&client]() { return client.isServerConnected(); }, 150 ) ); // Wait max 5 seconds
  GoalHandle *handle = dynamic_cast<GoalHandle *>(
      client.sendGoal( { { "goal", 400 } }, transition_callback, feedback_callback ) );
  ASSERT_NE( handle, nullptr );
  ASSERT_FALSE( handle->expired() );
  EXPECT_NE( handle->commState(), action_comm_states::DONE );
  ASSERT_TRUE(
      waitFor( [&handle]() { return handle->commState() == action_comm_states::DONE; }, 90 ) );
  EXPECT_TRUE( waitFor(
      [&callback_watcher]() { return callback_watcher->state == action_comm_states::DONE; } ) );
  EXPECT_EQ( callback_watcher->feedback, 401 );
  TerminalState terminal_state = handle->terminalState();
  EXPECT_EQ( terminal_state.state(), action_terminal_states::SUCCEEDED );
  EXPECT_EQ( terminal_state.text().toStdString(), "test result text" );
  QVariant result = handle->getResult();
  ASSERT_EQ( result.type(), QVariant::Map );
  QVariantMap result_map = result.toMap();
  ASSERT_TRUE( result_map.contains( "result" ) );
  EXPECT_EQ( result_map["result"].type(), QVariant::Int );
  EXPECT_EQ( result_map["result"].toInt(), 800 );
  delete handle;

  // Cancel
  handle = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 300 } } ) );
  ASSERT_NE( handle, nullptr );
  EXPECT_NE( handle->commState(), action_comm_states::DONE );
  EXPECT_TRUE( waitFor( [&handle]() { return handle->commState() == action_comm_states::ACTIVE; } ) );
  RosQml::getInstance().setThreads( 0 ); // Temporarily disable spinner to avoid race condition
  handle->cancel();
  EXPECT_EQ( handle->commState(), action_comm_states::WAITING_FOR_CANCEL_ACK );
  RosQml::getInstance().setThreads( 8 );
  EXPECT_TRUE( waitFor( [&handle]() { return handle->commState() == action_comm_states::DONE; } ) );
  terminal_state = handle->terminalState();
  EXPECT_EQ( handle->terminalState().state(), action_terminal_states::PREEMPTED );
  delete handle;

  // Cancel all goals
  GoalHandle *handle1 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 7000 } } ) );
  ASSERT_NE( handle1, nullptr );
  GoalHandle *handle2 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 8000 } } ) );
  ASSERT_NE( handle2, nullptr );
  GoalHandle *handle3 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 9000 } } ) );
  ASSERT_NE( handle3, nullptr );
  EXPECT_TRUE( waitFor(
      [&handle1]() { return handle1->commState() != action_comm_states::WAITING_FOR_GOAL_ACK; } ) );
  EXPECT_TRUE( waitFor(
      [&handle2]() { return handle2->commState() != action_comm_states::WAITING_FOR_GOAL_ACK; } ) );
  EXPECT_TRUE( waitFor(
      [&handle3]() { return handle3->commState() != action_comm_states::WAITING_FOR_GOAL_ACK; } ) );
  client.cancelAllGoals();
  EXPECT_TRUE( waitFor( [&handle1]() { return handle1->commState() == action_comm_states::DONE; }, 150 ) )
      << handle1->commState();
  EXPECT_TRUE( waitFor( [&handle2]() { return handle2->commState() == action_comm_states::DONE; }, 150 ) )
      << handle2->commState();
  EXPECT_TRUE( waitFor( [&handle3]() { return handle3->commState() == action_comm_states::DONE; }, 150 ) )
      << handle3->commState();
  EXPECT_EQ( handle1->terminalState().state(), action_terminal_states::PREEMPTED );
  EXPECT_EQ( handle2->terminalState().state(), action_terminal_states::PREEMPTED );
  EXPECT_EQ( handle3->terminalState().state(), action_terminal_states::PREEMPTED );
  delete handle1;
  delete handle2;
  delete handle3;

  // Cancel all goals before and at time
  handle1 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 700 } } ) );
  ASSERT_NE( handle1, nullptr );
  usleep( 5000 );
  handle2 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 800 } } ) );
  ASSERT_NE( handle2, nullptr );
  QDateTime now = rosToQmlTime( ros::Time::now() );
  usleep( 5000 );
  handle3 = dynamic_cast<GoalHandle *>( client.sendGoal( { { "goal", 190 } } ) );
  ASSERT_NE( handle3, nullptr );
  ros::spinOnce();
  QCoreApplication::processEvents();
  EXPECT_NE( handle1->commState(), action_comm_states::DONE );
  EXPECT_NE( handle2->commState(), action_comm_states::DONE );
  EXPECT_NE( handle3->commState(), action_comm_states::DONE );
  client.cancelGoalsAtAndBeforeTime( now );
  EXPECT_TRUE( waitFor( [&handle1]() { return handle1->commState() == action_comm_states::DONE; } ) );
  EXPECT_TRUE( waitFor( [&handle2]() { return handle2->commState() == action_comm_states::DONE; } ) );
  EXPECT_EQ( handle1->terminalState().state(), action_terminal_states::PREEMPTED );
  EXPECT_EQ( handle2->terminalState().state(), action_terminal_states::PREEMPTED );
  EXPECT_TRUE( waitFor( [&handle3]() { return handle3->commState() == action_comm_states::DONE; } ) );
  terminal_state = handle3->terminalState();
  EXPECT_EQ( terminal_state.state(), action_terminal_states::SUCCEEDED );
  EXPECT_EQ( terminal_state.text().toStdString(), "test result text" );
  result = handle3->getResult();
  ASSERT_EQ( result.type(), QVariant::Map );
  result_map = result.toMap();
  ASSERT_TRUE( result_map.contains( "result" ) );
  EXPECT_EQ( result_map["result"].type(), QVariant::Int );
  EXPECT_EQ( result_map["result"].toInt(), 380 );

  delete handle1;
  delete handle2;
  delete handle3;

  RosQml::getInstance().setThreads( 0 ); // Disable spinner
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
  EXPECT_TRUE( transform.enabled() );
  ASSERT_TRUE( transform.message().contains( "valid" ) );
  EXPECT_FALSE( transform.message()["valid"].toBool() );
  EXPECT_FALSE( transform.valid() );
  broadcaster.sendTransform( transform_stamped );
  ros::spinOnce();
  transform.setSourceFrame( "base" );
  EXPECT_EQ( transform.sourceFrame(), QString( "base" ) ) << transform.sourceFrame().toStdString();
  transform.setTargetFrame( "world" );
  EXPECT_EQ( transform.targetFrame(), QString( "world" ) ) << transform.targetFrame().toStdString();
  if ( !waitFor( [&]() {
         transform_stamped.header.stamp = ros::Time::now();
         broadcaster.sendTransform( transform_stamped );
         return transform.valid();
       } ) )
    FAIL() << "Failed to get transform in time!";
  ASSERT_TRUE( transform.valid() );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["x"].toDouble(), 1 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["y"].toDouble(), 2.4 );
  EXPECT_DOUBLE_EQ( transform.translation().toMap()["z"].toDouble(), 0.3 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["w"].toDouble(), 0.8432 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["x"].toDouble(), 0.068 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["y"].toDouble(), 0.4821 );
  EXPECT_DOUBLE_EQ( transform.rotation().toMap()["z"].toDouble(), -0.2281 );

  transform.setEnabled( false );
  EXPECT_FALSE( transform.enabled() );
  QCoreApplication::processEvents();
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.transform.translation.x = 3.14;
  broadcaster.sendTransform( transform_stamped );
  waitFor( []() { return false; } );
  EXPECT_DOUBLE_EQ( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(), 1 )
      << "Shouldn't have received the \"x: 3.14\" transform!";
  QDateTime last_transform_datetime = QDateTime::currentDateTime();
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.transform.translation.x = 0.577;
  broadcaster.sendTransform( transform_stamped );
  transform.setEnabled( true );
  EXPECT_TRUE( transform.enabled() );
  if ( !waitFor( [&]() {
         transform_stamped.header.stamp = ros::Time::now();
         broadcaster.sendTransform( transform_stamped );
         return std::abs( transform.translation().toMap()["x"].toDouble() - 0.577 ) < 1E-4;
       } ) )
    FAIL() << "Did not receive transform in time: "
           << transform.translation().toMap()["x"].toDouble();
  EXPECT_DOUBLE_EQ( transform.message()["transform"].toMap()["translation"].toMap()["x"].toDouble(),
                    0.577 );

  QVariant can_transform = wrapper.canTransform( "base", "world" ).toBool();
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool() );
  can_transform = wrapper.canTransform( "millionaire", "inheritance", QDateTime(), 500 );
  EXPECT_TRUE( can_transform.type() != QVariant::Bool || !can_transform.toBool() )
      << "Inheritance shouldn't be able to transform to millionaire!";
  can_transform = wrapper.canTransform( "base", last_transform_datetime, "world",
                                        last_transform_datetime, "world" );
  ASSERT_EQ( can_transform.type(), QVariant::Bool ) << can_transform.toString().toStdString();
  EXPECT_TRUE( can_transform.toBool() );

  EXPECT_TRUE( mapAndMessageEqual( wrapper.lookUpTransform( "world", "base", QDateTime(), 500 ),
                                   transform_stamped ) );
  QVariantMap lookup_result =
      wrapper.lookUpTransform( "world", Time( transform_stamped.header.stamp ), "base",
                               Time( transform_stamped.header.stamp ), "world" );
  ASSERT_TRUE( lookup_result["valid"].toBool() )
      << lookup_result["exception"].toString().toStdString() << std::endl
      << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ) );
  lookup_result =
      wrapper.lookUpTransform( "world", rosToQmlTime( transform_stamped.header.stamp ), "base",
                               rosToQmlTime( transform_stamped.header.stamp ), "world", 500 );
  ASSERT_TRUE( lookup_result["valid"].toBool() )
      << lookup_result["exception"].toString().toStdString() << std::endl
      << lookup_result["message"].toString().toStdString();
  EXPECT_TRUE( mapAndMessageEqual( lookup_result, transform_stamped, "msg", 1E-2 ) );

  EXPECT_EQ( wrapper
                 .canTransform( "world", QDateTime::currentDateTime(), "base",
                                QDateTime::currentDateTime(), "world" )
                 .type(),
             QVariant::String );
  EXPECT_EQ( wrapper
                 .canTransform( "world", QDateTime::currentDateTime(), "base",
                                QDateTime::currentDateTime(), "world", 500 )
                 .type(),
             QVariant::String );
  EXPECT_EQ( wrapper
                 .lookUpTransform( "world", QDateTime::currentDateTime(), "base",
                                   QDateTime::currentDateTime(), "world" )["exception"]
                 .toString(),
             QString( "ExtrapolationException" ) );
  EXPECT_EQ(
      wrapper.lookUpTransform( "world", "base", QDateTime::currentDateTime() )["exception"].toString(),
      QString( "ExtrapolationException" ) );
  EXPECT_EQ(
      wrapper.lookUpTransform( "world", QDateTime(), "equality", QDateTime(), "world" )["exception"]
          .toString(),
      QString( "LookupException" ) );
  EXPECT_EQ( wrapper.lookUpTransform( "world", "equality" )["exception"].toString(),
             QString( "LookupException" ) );
  EXPECT_EQ( wrapper
                 .lookUpTransform( "world", QDateTime(), "billionaires", QDateTime(),
                                   "world" )["exception"]
                 .toString(),
             QString( "ConnectivityException" ) );
  EXPECT_EQ( wrapper.lookUpTransform( "world", "politics" )["exception"].toString(),
             QString( "ConnectivityException" ) );
  // Currently no idea how to test InvalidArgumentException since invalid quaternions get rejected when publishing already

  /// ==============================================================================================
  /// Test rate limiting
  /// ==============================================================================================
  transform.setRate( 30 );
  EXPECT_NEAR( transform.rate(), 30, 3 );
  QSignalSpy transform_changed_spy( &transform, SIGNAL( messageChanged() ) );
  std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
  for ( int i = 0; i < 4000; ++i ) {
    transform_stamped.header.stamp = ros::Time::now();
    broadcaster.sendTransform( transform_stamped );
    processEvents();
    usleep( 100 );
  }
  auto length = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start );
  double sent_hz = 10000 * 1000.0 / length.count();
  ASSERT_GE( sent_hz, 60 ) << "Your computer seems to be too slow to run this test.";
  EXPECT_NEAR( transform_changed_spy.count() * 1000.0 / length.count(), 30, 3 )
      << "Data was sent with " << std::fixed << std::setprecision( 2 ) << sent_hz << "Hz. "
      << "Received " << transform_changed_spy.count() << " messages in " << length.count() << "ms.";

  transform.setRate( 120 );
  transform_changed_spy.clear();
  start = std::chrono::system_clock::now();
  for ( int i = 0; i < 4000; ++i ) {
    transform_stamped.header.stamp = ros::Time::now();
    broadcaster.sendTransform( transform_stamped );
    processEvents();
    usleep( 100 );
  }
  length = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now() -
                                                                  start );
  sent_hz = 10000 * 1000.0 / length.count();
  ASSERT_GE( sent_hz, 200 ) << "Your computer seems to be too slow to run this test.";
  EXPECT_NEAR( transform.rate(), 120, 12 );
  EXPECT_NEAR( transform_changed_spy.count() * 1000.0 / length.count(), transform.rate(), 12 )
      << "Data was sent with " << std::fixed << std::setprecision( 2 ) << sent_hz << "Hz. "
      << "Received " << transform_changed_spy.count() << " messages in " << length.count() << "ms.";
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  ros::init( argc, argv, "communication" );
  RosQml::getInstance().setThreads( 0 ); // Disable spinner
  return RUN_ALL_TESTS();
}

#include "communication.moc"
