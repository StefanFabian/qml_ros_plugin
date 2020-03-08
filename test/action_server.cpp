//
// Created by Stefan Fabian on 06.03.20.
//

#include <actionlib/server/action_server.h>
#include <ros_babel_fish_test_msgs/SimpleTestAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <thread>

class TestActionServer
{
public:
  TestActionServer()
    : spinner( 8, &queue )
      , server( nh, "action", boost::bind( &TestActionServer::goalCallback, this, _1 ),
                boost::bind( &TestActionServer::cancelCallback, this, _1 ), false )
  {
    nh.setCallbackQueue( &queue );
  }

  void start()
  {
    spinner.start();
    server.start();
  }

  void processGoal( actionlib::ActionServer<ros_babel_fish_test_msgs::SimpleTestAction>::GoalHandle handle )
  {
    int goal = handle.getGoal()->goal;
    for ( int i = 0; i < goal; ++i )
    {
      usleep( 1000 );
      if ( cancel_requested[handle.getGoalID().id] )
      {
        ros_babel_fish_test_msgs::SimpleTestResult result;
        result.result = handle.getGoal()->goal * 2 - 1;
        handle.setCanceled( result );
        return;
      }
      if ( i == goal / 2 )
      {
        ros_babel_fish_test_msgs::SimpleTestFeedback feedback;
        feedback.feedback = goal + 1;
        handle.publishFeedback( feedback );
      }
    }
    ros_babel_fish_test_msgs::SimpleTestResult result;
    result.result = goal * 2;
    handle.setSucceeded( result, "test result text" );
  }

  void goalCallback( actionlib::ActionServer<ros_babel_fish_test_msgs::SimpleTestAction>::GoalHandle handle )
  {
    cancel_requested.insert( { handle.getGoalID().id, false } );
    handle.setAccepted();
    std::thread execute_thread( &TestActionServer::processGoal, this, handle );
    execute_thread.detach();
//    threads.insert( { handle.getGoalID().id, std::move( execute_thread ) } );
  }

  void cancelCallback( actionlib::ActionServer<ros_babel_fish_test_msgs::SimpleTestAction>::GoalHandle handle )
  {
    cancel_requested[handle.getGoalID().id] = true;
  }

private:
  ros::NodeHandle nh;
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner;
  std::map<std::string, bool> cancel_requested;
  actionlib::ActionServer<ros_babel_fish_test_msgs::SimpleTestAction> server;
  std::map<std::string, std::thread> threads;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_action_server");

  TestActionServer server;
  server.start();
  ros::spin();
  return 0;
}
