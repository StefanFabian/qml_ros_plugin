//
// Created by stefan on 28.06.23.
//
#include "rtsp_camera_server_manager.h"
#include "rtsp_camera_subscription.h"

#include <gst/gst.h>
#include <hector_camera_server_msgs/Announcement.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/node_handle.h>

namespace qml_ros_plugin
{

RtspCameraServerManager &RtspCameraServerManager::getInstance()
{
  static RtspCameraServerManager instance;
  return instance;
}

RtspCameraServerManager::RtspCameraServerManager()
{
  if ( !gst_is_initialized() )
    gst_init( nullptr, nullptr );

  g_main_context_ = g_main_context_new();
  g_main_loop_ = g_main_loop_new( g_main_context_, false );
  gst_main_thread_ = std::thread( [this]() { g_main_loop_run( g_main_loop_ ); } );
}

void RtspCameraServerManager::initialize( ros::CallbackQueue *queue )
{
  nh_.setCallbackQueue( queue );
  camera_server_subscriber_ = nh_.subscribe<hector_camera_server_msgs::Announcement>(
      "/camera_server_announcement", 10,
      boost::function<void( const ros::MessageEvent<hector_camera_server_msgs::Announcement> & )>(
          [&]( const ros::MessageEvent<hector_camera_server_msgs::Announcement> &event ) {
            const std::string &publisher = event.getPublisherName();
            const auto &msg = event.getConstMessage();
            ROS_DEBUG_NAMED( "qml_ros_plugin", "Got announcement from %s.", publisher.c_str() );
            auto it = std::find_if( rtsp_servers_.begin(), rtsp_servers_.end(),
                                    [&msg]( const RtspServerInformation::SharedPtr &info ) {
                                      return info->url == msg->server_address &&
                                             info->port == msg->server_port;
                                    } );
            if ( it == rtsp_servers_.end() )
              it = rtsp_servers_.insert( it, std::make_shared<RtspServerInformation>() );
            **it = { .url = msg->server_address,
                     .port = msg->server_port,
                     .profiles = msg->profiles,
                     .topics = {} };
            ( *it )->topics.reserve( msg->topics.size() );
            std::transform( msg->topics.begin(), msg->topics.end(),
                            std::back_inserter( ( *it )->topics ),
                            []( auto t ) { return std::regex( t ); } );
            emit serversChanged();
          } ) );
}

RtspCameraServerManager::~RtspCameraServerManager()
{
  g_main_loop_quit( g_main_loop_ );
  gst_main_thread_.join();
  if ( g_main_loop_ )
    g_main_loop_unref( g_main_loop_ );
  if ( g_main_context_ )
    g_main_context_unref( g_main_context_ );
}
std::unique_ptr<RtspCameraSubscription>
RtspCameraServerManager::subscribe( const std::string &topic, std::vector<QSize> sizes,
                                    const std::vector<double> &framerates )
{
  std::vector<RtspServerInformation::SharedPtr> servers;
  for ( auto &s : rtsp_servers_ ) {
    if ( std::none_of( s->topics.begin(), s->topics.end(),
                       [&]( const std::regex &item ) { return std::regex_match( topic, item ); } ) )
      continue;
    servers.push_back( s );
  }
  if ( servers.empty() )
    return nullptr;
  auto subscription = std::make_unique<RtspCameraSubscription>( g_main_context_, topic, servers,
                                                                std::move( sizes ), framerates );
  return subscription;
}
} // namespace qml_ros_plugin
