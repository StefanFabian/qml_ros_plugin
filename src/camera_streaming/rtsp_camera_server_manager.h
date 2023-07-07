//
// Created by stefan on 28.06.23.
//

#ifndef QML_ROS_PLUGIN_RTSP_CAMERA_SERVER_MANAGER_H
#define QML_ROS_PLUGIN_RTSP_CAMERA_SERVER_MANAGER_H

#include "./rtsp/rtsp_server_information.h"
#include "camera_subscription.h"
#include "rtsp_camera_subscription.h"

#include <glib.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <thread>
#include <unordered_map>

namespace qml_ros_plugin
{

class RtspCameraServerManager final : public QObject
{
  Q_OBJECT
public:
  RtspCameraServerManager();

  ~RtspCameraServerManager() final;

  static RtspCameraServerManager &getInstance();

  void initialize( ros::CallbackQueue *queue );

  //! Checks if a stream for the given topic is available and subscribes to it if possible.
  //! @returns A subscription if successful, nullptr otherwise
  std::unique_ptr<RtspCameraSubscription>
  subscribe( const std::string &topic, std::vector<QSize> sizes, const std::vector<double>& framerates );

signals:
  void serversChanged();

private:
  std::vector<RtspServerInformation::SharedPtr> rtsp_servers_;
  ros::NodeHandle nh_;
  ros::Subscriber camera_server_subscriber_;
  GMainContext *g_main_context_ = nullptr;
  GMainLoop *g_main_loop_ = nullptr;
  std::thread gst_main_thread_;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_RTSP_CAMERA_SERVER_MANAGER_H
