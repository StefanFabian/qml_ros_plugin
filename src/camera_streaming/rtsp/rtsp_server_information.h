//
// Created by stefan on 04.07.23.
//

#ifndef QML_ROS_PLUGIN_RTSP_SERVER_INFORMATION_H
#define QML_ROS_PLUGIN_RTSP_SERVER_INFORMATION_H

#include <hector_camera_server_msgs/Profile.h>
#include <regex>
#include <memory>

namespace qml_ros_plugin
{

struct RtspServerInformation
{
  using SharedPtr = std::shared_ptr<RtspServerInformation>;

  std::string url;
  int port;
  std::vector<hector_camera_server_msgs::Profile> profiles;
  std::vector<std::regex> topics;

  std::string toUrl() const
  {
    return "rtsp://" + url + ":" + std::to_string( port ) + "/";
  }
};
}

#endif // QML_ROS_PLUGIN_RTSP_SERVER_INFORMATION_H
