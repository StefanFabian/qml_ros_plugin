//
// Created by stefan on 28.06.23.
//

#ifndef QML_ROS_PLUGIN_RTSP_CAMERA_SUBSCRIPTION_H
#define QML_ROS_PLUGIN_RTSP_CAMERA_SUBSCRIPTION_H

#include "./rtsp/rtsp_pipeline.h"
#include "./rtsp/rtsp_server_information.h"
#include "camera_subscription.h"

#include "qml_ros_plugin/helpers/rolling_average.h"

#include <QTimer>
#include <mutex>

namespace qml_ros_plugin
{

class RtspCameraSubscription : public CameraSubscription
{
  Q_OBJECT
public:
  RtspCameraSubscription( GMainContext *context, std::string topic,
                          std::vector<RtspServerInformation::SharedPtr> servers,
                          std::vector<QSize> sizes, std::vector<double> framerates );

  std::string topic() const override { return topic_; }

  std::string transport() const override { return "compressed"; }

  double framerate() const override
  {
    return std::round( 1000.0 / camera_base_interval_ * 10 ) / 10;
  }

  void setSupportedFormats( QList<QVideoFrame::PixelFormat> formats ) override;

  ImageBuffer *takeBuffer() override
  {
    std::lock_guard<std::mutex> image_lock( image_mutex_ );
    ImageBuffer *tmp = last_buffer_;
    last_buffer_ = nullptr;
    return tmp;
  }

  void setSizes( std::vector<QSize> sizes );

  void setFramerates( std::vector<double> framerates );

private slots:

  void pullSample();

  void switchPipeline();

private:
  void chooseProfile();

  std::string topic_;
  std::vector<RtspServerInformation::SharedPtr> servers_;
  QList<QVideoFrame::PixelFormat> formats_;
  std::vector<QSize> sizes_;
  RollingAverage<int, 10> network_latency_average_;
  RollingAverage<int, 10> processing_latency_average_;
  RollingAverage<int, 10> camera_base_interval_;
  //  ros::Time last_image_stamp_;
  ros::Time last_received_stamp_;
  std::mutex image_mutex_;

  RtspServerInformation::SharedPtr server_;
  size_t profile_ = 0;
  double min_framerate_ = 0;

  std::unique_ptr<RtspPipeline> pipeline_;
  std::unique_ptr<RtspPipeline> new_pipeline_;
  GMainContext *g_main_context_ = nullptr;
  ImageBuffer *last_buffer_ = nullptr;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_RTSP_CAMERA_SUBSCRIPTION_H
