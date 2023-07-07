//
// Created by stefan on 28.06.23.
//

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_CAMERA_SUBSCRIPTION_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_CAMERA_SUBSCRIPTION_H

#include "camera_subscription.h"
#include "include/qml_ros_plugin/image_buffer.h"
#include "qml_ros_plugin/helpers/rolling_average.h"

#include <image_transport/image_transport.h>

namespace qml_ros_plugin
{

class ImageTransportCameraSubscription : public CameraSubscription
{
  Q_OBJECT
public:
  ImageTransportCameraSubscription( image_transport::ImageTransport *transport, std::string topic,
                                    int queue_size, image_transport::TransportHints hints )
  {
    subscriber_ = transport->subscribe(
        topic, queue_size, &ImageTransportCameraSubscription::imageCallback, this, hints );
  }

  std::string topic() const override { return subscriber_.getTopic(); }

  std::string transport() const override { return subscriber_.getTransport(); }

  double framerate() const override { return std::round( 1000.0 / camera_base_interval_ * 10 ) / 10; }

  void setSupportedFormats( QList<QVideoFrame::PixelFormat> formats ) override{
    formats_ = formats;
  }

  RosImageBuffer *takeBuffer() override
  {
    std::lock_guard<std::mutex> image_lock( image_mutex_ );
    RosImageBuffer *tmp = last_buffer_;
    last_buffer_ = nullptr;
    return tmp;
  }

private:
  void imageCallback( const sensor_msgs::ImageConstPtr &image )
  {
    ros::Time received_stamp = ros::Time::now();
    auto buffer = new RosImageBuffer( image, formats_ );
    {
      std::lock_guard<std::mutex> image_lock( image_mutex_ );
      if ( !last_image_stamp_.isZero() )
        camera_base_interval_.add( static_cast<int>(
            ( image->header.stamp - last_image_stamp_ ).toNSec() / ( 1000 * 1000 ) ) );
      else if (!last_received_stamp_.isZero())
        camera_base_interval_.add( static_cast<int>(
            ( received_stamp - last_received_stamp_ ).toNSec() / ( 1000 * 1000 ) ) );
      last_received_stamp_ = received_stamp;
      last_image_stamp_ = image->header.stamp;
      delete last_buffer_;
      last_buffer_ = buffer;
    }

    emit newImage();
    //    // Deliver frames on UI thread
    //    QMetaObject::invokeMethod( this, "imageDelivery", Qt::AutoConnection );
  }

  QList<QVideoFrame::PixelFormat> formats_;
  image_transport::Subscriber subscriber_;
  RollingAverage<int, 10> network_latency_average_;
  RollingAverage<int, 10> processing_latency_average_;
  RollingAverage<int, 10> camera_base_interval_;
  ros::Time last_image_stamp_;
  ros::Time last_received_stamp_;
  std::mutex image_mutex_;
  RosImageBuffer *last_buffer_ = nullptr;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_IMAGE_TRANSPORT_CAMERA_SUBSCRIPTION_H
