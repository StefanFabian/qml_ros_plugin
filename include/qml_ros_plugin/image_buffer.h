// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_IMAGE_BUFFER_H
#define QML_ROS_PLUGIN_IMAGE_BUFFER_H

#include <QVideoFrame>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace qml_ros_plugin
{

class ImageBuffer : public QAbstractVideoBuffer
{
public:
  ImageBuffer( sensor_msgs::ImageConstPtr img, const QList<QVideoFrame::PixelFormat> &supported_formats );

  ~ImageBuffer() override;

  MapMode mapMode() const override;

  uchar *map( MapMode, int *num_bytes, int *bytes_per_line ) override;

  void unmap() override;

  QVideoFrame::PixelFormat format() const;

private:
  sensor_msgs::ImageConstPtr image_;
  QVideoFrame::PixelFormat format_;
  int num_bytes_;
  int bytes_per_line_;
  unsigned char *data_;
};
}

#endif //QML_ROS_PLUGIN_IMAGE_BUFFER_H
