//
// Created by Stefan Fabian on 04.03.20.
//

#include "common.h"
#include "message_comparison.h"

#include <qml_ros_plugin/ros.h>
#include <qml_ros_plugin/image_transport_subscriber.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <QCoreApplication>
#include <ros/ros.h>

void processSomeEvents( int n = 10, int sleep_duration_us = 5000 )
{
  for ( int i = 0; i < n; ++i )
  {
    usleep( sleep_duration_us );
    QCoreApplication::processEvents();
    ros::spinOnce();
  }
}

::testing::AssertionResult compareImage( const uint8_t *data, const std::vector<uint8_t> &reference )
{
  for ( size_t i = 0; i < reference.size(); ++i )
  {
    if ( data[i] != reference[i] )
      return ::testing::AssertionFailure() << "Image differed at i=" << i << "." << std::endl
                                           << "data[i]: " << static_cast<int>(data[i]) << std::endl
                                           << "reference[i]: " << static_cast<int>(reference[i]);
  }
  return ::testing::AssertionSuccess();
}

struct MockSurface : public QAbstractVideoSurface
{
  QList<QVideoFrame::PixelFormat> supportedPixelFormats( QAbstractVideoBuffer::HandleType ) const override
  {
    return { QVideoFrame::Format_RGB24 };
  }

  bool present( const QVideoFrame &frame ) override
  {
    last_frame = frame;
    return true;
  }

  QVideoFrame last_frame;
};

struct MockNoFormatSurface : public QAbstractVideoSurface
{
  QList<QVideoFrame::PixelFormat> supportedPixelFormats( QAbstractVideoBuffer::HandleType ) const override
  {
    return {};
  }

  bool isFormatSupported( const QVideoSurfaceFormat & ) const override
  {
    return false;
  }

  bool present( const QVideoFrame &frame ) override
  {
    last_frame = frame;
    return true;
  }

  QVideoFrame last_frame;
};


TEST( ImageTransportSubscriber, testCorrectFormat )
{
  ros::NodeHandle nh( "~" );
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>( "image", 10 );
  NodeHandle::Ptr qml_nh = std::make_shared<NodeHandle>( "~" );
  ImageTransportSubscriber subscriber( qml_nh, "test", 10 );
  EXPECT_EQ( subscriber.topic(), "test" );
  subscriber.setTopic( "image" );
  EXPECT_EQ( subscriber.topic(), "image" ); // Before subscribing the topic name is not resolved.
  EXPECT_EQ( subscriber.defaultTransport(), "compressed" ); // compressed is default
  subscriber.setDefaultTransport( "raw" );
  EXPECT_EQ( subscriber.defaultTransport(), "raw" );
  MockSurface mock_surface;
  subscriber.setVideoSurface( &mock_surface );
  processSomeEvents();
  EXPECT_EQ( subscriber.topic().toStdString(), img_pub.getTopic());
  ASSERT_EQ( img_pub.getNumSubscribers(), 1U );

  sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  // @formatter:off
  image->data = { 255,   0,   0,   0, 255,   0,
                  200, 100,   0,   0, 100, 200,
                  50, 100,  20, 150, 150, 200 };
  // @formatter:on
  img_pub.publish( image );
  processSomeEvents();

  ASSERT_EQ( mock_surface.last_frame.pixelFormat(), QVideoFrame::Format_RGB24 );
  EXPECT_TRUE( mock_surface.last_frame.map( QAbstractVideoBuffer::MapMode::ReadOnly ));
  EXPECT_EQ( mock_surface.last_frame.mappedBytes(), 3 * 3 * 2 );
  EXPECT_EQ( mock_surface.last_frame.bytesPerLine(), 3 * 2 );
  EXPECT_TRUE( compareImage( mock_surface.last_frame.bits(), image->data ));
}

TEST( ImageTransportSubscriber, testWrongFormat )
{
  ros::NodeHandle nh( "~" );
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>( "wrong_image", 10 );
  NodeHandle::Ptr qml_nh = std::make_shared<NodeHandle>( "~" );
  ImageTransportSubscriber subscriber( qml_nh, "wrong_image", 10 );
  EXPECT_EQ( subscriber.topic(), "wrong_image" ); // Before subscribing the topic name is not resolved.
  subscriber.setDefaultTransport( "raw" );
  EXPECT_EQ( subscriber.defaultTransport(), "raw" );
  MockNoFormatSurface mock_surface;
  subscriber.setVideoSurface( &mock_surface );
  processSomeEvents();
  EXPECT_EQ( subscriber.topic().toStdString(), img_pub.getTopic());
  ASSERT_EQ( img_pub.getNumSubscribers(), 1U );

  sensor_msgs::ImagePtr image = boost::make_shared<sensor_msgs::Image>();
  image->width = 2;
  image->height = 3;
  image->step = 2 * 3;
  image->encoding = sensor_msgs::image_encodings::RGB8;
  // @formatter:off
  image->data = { 255,   0,   0,   0, 255,   0,
                  200, 100,   0,   0, 100, 200,
                  50, 100,  20, 150, 150, 200 };
  // @formatter:on
  img_pub.publish( image );
  mock_surface.stop();
  processSomeEvents();

  EXPECT_FALSE( subscriber.subscribed());
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  ros::init( argc, argv, "test_image_transport_subscriber" );
  return RUN_ALL_TESTS();
}
