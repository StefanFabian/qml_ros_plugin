//
// Created by stefan on 28.06.23.
//

#include "rtsp_camera_subscription.h"

#include "qml_ros_plugin/image_buffer.h"
#include <utility>

namespace qml_ros_plugin
{
namespace
{
// Not tested
// 18 YUV420P
// 19 YV12
// 20 UYVY
// 21 YUYV
//
// Not working would have to inspect memory layout
// 22 NV12
// 23 NV21
std::string pixelFormatToGstreamer( QVideoFrame::PixelFormat format );

QVideoFrame::PixelFormat gstreamerToPixelFormat( const std::string &format );

} // namespace

RtspCameraSubscription::RtspCameraSubscription( GMainContext *context, std::string topic,
                                                std::vector<RtspServerInformation::SharedPtr> servers,
                                                std::vector<QSize> sizes,
                                                std::vector<double> framerates )
    : topic_( std::move( topic ) ), servers_( std::move( servers ) ), sizes_( std::move( sizes ) ),
      profile_( 0 ), min_framerate_( 0 ), g_main_context_( context )
{
  if ( !framerates.empty() )
    min_framerate_ = *std::min_element( framerates.begin(), framerates.end() );
  chooseProfile();
}

void RtspCameraSubscription::setSupportedFormats( QList<QVideoFrame::PixelFormat> formats )
{
  //  gst_element_set_state( pipeline_, GST_STATE_PAUSED );
  //  formats_ = formats;
  //  std::set<std::string> gst_formats;
  //  for ( const auto &format : formats ) {
  //    std::string gst_format = pixelFormatToGstreamer( format );
  //    if ( gst_format.empty() )
  //      continue;
  //    gst_formats.insert( gst_format );
  //  }
  //  std::stringstream format_string;
  //  format_string << "{ ";
  //  for ( const auto &format : gst_formats ) { format_string << format << ", "; }
  //  format_string.seekp( -2, std::ios_base::end );
  //  format_string << " }";
  //
  //  //  GstPad *sink_pad = gst_element_get_static_pad(sink_, "sink");
  //  std::string gst_format_string = format_string.str();
  //  GstCaps *caps = gst_caps_new_simple( "video/x-raw", "format", G_TYPE_STRING,
  //                                       gst_format_string.c_str(), nullptr );
  //  //  gst_pad_set_caps( sink_pad, caps );
  //  gst_app_sink_set_caps( GST_APP_SINK( sink_ ), caps );
  //  //  GstCaps *caps = nullptr;
  //  //  g_object_get(G_OBJECT(caps_filter_), "caps", &caps, nullptr);
  //  //  GstCaps *new_caps = gst_caps_copy(caps);
  //  //  gst_caps_set_simple( new_caps, "format", G_TYPE_STRING, gst_format_string.c_str(), nullptr);
  //  //  g_object_set( G_OBJECT(caps_filter_), "caps", new_caps, NULL );
  //  ROS_ERROR_STREAM( "Formats: " << gst_format_string );
  //  gst_element_set_state( pipeline_, GST_STATE_PLAYING );
}

class GstreamerImageBuffer final : public ImageBuffer
{
public:
  GstreamerImageBuffer( GstBuffer *buffer, int width, int height, QVideoFrame::PixelFormat format )
      : ImageBuffer( width, height, format ), buffer_( buffer ), info_()
  {
    gst_buffer_ref( buffer_ );
  }

  ~GstreamerImageBuffer() final { gst_buffer_unref( buffer_ ); }

  uchar *map( MapMode, int *num_bytes, int *bytes_per_line ) override
  {
    if ( !gst_buffer_map( buffer_, &info_, GstMapFlags::GST_MAP_READ ) )
      return nullptr;
    *num_bytes = int( info_.size );
    *bytes_per_line = int( info_.size / height_ );
    return info_.data;
  }

  void unmap() override { gst_buffer_unmap( buffer_, &info_ ); }

private:
  GstBuffer *buffer_;
  GstMapInfo info_;
};

void RtspCameraSubscription::pullSample()
{
  ros::Time received_stamp = ros::Time::now();
  GstSample *sample = pipeline_->pullSample();
  GstCaps *caps = gst_sample_get_caps( sample );
  GstBuffer *sample_buffer = gst_sample_get_buffer( sample );
  const GstStructure *capsformat = gst_caps_get_structure( caps, 0 );
  const char *format;
  int width, height;
  gst_structure_get( capsformat, "width", G_TYPE_INT, &width, "height", G_TYPE_INT, &height,
                     "format", G_TYPE_STRING, &format, nullptr );
  auto buffer =
      new GstreamerImageBuffer( sample_buffer, width, height, gstreamerToPixelFormat( format ) );
  {
    std::lock_guard<std::mutex> image_lock( image_mutex_ );
    if ( !last_received_stamp_.isZero() )
      camera_base_interval_.add(
          static_cast<int>( ( received_stamp - last_received_stamp_ ).toNSec() / ( 1000 * 1000 ) ) );
    last_received_stamp_ = received_stamp;
    delete last_buffer_;
    last_buffer_ = buffer;
  }
  gst_sample_unref( sample );
  emit newImage();
}

void RtspCameraSubscription::setSizes( std::vector<QSize> sizes )
{
  sizes_ = std::move( sizes );
  chooseProfile();
}

void RtspCameraSubscription::setFramerates( std::vector<double> framerates )
{
  min_framerate_ = *std::min_element( framerates.begin(), framerates.end() );
  chooseProfile();
}

void RtspCameraSubscription::chooseProfile()
{
  struct {
    RtspServerInformation::SharedPtr server = nullptr;
    QSize size;
    size_t profile_index = 0;
    double zoom = 0;
  } best_server;

  bool all_valid =
      std::all_of( sizes_.begin(), sizes_.end(), []( const auto &size ) { return size.isValid(); } );
  for ( const auto &server : servers_ ) {
    for ( size_t i = 0; i < server->profiles.size(); ++i ) {
      const auto &profile = server->profiles[i];
      if ( profile.framerate != 0 && profile.framerate < min_framerate_ )
        continue;
      // If profile doesn't specify size, assume unrealistically large
      int profile_width = profile.width == 0 ? std::numeric_limits<int>::max() : profile.width;
      int profile_height = profile.height == 0 ? std::numeric_limits<int>::max() : profile.height;
      if ( !all_valid ) {
        // If one of the sizes is invalid, we take the largest stream
        if ( long(profile_width) * profile_height < long(best_server.size.width()) * best_server.size.height() )
          continue;
        best_server.server = server;
        best_server.profile_index = i;
        best_server.size = QSize( profile_width, profile_height );
        continue;
      }
      // Find the smallest stream that is slightly larger than the largest surface
      auto min_zoom_factor = std::numeric_limits<double>::infinity();
      for ( const auto &size : sizes_ ) {
        // Compute score as zoom factor in size
        const double zoom = std::min( profile_width / size.width(), profile_height / size.height() );
        if ( zoom < min_zoom_factor )
          min_zoom_factor = zoom;
        if ( size.isValid() )
          continue;
        all_valid = false;
        break;
      }
      if ( ( best_server.zoom < 1 && min_zoom_factor > best_server.zoom ) ||
           ( min_zoom_factor >= 1 && min_zoom_factor < best_server.zoom ) ) {
        best_server.zoom = min_zoom_factor;
        best_server.profile_index = i;
        best_server.server = server;
      }
    }
  }
  if ( server_ == best_server.server && profile_ == best_server.profile_index )
    return; // No change necessary

  // Switch server in background and replace pipeline once ready.
  server_ = best_server.server;
  profile_ = best_server.profile_index;
  if ( server_ == nullptr ) {
    pipeline_ = nullptr;
    new_pipeline_ = nullptr;
    return;
  }

  const auto &profile = server_->profiles[profile_];
  new_pipeline_ = std::make_unique<RtspPipeline>(
      g_main_context_, server_->toUrl() + server_->profiles[profile_].name + topic_, profile.codec );
  connect( new_pipeline_.get(), &RtspPipeline::newSample, this,
           &RtspCameraSubscription::switchPipeline, Qt::DirectConnection );
}
void RtspCameraSubscription::switchPipeline()
{
  if ( new_pipeline_ == nullptr )
    return;
  if ( pipeline_ != nullptr )
    disconnect( pipeline_.get() );
  pipeline_ = std::move( new_pipeline_ );
  disconnect( pipeline_.get(), &RtspPipeline::newSample, this,
              &RtspCameraSubscription::switchPipeline );
  pullSample();
  connect( pipeline_.get(), &RtspPipeline::newSample, this, &RtspCameraSubscription::pullSample,
           Qt::DirectConnection );
}

namespace
{

std::string pixelFormatToGstreamer( QVideoFrame::PixelFormat format )
{
  switch ( format ) {
  case QVideoFrame::Format_ARGB32:
  case QVideoFrame::Format_RGB32:
    return "BGRx";
  case QVideoFrame::Format_RGB24:
    return "RGB";
  case QVideoFrame::Format_RGB565:
    return "RGB16";
  case QVideoFrame::Format_ARGB8565_Premultiplied:
    break;
  case QVideoFrame::Format_BGRA32_Premultiplied:
    break;
  case QVideoFrame::Format_BGRA32:
  case QVideoFrame::Format_BGR32:
    return "RGBx";
  case QVideoFrame::Format_BGR24:
    return "BGR";
  case QVideoFrame::Format_BGR565:
    break;
  case QVideoFrame::Format_BGR555:
    break;
  case QVideoFrame::Format_BGRA5658_Premultiplied:
    break;
  case QVideoFrame::Format_AYUV444:
    break;
  case QVideoFrame::Format_AYUV444_Premultiplied:
    break;
  case QVideoFrame::Format_YUV444:
    break;
  case QVideoFrame::Format_YUV420P:
    break;
  case QVideoFrame::Format_YV12:
    break;
  case QVideoFrame::Format_UYVY:
    break;
  case QVideoFrame::Format_YUYV:
    break;
  case QVideoFrame::Format_NV12:
    break;
  case QVideoFrame::Format_NV21:
    break;
  case QVideoFrame::Format_Y8:
    return "GRAY8";
  case QVideoFrame::Format_Y16:
    return "Y16";
  case QVideoFrame::Format_RGB555:
  case QVideoFrame::Format_ARGB32_Premultiplied:
  case QVideoFrame::Format_IMC1:
  case QVideoFrame::Format_IMC2:
  case QVideoFrame::Format_IMC3:
  case QVideoFrame::Format_IMC4:
  case QVideoFrame::Format_Jpeg:
  case QVideoFrame::Format_CameraRaw:
  case QVideoFrame::Format_AdobeDng:
  case QVideoFrame::NPixelFormats:
  case QVideoFrame::Format_Invalid:
  case QVideoFrame::Format_User:
  default:
    break;
  }
  return "";
}

QVideoFrame::PixelFormat gstreamerToPixelFormat( const std::string &format )
{
  if ( format == "RGBx" )
    return QVideoFrame::Format_BGR32;
  if ( format == "BGRx" )
    return QVideoFrame::Format_RGB32;
  return QVideoFrame::Format_Invalid;
}
} // namespace
} // namespace qml_ros_plugin
