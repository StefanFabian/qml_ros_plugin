//
// Created by stefan on 04.07.23.
//

#include "rtsp_pipeline.h"

#include <gst/app/gstappsink.h>
#include <ros/console.h>

namespace qml_ros_plugin
{

gboolean RtspPipeline::forwardPipelineMessage( GstBus *bus, GstMessage *msg, gpointer udata )
{
  auto instance = static_cast<RtspPipeline *>( udata );
  instance->handleMessage( bus, msg );
  return TRUE;
}

GstFlowReturn RtspPipeline::onNewSample( GstElement *, gpointer udata )
{
  auto instance = static_cast<RtspPipeline *>( udata );
  instance->state_ = Connected;
  emit instance->newSample();
  return GST_FLOW_OK;
}

gboolean RtspPipeline::onRestartTimeout( gpointer udata )
{
  auto instance = static_cast<RtspPipeline *>( udata );
  instance->restartPipeline();
  return FALSE;
}

GstSample *RtspPipeline::pullSample()
{
  GstAppSink *sink = GST_APP_SINK( sink_ );
  return gst_app_sink_pull_sample( sink );
}

RtspPipeline::RtspPipeline( GMainContext *context, std::string url, std::string codec )
    : url_( std::move( url ) ), codec_( std::move( codec ) ), g_main_context_( context )
{
  createPipeline();
}

RtspPipeline::~RtspPipeline()
{
  state_ = Destroying;
  gst_element_set_state( pipeline_, GST_STATE_NULL );
  if ( sink_ )
    gst_object_unref( sink_ );
  if ( rtspsrc_ )
    gst_object_unref( rtspsrc_ );
  if ( pipeline_ )
    gst_object_unref( pipeline_ );
}

void RtspPipeline::handleMessage( GstBus *, GstMessage *msg )
{
  if ( state_ == Destroying )
    return;
  switch ( GST_MESSAGE_TYPE( msg ) ) {
  case GST_MESSAGE_EOS:
    // Restart pipeline
    ROS_INFO_NAMED( "qml_ros_plugin", "Received EOS. Restarting pipeline." );
    schedulePipelineRestart();
    break;
  case GST_MESSAGE_ERROR: {
    GError *error = nullptr;
    gchar *dbg = nullptr;
    gst_message_parse_error( msg, &error, &dbg );
    if ( error != nullptr ) {
      ROS_ERROR_COND( ( error->code != 7 ), "Pipeline error: %d (%s) - %s", error->code,
                      error->message, dbg );
      g_error_free( error );
    }
    //    if ( msg->src != GST_OBJECT( rtspsrc_ ) )
    //      break;
    schedulePipelineRestart();
    break;
  }
  case GST_MESSAGE_STATE_CHANGED: {
    if ( msg->src != GST_OBJECT( pipeline_ ) )
      return;
    GstState old_state, new_state, pending_state;
    gst_message_parse_state_changed( msg, &old_state, &new_state, &pending_state );
    //    if ( new_state == GST_STATE_PAUSED && state_ == Connecting ) {
    //      schedulePipelineRestart( 10000 );
    //    }
    break;
  }
  case GST_MESSAGE_CLOCK_LOST:
  case GST_MESSAGE_UNKNOWN:
  case GST_MESSAGE_WARNING:
  case GST_MESSAGE_INFO:
  case GST_MESSAGE_TAG:
  case GST_MESSAGE_BUFFERING:
  case GST_MESSAGE_STATE_DIRTY:
  case GST_MESSAGE_STEP_DONE:
  case GST_MESSAGE_CLOCK_PROVIDE:
  case GST_MESSAGE_NEW_CLOCK:
  case GST_MESSAGE_STRUCTURE_CHANGE:
  case GST_MESSAGE_STREAM_STATUS:
  case GST_MESSAGE_APPLICATION:
  case GST_MESSAGE_ELEMENT:
  case GST_MESSAGE_SEGMENT_START:
  case GST_MESSAGE_SEGMENT_DONE:
  case GST_MESSAGE_DURATION_CHANGED:
  case GST_MESSAGE_LATENCY:
  case GST_MESSAGE_ASYNC_START:
  case GST_MESSAGE_ASYNC_DONE:
  case GST_MESSAGE_REQUEST_STATE:
  case GST_MESSAGE_STEP_START:
  case GST_MESSAGE_QOS:
  case GST_MESSAGE_PROGRESS:
  case GST_MESSAGE_TOC:
  case GST_MESSAGE_RESET_TIME:
  case GST_MESSAGE_STREAM_START:
  case GST_MESSAGE_NEED_CONTEXT:
  case GST_MESSAGE_HAVE_CONTEXT:
  case GST_MESSAGE_EXTENDED:
  case GST_MESSAGE_DEVICE_ADDED:
  case GST_MESSAGE_DEVICE_REMOVED:
  case GST_MESSAGE_PROPERTY_NOTIFY:
  case GST_MESSAGE_STREAM_COLLECTION:
  case GST_MESSAGE_STREAMS_SELECTED:
  case GST_MESSAGE_REDIRECT:
  case GST_MESSAGE_DEVICE_CHANGED:
  case GST_MESSAGE_ANY:
    break;
  }
}

void RtspPipeline::schedulePipelineRestart( int msec )
{
  if ( state_ == Connected ) {
    state_ = RestartScheduled;
    restartPipeline();
    return;
  }
  if ( state_ == RestartScheduled )
    return;
  state_ = RestartScheduled;
  if ( timeout_source_ )
    g_source_destroy( timeout_source_ );
  timeout_source_ = g_timeout_source_new( msec );
  g_source_set_callback( timeout_source_, (GSourceFunc)onRestartTimeout, this, nullptr );
  g_source_attach( timeout_source_, g_main_context_ );
}

void RtspPipeline::restartPipeline()
{
  if ( state_ != RestartScheduled )
    return;
  state_ = Connecting;
  gst_element_set_state( pipeline_, GST_STATE_NULL );
  createPipeline();
}

void RtspPipeline::createPipeline()
{

  std::string decode_pipeline;
  if ( codec_ == "h264" ) {
    decode_pipeline = "rtph264depay ! h264parse";
  } else {
    decode_pipeline = "rtph265depay ! h265parse";
  }

  GError *error = nullptr;
  std::string full_pipeline_launch =
      "rtspsrc name=src latency=0 do-retransmission=false is-live=true location=" + url_ + " ! " +
      decode_pipeline +
      " ! decodebin ! videoconvert ! video/x-raw,format=BGRx !"
      " appsink sync=false drop=true max-buffers=1 name=sink emit-signals=true";
  ROS_DEBUG_STREAM_NAMED( "qml_ros_plugin", "Starting pipeline: " << full_pipeline_launch );
  state_ = Connecting;
  if ( pipeline_ )
    gst_object_unref( pipeline_ );
  pipeline_ = gst_parse_launch( full_pipeline_launch.c_str(), &error );

  if ( rtspsrc_ )
    gst_object_unref( rtspsrc_ );
  rtspsrc_ = gst_bin_get_by_name( GST_BIN( pipeline_ ), "src" );

  if ( sink_ )
    gst_object_unref( sink_ );
  GstAppSink *sink = GST_APP_SINK( gst_bin_get_by_name( GST_BIN( pipeline_ ), "sink" ) );
  g_signal_connect( sink, "new-sample", G_CALLBACK( onNewSample ), this );
  sink_ = GST_ELEMENT( sink );
  gst_element_set_state( pipeline_, GST_STATE_PLAYING );

  // Error handling
  GstBus *bus = gst_pipeline_get_bus( GST_PIPELINE( pipeline_ ) );
  GSource *source = gst_bus_create_watch( bus );
  g_source_set_callback( source, (GSourceFunc)forwardPipelineMessage, this, nullptr );
  g_source_attach( source, g_main_context_ );
  g_source_unref( source );
  gst_object_unref( bus );
}
} // namespace qml_ros_plugin
