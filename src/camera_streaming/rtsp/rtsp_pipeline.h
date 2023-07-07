//
// Created by stefan on 04.07.23.
//

#ifndef QML_ROS_PLUGIN_RTSP_PIPELINE_H
#define QML_ROS_PLUGIN_RTSP_PIPELINE_H

#include <qobject.h>
#include <gst/gstelement.h>

namespace qml_ros_plugin
{

class RtspPipeline : public QObject
{
  Q_OBJECT
public:
  enum State {
    Connecting,
    Connected,
    RestartScheduled,
    Destroying
  };

  RtspPipeline(GMainContext *context, std::string url, std::string codec);

  ~RtspPipeline() override;

  GstSample *pullSample();

signals:
  void playing();

  void newSample();

private:
  static gboolean forwardPipelineMessage( GstBus *bus, GstMessage *msg, gpointer udata );
  static GstFlowReturn onNewSample( GstElement *sink, gpointer udata );
  static gboolean onRestartTimeout( gpointer udata );

  void handleMessage( GstBus *bus, GstMessage *msg );

  void schedulePipelineRestart(int msec = 200);
  void restartPipeline();

  void createPipeline();

  std::string url_;
  std::string codec_;
  State state_;
  GMainContext *g_main_context_ = nullptr;
  GSource *timeout_source_ = nullptr;
  GstElement *pipeline_ = nullptr;
  GstElement *rtspsrc_ = nullptr;
  GstElement *sink_ = nullptr;
};
}

#endif // QML_ROS_PLUGIN_RTSP_PIPELINE_H
