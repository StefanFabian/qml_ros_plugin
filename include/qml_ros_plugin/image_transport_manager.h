//
// Created by stefan on 22.02.21.
//

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H

#include "qml_ros_plugin/node_handle.h"

#include <QAbstractVideoSurface>
#include <image_transport/image_transport.h>
#include <mutex>
#include <unordered_map>

namespace qml_ros_plugin
{
class ImageTransportSubscriptionHandle;

/*!
 * Encapsulates the image transport communication to share the subscription resources, avoiding multiple conversions of
 *  the same image and subscription overhead if multiple cameras are set to throttle.
 */
class ImageTransportManager
{
  ImageTransportManager();

  struct SubscriptionManager;
  class Subscription;

public:
  static ImageTransportManager &getInstance();

  /*!
   * Note: Can only be called with a ready NodeHandle!
   *
   * Subscribes to the given topic with the given settings. Makes sure that multiple subscriptions_
   * only result in a single subscription. If multiple subscriptions_ of the same topic and
   * namespace are created, the settings of the first subscription are used.
   * @param nh The NodeHandle the subscription is made from if it is the first subscribe call
   * @param qtopic
   * @param queue_size
   * @param transport_hints
   * @param callback
   * @param surface
   * @return
   */
  std::shared_ptr<ImageTransportSubscriptionHandle>
  subscribe( const NodeHandle::Ptr &nh, const QString &qtopic, quint32 queue_size,
             const image_transport::TransportHints &transport_hints,
             const std::function<void( const QVideoFrame & )> &callback,
             QAbstractVideoSurface *surface = nullptr, QSize minimumSize = {},
             double minimumFramerate = 0 );

private:
  std::unordered_map<std::string, std::shared_ptr<SubscriptionManager>> subscriptions_;

  friend class ImageTransportSubscriptionHandle;
};

class ImageTransportSubscriptionHandle
{
public:
  ~ImageTransportSubscriptionHandle();

  //! The subscribed topic. Once subscribed this is the full topic name without the transport.
  std::string getTopic() const;

  //! The full latency (in ms) from camera to your display excluding drawing time.
  int latency() const;

  //! The latency (in ms) from the camera to the reception of the image in this node.
  int networkLatency() const;

  //! The latency (in ms) from the reception of the image until it is in a displayable format.
  int processingLatency() const;

  //! The framerate (in frames per second).
  double framerate() const;

  void setMinimumSize( const QSize &value );

  void setMinimumFramerate( double value );

private:
  std::shared_ptr<ImageTransportManager::Subscription> subscription;
  QAbstractVideoSurface *surface = nullptr;
  std::function<void( const QVideoFrame & )> callback;
  double framerate_ = 0;
  int network_latency = -1;
  int processing_latency = -1;
  QSize minimum_size;
  double minimum_framerate;

  friend class ImageTransportManager;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H
