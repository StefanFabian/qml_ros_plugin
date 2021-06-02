//
// Created by stefan on 22.02.21.
//

#ifndef QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H
#define QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H

#include "qml_ros_plugin/node_handle.h"

#include <image_transport/image_transport.h>
#include <QAbstractVideoSurface>
#include <mutex>

namespace qml_ros_plugin
{
class ImageTransportSubscriptionHandle;

class ImageTransportManagerSingletonWrapper : public QObject
{
Q_OBJECT
public:
  //! @copydoc ImageTransportManager::setLoadBalancingEnabled
  Q_INVOKABLE void setLoadBalancingEnabled( bool value );
};

/*!
 * Encapsulates the image transport communication to share the subscription resources, avoiding multiple conversions of
 *  the same image and subscription overhead if multiple cameras are set to throttle.
 */
class ImageTransportManager
{
  ImageTransportManager();

  struct SubscriptionManager;
  class Subscription;
  class LoadBalancer;
public:

  static ImageTransportManager &getInstance();

  //! Sets whether the manager should try to balance throttled subscriptions_ to ensure they don't update at the same
  //!   time which would result in network spikes.
  void setLoadBalancingEnabled( bool value );

  /*!
   * Note: Can only be called with a ready NodeHandle!
   *
   * Subscribes to the given topic with the given settings. Makes sure that multiple subscriptions_ (especially throttled)
   *  only result in a single (throttled) subscription.
   * If multiple subscriptions_ of the same topic and namespace are created, the settings of the first subscription are used.
   * Except for the throttle interval where the minimum value across all active subscriptions_ is used.
   * @param nh The NodeHandle the subscription is made from if it is the first subscribe call
   * @param qtopic
   * @param queue_size
   * @param transport_hints
   * @param callback
   * @param surface
   * @param throttle_interval
   * @return
   */
  std::shared_ptr<ImageTransportSubscriptionHandle>
  subscribe( const NodeHandle::Ptr &nh, const QString &qtopic, quint32 queue_size,
             const image_transport::TransportHints &transport_hints,
             const std::function<void( const QVideoFrame & )> &callback, QAbstractVideoSurface *surface = nullptr,
             int throttle_interval = 0 );

private:
  std::map<std::string, std::shared_ptr<SubscriptionManager>> subscriptions_;
  std::unique_ptr<LoadBalancer> load_balancer_;

  friend class ImageTransportSubscriptionHandle;
};

class ImageTransportSubscriptionHandle
{
public:

  ~ImageTransportSubscriptionHandle();

  //! The interval in ms the subscription waits between receiving images.
  int throttleInterval() const { return throttle_interval; }

  //! Set the interval in ms the subscription may wait between images.
  //! The images may still arrive at a higher rate if other subscriptions request it.
  void updateThrottleInterval( int interval );

  std::string getTopic();

private:
  int throttle_interval = 0;
  std::shared_ptr<ImageTransportManager::Subscription> subscription;
  QAbstractVideoSurface *surface = nullptr;
  std::function<void( const QVideoFrame & )> callback;

  friend class ImageTransportManager;
};
}

#endif //QML_ROS_PLUGIN_IMAGE_TRANSPORT_MANAGER_H
