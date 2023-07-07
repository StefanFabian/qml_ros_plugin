//
// Created by stefan on 22.02.21.
//

#include "qml_ros_plugin/image_transport_manager.h"

#include "./camera_streaming/image_transport_camera_subscription.h"
#include "./camera_streaming/rtsp_camera_server_manager.h"
#include "./camera_streaming/rtsp_camera_subscription.h"
#include "qml_ros_plugin/helpers/rolling_average.h"
#include "qml_ros_plugin/image_buffer.h"

#include <QTimer>
#include <QVideoSurfaceFormat>
#include <mutex>
#include <thread>

namespace qml_ros_plugin
{

struct ImageTransportManager::SubscriptionManager {
  explicit SubscriptionManager( const ros::NodeHandle &nh )
  {
    transport = std::make_unique<image_transport::ImageTransport>( nh );
  }

  std::vector<std::shared_ptr<Subscription>> subscriptions;
  std::unique_ptr<image_transport::ImageTransport> transport;
};

class ImageTransportManager::Subscription final : public QObject
{
  Q_OBJECT
public:
  ImageTransportManager *manager = nullptr;
  std::shared_ptr<SubscriptionManager> subscription_manager;
  std::string topic;
  quint32 queue_size = 0;
  image_transport::TransportHints hints;

  void subscribe()
  {
    if ( subscriptions_.empty() )
      return;
    // Subscribing on background thread to reduce load on UI thread
    std::thread( [this]() {
      std::lock_guard<std::mutex> lock( camera_mutex_ );
      // Make sure we don't subscribe twice in a row due to a race condition
      if ( activeSubscription() != nullptr )
        return;
      connect( &RtspCameraServerManager::getInstance(), &RtspCameraServerManager::serversChanged,
               this, &Subscription::onRtspServersChanged, Qt::QueuedConnection );
      // Try to use rtsp if possible
      if ( !subscribeRtsp() ) {
        // Fallback to image transport
        ROS_DEBUG_NAMED( "qml_ros_plugin", "Falling back to image transport for: %s", topic.c_str() );
        camera_subscription_ = std::make_unique<ImageTransportCameraSubscription>(
            subscription_manager->transport.get(), topic, 1, hints );
      }
      auto *sub = activeSubscription();
      ROS_ERROR("Created subscription for: %s", topic.c_str());

      sub->setSupportedFormats( supported_formats_ );
      connect( sub, &CameraSubscription::newImage, this, &Subscription::imageDelivery,
               Qt::QueuedConnection );
    } ).detach();
  }

  bool subscribeRtsp()
  {
    if ( rtsp_subscription_ != nullptr )
      return true;
    if ( hints.getTransport() != "compressed" )
      return false;
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    std::vector<QSize> sizes;
    std::vector<double> framerates;
    sizes.reserve( subscription_handles_.size() );
    framerates.reserve( subscription_handles_.size() );
    for ( const auto &handle : subscription_handles_ ) {
      auto sub = handle.lock();
      if ( sub == nullptr )
        continue;
      sizes.push_back( sub->minimum_size );
      framerates.push_back( sub->framerate_ );
    }
    rtsp_subscription_ = RtspCameraServerManager::getInstance().subscribe( topic, sizes, framerates );
    return rtsp_subscription_ != nullptr;
  }

  void onRtspServersChanged()
  {
    std::lock_guard<std::mutex> lock( camera_mutex_ );
    if ( rtsp_subscription_ != nullptr )
      return; // Subscription will update itself
    // Try to subscribe again and replace stream if possible
    if ( !subscribeRtsp() )
      return;
    // If we have a subscriber now, wait for the first image and replace then
    connect( rtsp_subscription_.get(), &CameraSubscription::newImage, this,
             &Subscription::replaceStream, Qt::QueuedConnection );
  }

  void replaceStream()
  {
    std::unique_lock<std::mutex> lock( camera_mutex_ );
    if ( camera_subscription_ == nullptr )
      return;
    disconnect( rtsp_subscription_.get(), &CameraSubscription::newImage, this,
                &Subscription::replaceStream );
    disconnect( camera_subscription_.get(), &CameraSubscription::newImage, this,
                &Subscription::imageDelivery );
    camera_subscription_ = nullptr;
    connect( activeSubscription(), &CameraSubscription::newImage, this,
             &Subscription::imageDelivery, Qt::QueuedConnection );
    lock.unlock();
    imageDelivery();
    ROS_INFO_NAMED( "qml_ros_plugin", "Switched %s to rtsp", topic.c_str() );
  }

  void addSubscription( const std::shared_ptr<ImageTransportSubscriptionHandle> &sub )
  {
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get() );
    subscription_handles_.push_back( sub );
    updateSupportedFormats();
    // If this was the first subscription, subscribe
    if ( subscriptions_.size() == 1 )
      subscribe();
  }

  void removeSubscription( const ImageTransportSubscriptionHandle *sub )
  {
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    auto it = std::find_if(
        subscriptions_.begin(), subscriptions_.end(),
        [sub]( const ImageTransportSubscriptionHandle *handle ) { return handle == sub; } );
    if ( it == subscriptions_.end() ) {
      ROS_ERROR_NAMED(
          "qml_ros_plugin",
          "Tried to remove a subscription that was not found! Please file a bug report!" );
      return;
    }
    size_t index = it - subscriptions_.begin();
    subscriptions_.erase( it );
    subscription_handles_.erase( subscription_handles_.begin() + index );
    updateSupportedFormats();
    if ( subscriptions_.empty() ) {
      // Unsubscribe if no subscriptions left
      std::lock_guard<std::mutex> lock( camera_mutex_ );
      camera_subscription_ = nullptr;
      rtsp_subscription_ = nullptr;
      ROS_ERROR("Shut down subscription for: %s", topic.c_str());
    }
  }

  void updateSize()
  {
    if ( rtsp_subscription_ == nullptr )
      return; // Only relevant for rtsp streams
    std::vector<QSize> sizes;
    {
      std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
      sizes.reserve( subscription_handles_.size() );
      for ( const auto &sub : subscription_handles_ ) {
        sizes.push_back( sub.lock()->minimum_size );
      }
    }
    rtsp_subscription_->setSizes( sizes );
  }

  void updateFramerate()
  {
    if ( rtsp_subscription_ == nullptr )
      return; // Only relevant for rtsp streams
    std::vector<double> framerates;
    {
      std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
      framerates.reserve( subscription_handles_.size() );
      for ( const auto &sub : subscription_handles_ ) {
        framerates.push_back( sub.lock()->minimum_framerate );
      }
    }
    rtsp_subscription_->setFramerates( framerates );
  }

  std::string getTopic() const
  {
    const auto *sub = activeSubscription();
    if ( sub == nullptr )
      return "";
    const std::string &topic = sub->topic();
    const std::string &transport = sub->transport();
    if ( topic.size() < transport.size() + 1 ||
         0 != topic.compare( topic.size() - transport.size() - 1, transport.size() + 1,
                             "/" + transport ) )
      return topic;
    return topic.substr( 0, topic.size() - transport.size() - 1 );
  }

  const CameraSubscription *activeSubscription() const noexcept
  {
    if ( camera_subscription_ != nullptr )
      return camera_subscription_.get();
    return rtsp_subscription_.get();
  }

  CameraSubscription *activeSubscription() noexcept
  {
    if ( camera_subscription_ != nullptr )
      return camera_subscription_.get();
    return rtsp_subscription_.get();
  }

private:
  Q_INVOKABLE void imageDelivery()
  {
    ImageBuffer *buffer = nullptr;
    double framerate;
    {
      std::lock_guard<std::mutex> lock( camera_mutex_ );
      auto *camera_subscription = activeSubscription();
      if ( camera_subscription == nullptr )
        return;
      buffer = camera_subscription->takeBuffer();
      framerate = camera_subscription->framerate();
    }
    if ( buffer == nullptr )
      return;
    QVideoFrame frame( buffer, QSize( buffer->width(), buffer->height() ), buffer->format() );
    std::vector<std::shared_ptr<ImageTransportSubscriptionHandle>> subscribers;
    {
      // This nested lock makes sure that our destruction of the subscription pointer will not lead to a deadlock
      std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
      for ( const auto &sub_weak : subscription_handles_ ) {
        if ( sub_weak.expired() )
          continue;
        subscribers.push_back( sub_weak.lock() );
      }
    }
    //    int network_latency =
    //        last_image_stamp_.isZero() ? 0 : int( ( received - last_image_stamp_ ).toSec() * 1000 );
    //    if ( network_latency >= 0 ) {
    //      network_latency_average_.add( network_latency );
    //      int processing_latency = int( ( ros::Time::now() - received ).toSec() * 1000 );
    //      processing_latency_average_.add( processing_latency );
    //    } else {
    //      ROS_WARN_ONCE_NAMED( "qml_ros_plugin",
    //                           "The estimated camera network latency was less than 0, make sure your "
    //                           "system's clocks are synced. This warning is only printed once." );
    //    }
    for ( const auto &sub : subscribers ) {
      if ( sub == nullptr || !sub->callback )
        continue;
      //      sub->network_latency = network_latency_average_;
      sub->network_latency = 0;
      sub->processing_latency = processing_latency_average_;
      sub->framerate_ = framerate;
      sub->callback( frame );
    }
  }

  // It's expected that a lock is held for the subscriptions when calling this method
  void updateSupportedFormats()
  {
    bool first = true;
    for ( const auto &sub_weak : subscription_handles_ ) {
      std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
      if ( sub == nullptr || sub->surface == nullptr )
        continue;
      const QList<QVideoFrame::PixelFormat> surface_formats = sub->surface->supportedPixelFormats();
      if ( first ) {
        supported_formats_ = surface_formats;
        first = false;
        continue;
      }
      for ( int i = supported_formats_.size() - 1; i >= 0; --i ) {
        if ( surface_formats.contains( supported_formats_[i] ) )
          continue;
        supported_formats_.removeAt( i );
      }
    }
    if ( camera_subscription_ )
      camera_subscription_->setSupportedFormats( supported_formats_ );
    if ( rtsp_subscription_ )
      rtsp_subscription_->setSupportedFormats( supported_formats_ );
  }

  std::mutex subscriptions_mutex_;
  std::mutex subscribe_mutex_;
  std::mutex camera_mutex_;
  std::unique_ptr<CameraSubscription> camera_subscription_;
  std::unique_ptr<RtspCameraSubscription> rtsp_subscription_;
  std::vector<ImageTransportSubscriptionHandle *> subscriptions_;
  std::vector<std::weak_ptr<ImageTransportSubscriptionHandle>> subscription_handles_;
  QList<QVideoFrame::PixelFormat> supported_formats_;
  RollingAverage<int, 10> processing_latency_average_;
};

ImageTransportSubscriptionHandle::~ImageTransportSubscriptionHandle()
{
  subscription->removeSubscription( this );
}

std::string ImageTransportSubscriptionHandle::getTopic() const { return subscription->getTopic(); }

int ImageTransportSubscriptionHandle::latency() const
{
  return network_latency + processing_latency;
}

int ImageTransportSubscriptionHandle::networkLatency() const { return network_latency; }

int ImageTransportSubscriptionHandle::processingLatency() const { return processing_latency; }

double ImageTransportSubscriptionHandle::framerate() const { return framerate_; }

void ImageTransportSubscriptionHandle::setMinimumSize( const QSize &value )
{
  minimum_size = value;
  subscription->updateSize();
}
void ImageTransportSubscriptionHandle::setMinimumFramerate( double value )
{
  minimum_framerate = value;
  subscription->updateFramerate();
}

ImageTransportManager::ImageTransportManager() = default;

ImageTransportManager &ImageTransportManager::getInstance()
{
  static ImageTransportManager manager;
  return manager;
}

std::shared_ptr<ImageTransportSubscriptionHandle> ImageTransportManager::subscribe(
    const NodeHandle::Ptr &nh, const QString &qtopic, quint32 queue_size,
    const image_transport::TransportHints &transport_hints,
    const std::function<void( const QVideoFrame & )> &callback, QAbstractVideoSurface *surface,
    QSize minimumSize, double minimumFramerate )
{
  std::string ns = nh->ns().toStdString();
  std::string topic = qtopic.toStdString();
  auto it = subscriptions_.find( ns );
  if ( it == subscriptions_.end() ) {
    it = subscriptions_.insert( { ns, std::make_shared<SubscriptionManager>( nh->nodeHandle() ) } ).first;
  }
  std::shared_ptr<SubscriptionManager> &subscription_manager = it->second;
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager->subscriptions;
  try {
    size_t i = 0;
    for ( ; i < subscriptions.size(); ++i ) {
      if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size )
        break; // We could also compare transport type and hints
    }
    auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
    handle->minimum_size = minimumSize;
    handle->minimum_framerate = minimumFramerate;
    handle->surface = surface;
    handle->callback = callback;
    if ( i == subscriptions.size() ) {
      auto sub = new Subscription;
      sub->manager = this;
      sub->subscription_manager = subscription_manager;
      sub->topic = topic;
      sub->queue_size = queue_size;
      sub->hints = transport_hints;
      subscriptions.emplace_back( sub );
      ROS_DEBUG_NAMED( "qml_ros_plugin", "Subscribed to '%s' with transport '%s'.", topic.c_str(),
                       transport_hints.getTransport().c_str() );
    }
    handle->subscription = subscriptions[i];
    subscriptions[i]->addSubscription( handle );
    return handle;
  } catch ( image_transport::TransportLoadException &ex ) {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Could not subscribe to image topic: %s", ex.what() );
  }
  return nullptr;
}
} // namespace qml_ros_plugin

#include "image_transport_manager.moc"
