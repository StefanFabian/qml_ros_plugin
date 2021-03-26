//
// Created by stefan on 22.02.21.
//

#include "qml_ros_plugin/image_transport_manager.h"
#include "qml_ros_plugin/image_buffer.h"

#include <QTimer>
#include <mutex>

namespace qml_ros_plugin
{

struct ImageTransportManager::SubscriptionManager
{
  explicit SubscriptionManager( const ros::NodeHandle &nh )
  {
    transport.reset( new image_transport::ImageTransport( nh ));
  }

  std::vector<std::shared_ptr<Subscription>> subscriptions;
  std::unique_ptr<image_transport::ImageTransport> transport;
};

class ImageTransportManager::Subscription : public QObject
{
Q_OBJECT
public:
  ImageTransportManager *manager = nullptr;
  std::shared_ptr<SubscriptionManager> subscription_manager;
  std::string topic;
  quint32 queue_size = 0;
  image_transport::TransportHints hints;
  QTimer throttle_timer;

  Subscription()
  {
    QObject::connect( &throttle_timer, &QTimer::timeout, this, &ImageTransportManager::Subscription::subscribe );
  }

  int getThrottleInterval()
  {
    return std::accumulate( subscription_handles_.begin(), subscription_handles_.end(), std::numeric_limits<int>::max(),
                            []( int current, const std::weak_ptr<ImageTransportSubscriptionHandle> &sub_weak )
                            {
                              std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
                              if ( sub == nullptr ) return current;
                              return std::min( current, sub->throttle_interval );
                            } );
  }

  void updateTimer()
  {
    int min_interval = getThrottleInterval();
    if ( min_interval == 0 )
    {
      throttle_timer.stop();
      if ( !subscriber_ ) subscribe();
    }
    else if ( throttle_timer.remainingTime() > min_interval )
    {
      throttle_timer.start( min_interval );
    }
  }

  Q_INVOKABLE void restartTimer()
  {
    int interval = getThrottleInterval();
    int timeout = interval == 0 ? 0 : manager->getLoadBalancedTimeout( interval );
    if ( timeout == 0 ) subscribe();
    else throttle_timer.start( timeout );
  }

  void subscribe()
  {
    subscriber_ = subscription_manager->transport->subscribe( topic, queue_size, &Subscription::imageCallback, this,
                                                              hints );
  }

  void addSubscription( const std::shared_ptr<ImageTransportSubscriptionHandle> &sub )
  {
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get());
    subscription_handles_.push_back( sub );
    updateSupportedFormats();
    updateTimer();
  }

  void removeSubscription( const ImageTransportSubscriptionHandle *sub )
  {
    std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
    auto it = std::find_if( subscriptions_.begin(), subscriptions_.end(),
                            [ sub ]( const ImageTransportSubscriptionHandle *handle )
                            {
                              return handle == sub;
                            } );
    if ( it == subscriptions_.end())
    {
      ROS_ERROR_NAMED( "qml_ros_plugin",
                       "Tried to remove a subscription that was not found! Please file a bug report!" );
      return;
    }
    size_t index = it - subscriptions_.begin();
    subscriptions_.erase( it );
    subscription_handles_.erase( subscription_handles_.begin() + index );
    updateSupportedFormats();
  }

  std::string getTopic() const { return subscriber_.getTopic(); }

private:

  void imageCallback( const sensor_msgs::ImageConstPtr &image )
  {
    ros::Time received_stamp = ros::Time::now();
    // Check if we have to throttle
    int interval = getThrottleInterval();
    throttled_ = false;
    if ( interval != 0 && interval > camera_base_interval_ )
    {
      subscriber_.shutdown();
      QMetaObject::invokeMethod( this, "restartTimer", Qt::AutoConnection );
      throttled_ = true;
    }
    QList<QVideoFrame::PixelFormat> formats;
    {
      std::lock_guard<std::mutex> subscription_lock( subscriptions_mutex_ );
      if ( subscription_handles_.empty()) return;
      formats = supported_formats_;
    }
    auto buffer = new ImageBuffer( image, formats );

    {
      std::lock_guard<std::mutex> image_lock( image_mutex_ );
      if ( !throttled_ && last_image_ != nullptr )
      {
        // Update the base interval only if we are not throttled
        if ( last_image_->header.stamp.isZero())
          camera_base_interval_ = static_cast<int>((last_received_stamp_ - received_stamp).toNSec() / (1000 * 1000));
        else
          camera_base_interval_ = static_cast<int>((last_image_->header.stamp - image->header.stamp).toNSec() /
                                                   (1000 * 1000));
      }
      last_received_stamp_ = received_stamp;
      last_image_ = image;
      last_buffer_ = buffer;
    }
    // Deliver frames on UI thread
    QMetaObject::invokeMethod( this, "imageDelivery", Qt::AutoConnection );
  }

  Q_INVOKABLE void imageDelivery()
  {
    ImageBuffer *buffer;
    sensor_msgs::ImageConstPtr image;
    {
      std::lock_guard<std::mutex> image_lock( image_mutex_ );
      if ( last_buffer_ == nullptr || last_image_ == nullptr ) return;
      buffer = last_buffer_;
      image = last_image_;
      last_buffer_ = nullptr;
    }
    QVideoFrame frame( buffer, QSize( image->width, image->height ), buffer->format());
    std::vector<std::shared_ptr<ImageTransportSubscriptionHandle>> subscribers;
    {
      // This nested lock makes sure that our destruction of the subscription pointer will not lead to a deadlock
      std::lock_guard<std::mutex> subscriptions_lock( subscriptions_mutex_ );
      for ( const auto &sub_weak : subscription_handles_ )
      {
        if ( sub_weak.expired()) continue;
        subscribers.push_back( sub_weak.lock());
      }
    }
    for ( const auto &sub : subscribers )
    {
      if ( sub == nullptr || !sub->callback ) continue;
      sub->callback( frame );
    }
  }

  // It's expected that a lock is held for the subscriptions when calling this method
  void updateSupportedFormats()
  {
    bool first = true;
    for ( const auto &sub_weak : subscription_handles_ )
    {
      std::shared_ptr<ImageTransportSubscriptionHandle> sub = sub_weak.lock();
      if ( sub == nullptr || sub->surface == nullptr ) continue;
      const QList<QVideoFrame::PixelFormat> surface_formats = sub->surface->supportedPixelFormats();
      if ( first )
      {
        supported_formats_ = surface_formats;
        first = false;
        continue;
      }
      for ( int i = supported_formats_.size() - 1; i >= 0; --i )
      {
        if ( surface_formats.contains( supported_formats_[i] )) continue;
        supported_formats_.removeAt( i );
      }
    }
  }

  std::mutex subscriptions_mutex_;
  std::mutex image_mutex_;
  image_transport::Subscriber subscriber_;
  std::vector<ImageTransportSubscriptionHandle *> subscriptions_;
  std::vector<std::weak_ptr<ImageTransportSubscriptionHandle>> subscription_handles_;
  QList<QVideoFrame::PixelFormat> supported_formats_;
  ImageBuffer *last_buffer_ = nullptr;
  sensor_msgs::ImageConstPtr last_image_;
  ros::Time last_received_stamp_;
  int camera_base_interval_ = std::numeric_limits<int>::max();
  bool throttled_ = false;
};

ImageTransportSubscriptionHandle::~ImageTransportSubscriptionHandle()
{
  subscription->removeSubscription( this );
}

void ImageTransportSubscriptionHandle::updateThrottleInterval( int interval )
{
  throttle_interval = interval;
  subscription->updateTimer();
}

std::string ImageTransportSubscriptionHandle::getTopic()
{
  return subscription->getTopic();
}

ImageTransportManager::ImageTransportManager() = default;


ImageTransportManager &ImageTransportManager::getInstance()
{
  static ImageTransportManager manager;
  return manager;
}

std::shared_ptr<ImageTransportSubscriptionHandle>
ImageTransportManager::subscribe( const NodeHandle::Ptr &nh, const QString &qtopic, quint32 queue_size,
                                  const image_transport::TransportHints &transport_hints,
                                  const std::function<void( const QVideoFrame & )> &callback,
                                  QAbstractVideoSurface *surface, int throttle_interval )
{
  std::string ns = nh->ns().toStdString();
  std::string topic = qtopic.toStdString();
  auto it = subscriptions_.find( ns );
  if ( it == subscriptions_.end())
  {
    it = subscriptions_.insert( { topic, std::make_shared<SubscriptionManager>( nh->nodeHandle()) } ).first;
  }
  std::shared_ptr<SubscriptionManager> &subscription_manager = it->second;
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager->subscriptions;
  try
  {
    size_t i = 0;
    for ( ; i < subscriptions.size(); ++i )
    {
      if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size )
        break; // We could also compare transport type and hints
    }
    auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
    handle->surface = surface;
    handle->callback = callback;
    handle->throttle_interval = throttle_interval;
    if ( i == subscriptions.size())
    {
      auto sub = new Subscription;
      sub->manager = this;
      sub->subscription_manager = subscription_manager;
      sub->topic = topic;
      sub->queue_size = queue_size;
      sub->hints = transport_hints;
      sub->subscribe();
      subscriptions.emplace_back( sub );
      ROS_DEBUG_NAMED( "qml_ros_plugin", "Subscribed to '%s' with transport '%s'.", topic.c_str(),
                       transport_hints.getTransport().c_str());
    }
    handle->subscription = subscriptions[i];
    subscriptions[i]->addSubscription( handle );
    return handle;
  }
  catch ( image_transport::TransportLoadException &ex )
  {
    ROS_ERROR_NAMED( "qml_ros_plugin", "Could not subscribe to image topic: %s", ex.what());
  }
  return nullptr;
}

int ImageTransportManager::getLoadBalancedTimeout( int desired_throttle_interval )
{
  if ( !load_balancing_enabled_ ) return desired_throttle_interval;
  long now = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  long timeout = now + desired_throttle_interval;
  long largest_gap = 0;
  size_t ind_largest_gap = timeouts_.empty() ? 0 : timeouts_[0] - now;
  // Very unsophisticated method. We just find the largest gap before the desired interval and insert the timeout in the middle
  for ( size_t i = 1; i < timeouts_.size(); ++i )
  {
    if ( timeouts_[i] > timeout ) break;
    long gap;
    // Gap is either between two timeouts or between now and the first timeout that is not in the past
    if ( timeouts_[i - 1] < now )
    {
      if ( timeouts_[i] <= now ) continue;
      gap = timeouts_[i] - now;
    }
    else
    {
      gap = timeouts_[i] - timeouts_[i - 1];
    }
    if ( gap <= largest_gap ) continue;
    ind_largest_gap = i;
    largest_gap = gap;
  }
  // If just using the interval would create a new gap that is larger than inserting it inbetween, don't modify
  if ( !timeouts_.empty() && timeout - timeouts_[timeouts_.size() - 1] < largest_gap / 2 )
  {
    timeout = timeouts_[ind_largest_gap] - largest_gap / 2;
    desired_throttle_interval = static_cast<int>(timeout - now);
  }
  // We insert the timeout in the table so that the timeouts are sorted or before now
  size_t i = 0, old = 0;
  for ( ; i < timeouts_.size(); ++i )
  {
    if ( timeouts_[i] < now ) ++old;
    if ( timeouts_[i] > timeout )
    {
      timeouts_.insert( timeouts_.begin() + i, timeout );
      break;
    }
  }
  if ( i == timeouts_.size()) timeouts_.push_back( timeout );
  // If we have more than 10 times before now, we remove them to limit memory moves
  if ( old > 10 ) timeouts_.erase( timeouts_.begin(), timeouts_.begin() + old );
  return desired_throttle_interval;
}

void ImageTransportManager::setLoadBalancingEnabled( bool value )
{
  load_balancing_enabled_ = value;
}

void ImageTransportManagerSingletonWrapper::setLoadBalancingEnabled( bool value )
{
  ImageTransportManager::getInstance().setLoadBalancingEnabled( value );
}
}

#include "image_transport_manager.moc"
