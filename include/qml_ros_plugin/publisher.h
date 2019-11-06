// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_PUBLISHER_H
#define QML_ROS_PLUGIN_PUBLISHER_H

#include <QObject>
#include <QVariant>
#include <QMap>
#include <QTimer>

#include <ros_babel_fish/babel_fish.h>

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/single_subscriber_publisher.h>

namespace qml_ros_plugin
{
class NodeHandle;

class Publisher : public QObject
{
Q_OBJECT
  // @formatter:off
  //! The type of the published messages, e.g., geometry_msgs/Pose.
  Q_PROPERTY( QString type READ type )
  //! The topic this Publisher publishes messages on. This property is only valid if the publisher is already advertised!
  Q_PROPERTY( QString topic READ topic )
  //! Whether or not this Publisher is latched. A latched Publisher always sends the last message to new subscribers.
  Q_PROPERTY( bool isLatched READ isLatched )
  //! The queue size of this Publisher. This is the maximum number of messages that are queued for delivery to subscribers at a time.
  Q_PROPERTY( quint32 queueSize READ queueSize )
  //! Whether or not this publisher has advertised its existence on its topic.
  //! Reasons for not being advertised include ROS not being initialized yet.
  Q_PROPERTY( bool isAdvertised READ isAdvertised NOTIFY advertised )
  // @formatter:on
public:
  Publisher( NodeHandle *nh, QString type, QString topic, uint32_t queue_size, bool latch );

  ~Publisher() override;

  QString topic() const;

  const QString &type() const;

  bool isLatched() const;

  quint32 queueSize() const;

  bool isAdvertised() const;

  //! @return The number of subscribers currently connected to this Publisher.
  Q_INVOKABLE unsigned int getNumSubscribers();

  /*!
   * Sends a message to subscribers currently connected to this Publisher.
   * @param msg The message that is published.
   * @return True if the message was sent successfully, false otherwise.
   */
  Q_INVOKABLE bool publish( const QVariantMap &msg );

signals:

  /*!
   * Fired once this Publisher was advertised.
   * This is either done at construction or immediately after ROS is initialized.
   * Since this is only fired once, you should check if the Publisher is already advertised using the isAdvertised property.
   */
  void advertised();

  //! Fired whenever a new subscriber connects to this Publisher.
  void connected();

  //! Fired whenever a subscriber disconnects from this Publisher.
  void disconnected();

protected slots:

  void onNodeHandleReady();

protected:
  void advertise();

  void onSubscriberConnected( const ros::SingleSubscriberPublisher &pub );

  void onSubscriberDisconnected( const ros::SingleSubscriberPublisher &pub );

  NodeHandle *nh_;
  ros::Publisher publisher_;
  ros_babel_fish::BabelFish babel_fish_;

  bool is_advertised_;
  QString type_;
  std::string std_type_;
  QString topic_;
  bool is_latched_;
  uint32_t queue_size_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_PUBLISHER_H
