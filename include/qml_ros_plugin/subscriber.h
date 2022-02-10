// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_SUBSCRIBER_H
#define QML_ROS_PLUGIN_SUBSCRIBER_H

#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/qobject_ros.h"

#include <QVariant>
#include <QMap>
#include <QTimer>

#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/babel_fish_message.h>

#include <ros/subscriber.h>

namespace qml_ros_plugin
{
class NodeHandle;

class Subscriber : public QObjectRos
{
Q_OBJECT
  // @formatter:off
  //! The topic this subscriber subscribes to.
  Q_PROPERTY( QString topic READ topic WRITE setTopic NOTIFY topicChanged )
  //! The maximum number of messages that are queued for processing. Default: 10
  Q_PROPERTY( quint32 queueSize READ queueSize WRITE setQueueSize NOTIFY queueSizeChanged )

  //! The namespace of the NodeHandle created for this subscriber.
  Q_PROPERTY( QString ns READ ns WRITE setNs NOTIFY nsChanged )

  //! The last message that was received by this subscriber.
  Q_PROPERTY( QVariant message READ message NOTIFY messageChanged )

  //! The type of the last received message, e.g., geometry_msgs/Pose.
  Q_PROPERTY( QString messageType READ messageType NOTIFY messageTypeChanged )

  //! Limits the frequency in which the notification for an updated message is emitted. Default: 20 Hz
  Q_PROPERTY( int throttleRate READ throttleRate WRITE setThrottleRate NOTIFY throttleRateChanged )

  //! Controls whether or not the subscriber is currently running, i.e., able to receive messages. Default: true
  Q_PROPERTY( bool running READ running WRITE setRunning NOTIFY runningChanged )
  // @formatter:on
public:
  Subscriber();

  explicit Subscriber( NodeHandle::Ptr nh, QString topic, quint32 queue_size, bool running = true );

  ~Subscriber() override;

  QString topic() const;

  void setTopic( const QString &value );

  quint32 queueSize() const;

  void setQueueSize( quint32 value );

  QString ns() const;

  void setNs( const QString &value );

  bool running() const;

  void setRunning( bool value );

  int throttleRate() const;

  void setThrottleRate( int value );

  const QVariant &message() const;

  const QString &messageType() const;

  //! @return The number of publishers this subscriber is connected to.
  Q_INVOKABLE unsigned int getNumPublishers();

signals:

  //! Emitted when the topic changed.
  void topicChanged();

  //! Emitted when the queueSize changed.
  void queueSizeChanged();

  //! Emitted when the namespace ns changed.
  void nsChanged();

  //! Emitted when the throttle rate was changed.
  void throttleRateChanged();

  //! Emitted if the running state of this subscriber changed.
  void runningChanged();

  //! Emitted whenever a new message was received.
  void messageChanged();

  //! Emitted whenever the type of the last received message changed.
  void messageTypeChanged();

  /*!
   * Emitted whenever a new message was received.
   * @param message The received message.
   */
  void newMessage( QVariant message );

protected slots:

  void subscribe();

  void updateMessage();

protected:
  void initTimer();

  void onRosInitialized() override;

  void shutdown();

  void messageCallback( const ros_babel_fish::BabelFishMessage::ConstPtr &msg );

  NodeHandle::Ptr nh_;
  ros::Subscriber subscriber_;
  ros_babel_fish::BabelFish babel_fish_;
  ros_babel_fish::BabelFishMessage::ConstPtr last_message_;
  ros_babel_fish::BabelFishMessage::ConstPtr current_message_;
  QTimer throttle_timer_;
  bool running_;
  bool is_subscribed_;

  QString topic_;
  QString message_type_;
  QVariant message_;
  quint32 queue_size_;
  int throttle_rate_ = 20;
};
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_SUBSCRIBER_H
