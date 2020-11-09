// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_ROS_H
#define QML_ROS_PLUGIN_ROS_H

#include "qml_ros_plugin/console.h"
#include "qml_ros_plugin/io.h"
#include "qml_ros_plugin/node_handle.h"
#include "qml_ros_plugin/package.h"
#include "qml_ros_plugin/topic_info.h"

#include <QJSValue>
#include <QObject>
#include <QTimer>

namespace ros
{
class AsyncSpinner;
}

//! @brief Project namespace.
namespace qml_ros_plugin
{

/*!
 * @brief Options that can be passed to Ros.init.
 *
 * See ros::init_options for detailed documentation.
 * Usage in QML:
 * @code
 * Ros.init("name", RosInitOptions.NoRosout | RosInitOptions.NoSigintHandler)
 * @endcode
 */
namespace ros_init_options
{
Q_NAMESPACE

enum RosInitOption
{
  //! Don't install a SIGINT handler.
  NoSigintHandler = ros::init_options::NoSigintHandler,
  //! Anonymize the node name. Adds a random number to the node's name to make it unique.
  AnonymousName = ros::init_options::AnonymousName,
  //! Don't broadcast rosconsole output to the /rosout topic.
  NoRosout = ros::init_options::NoRosout
};

Q_ENUM_NS( RosInitOption )
}

class RosQml : public QObject
{
Q_OBJECT
private:
  RosQml();

public:
  static RosQml &getInstance();

  RosQml( const RosQml & ) = delete;

  void operator=( const RosQml & ) = delete;

  /*!
   * Checks whether ROS is initialized.
   * @return True if ROS is initialized, false otherwise.
   */
  bool isInitialized() const;

  /*!
   * Initializes the ros node with the given name and the command line arguments passed from the command line.
   * @param name The name of the ROS node.
   * @param options The options passed to ROS, see ros_init_options::RosInitOption.
   */
  void init( const QString &name, quint32 options = 0 );

  /*!
   * Initializes the ros node with the given args.
   * @param args The args that are passed to ROS. Normally, these would be the command line arguments see init(const QString &, quint32)
   * @param name The name of the ROS node.
   * @param options The options passed to ROS, see ros_init_options::RosInitOption.
   */
  void init( const QStringList &args, const QString &name, quint32 options = 0 );

  /*!
   * Can be used to query the state of ROS.
   * @return False if it's time to exit, true if still ok.
   */
  bool ok() const;

  /*!
   * Processes a single round of callbacks. This call is non-blocking as the queue will be called on a detached thread.
   * Not needed unless you disable the AsyncSpinner using setThreads(int) with the argument 0.
   */
  void spinOnce();

  /*!
   * Sets the thread count for the AsyncSpinner. If asynchronous spinning is disabled, you have to manually call
   * spinOnce() to receive and publish messages.
   * @param count How many threads the AsyncSpinner will use. Set to zero to disable spinning. Default: 1
   */
  void setThreads( int count );

  /*!
   * Queries the ROS master for its topics or using the optional datatype parameter for all topics with the given type.
   * @param datatype The message type to filter topics for, e.g., sensor_msgs/Image. Omit to query for all topics.
   * @return A list of topics that matches the given datatype or all topics if no datatype provided.
   */
  QStringList queryTopics( const QString &datatype = QString()) const;

  /*!
   * Queries the ROS master for its topics and their type.
   * @return A list of TopicInfo.
   */
  QList<TopicInfo> queryTopicInfo() const;

  /*!
   * Queries the ROS master for a topic with the given name.
   * @param name The name of the topic, e.g., /front_camera/image_raw.
   * @return The type of the topic if found, otherwise an empty string.
   */
  QString queryTopicType( const QString &name ) const;


  Console console() const;

  Package package() const;

  /*!
   * A callback queue that is guaranteed to be called on a background thread.
   * @return A shared pointer to the callback queue.
   */
  std::shared_ptr<ros::CallbackQueue> callbackQueue();

signals:

  //! Emitted once when ROS was initialized.
  void initialized();

  //! Emitted when this ROS node was shut down and it is time to exit.
  void shutdown();

protected slots:

  void checkInitialized();

  void checkShutdown();

private:
  void onInitialized();

  void updateSpinner();

  QTimer timer_;
  std::shared_ptr<ros::CallbackQueue> callback_queue_;
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  int threads_;
  bool initialized_;
};

class RosQmlSingletonWrapper : public QObject
{
Q_OBJECT

  // @formatter:off
  Q_PROPERTY( qml_ros_plugin::Console console READ console CONSTANT )
  Q_PROPERTY( qml_ros_plugin::Package package READ package CONSTANT )
  Q_PROPERTY( qml_ros_plugin::IO io READ io CONSTANT )
  Q_PROPERTY( QJSValue debug READ debug CONSTANT )
  Q_PROPERTY( QJSValue info READ info CONSTANT )
  Q_PROPERTY( QJSValue warn READ warn CONSTANT )
  Q_PROPERTY( QJSValue error READ error CONSTANT )
  Q_PROPERTY( QJSValue fatal READ fatal CONSTANT )
  // @formatter:on
public:
  RosQmlSingletonWrapper();

  ~RosQmlSingletonWrapper() override;

  //! @copydoc RosQml::isInitialized
  Q_INVOKABLE bool isInitialized() const;

  //! @copydoc RosQml::init(const QString &, quint32)
  Q_INVOKABLE void init( const QString &name, quint32 options = 0 );

  //! @copydoc RosQml::init(const QStringList &, const QString &, quint32)
  Q_INVOKABLE void init( const QStringList &args, const QString &name, quint32 options = 0 );

  //! @copydoc RosQml::ok
  Q_INVOKABLE bool ok() const;

  //! @copydoc RosQml::spinOnce
  Q_INVOKABLE void spinOnce();

  //! @copydoc RosQml::setThreads
  Q_INVOKABLE void setThreads( int count );

  //! @copydoc RosQml::queryTopics
  Q_INVOKABLE QStringList queryTopics( const QString &datatype = QString()) const;

  //! @copydoc RosQml::queryTopicInfo
  Q_INVOKABLE QList<TopicInfo> queryTopicInfo() const;

  //! @copydoc RosQml::queryTopicType
  Q_INVOKABLE QString queryTopicType( const QString &name ) const;

  Console console() const;

  Package package() const;

  IO io() const;

  /*!
   * Outputs a ROS debug message. The equivalent of calling ROS_DEBUG in C++.
   * The signature in QML is @c debug(msg, consoleName) where @a consoleName is an optional parameter which defaults to
   * @a ros.qml_ros_plugin.
   *
   * @b Note: The msg is not a format string.
   *
   * Example:
   * @code
   * Ros.debug("The value of x is: " + x);
   * @endcode
   */
  QJSValue debug();

  /*!
   * Outputs a ROS info message. The equivalent of calling ROS_INFO in C++.
   * The signature in QML is @c info(msg, consoleName) where @a consoleName is an optional parameter which defaults to
   * @a ros.qml_ros_plugin.
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros.info("The value of x is: " + x);
   * @endcode
   */
  QJSValue info();

  /*!
   * Outputs a ROS warn message. The equivalent of calling ROS_WARN in C++.
   * The signature in QML is @c warn(msg, consoleName) where @a consoleName is an optional parameter which defaults to
   * @a ros.qml_ros_plugin.
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros.warn("The value of x is: " + x);
   * @endcode
   */
  QJSValue warn();

  /*!
   * Outputs a ROS error message. The equivalent of calling ROS_ERROR in C++.
   * The signature in QML is @c error(msg, consoleName) where @a consoleName is an optional parameter which defaults to
   * @a ros.qml_ros_plugin.
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros.error("The value of x is: " + x);
   * @endcode
   */
  QJSValue error();

  /*!
   * Outputs a ROS fatal message. The equivalent of calling ROS_FATAL in C++.
   * The signature in QML is @c fatal(msg, consoleName) where @a consoleName is an optional parameter which defaults to
   * @a ros.qml_ros_plugin.
   *
   * @b Note: The argument is not a format string.
   *
   * Example:
   * @code
   * Ros.fatal("The value of x is: " + x);
   * @endcode
   */
  QJSValue fatal();

  Q_INVOKABLE void logInternal( int level, const QString &function,
                                const QString &file, int line, const QString &msg,
                                const QString &name = QString()) const;

  /*!
   * Creates a Publisher to publish ROS messages.
   *
   * @param type The type of the messages published using this publisher.
   * @param topic The topic on which the messages are published.
   * @param queue_size The maximum number of outgoing messages to be queued for delivery to subscribers.
   * @param latch Whether or not this publisher should latch, i.e., always send out the last message to new subscribers.
   * @return A Publisher instance.
   */
  Q_INVOKABLE QObject *advertise( const QString &type, const QString &topic, quint32 queue_size, bool latch = false );

  /*!
   * Creates a Publisher to publish ROS messages.
   *
   * @param ns The namespace for this publisher.
   * @param type The type of the messages published using this publisher.
   * @param topic The topic on which the messages are published.
   * @param queue_size The maximum number of outgoing messages to be queued for delivery to subscribers.
   * @param latch Whether or not this publisher should latch, i.e., always send out the last message to new subscribers.
   * @return A Publisher instance.
   */
  Q_INVOKABLE QObject *advertise( const QString &ns, const QString &type, const QString &topic, quint32 queue_size,
                                  bool latch = false );

  /*!
   * Creates a Subscriber to subscribe to ROS messages.
   * Convenience function to create a subscriber in a single line.
   *
   * @param topic The topic to subscribe to.
   * @param queue_size The maximum number of incoming messages to be queued for processing.
   * @return A Subscriber instance.
   */
  Q_INVOKABLE QObject *subscribe( const QString &topic, quint32 queue_size );

  /*!
   * Creates a Subscriber to subscribe to ROS messages.
   * Convenience function to create a subscriber in a single line.
   *
   * @param ns The namespace for this Subscriber.
   * @param topic The topic to subscribe to.
   * @param queue_size The maximum number of incoming messages to be queued for processing.
   * @return A Subscriber instance.
   */
  Q_INVOKABLE QObject *subscribe( const QString &ns, const QString &topic, quint32 queue_size );

  /*!
   * Creates an ActionClient for the given type and the given name.
   * @param type The type of the action (which always ends with 'Action'). Example: actionlib_tutorials/FibonacciAction
   * @param name The name of the action server to connect to. This is essentially a base topic.
   * @return An instance of ActionClient.
   */
  Q_INVOKABLE QObject *createActionClient( const QString &type, const QString &name );

  /*!
   * Creates an ActionClient for the given type and the given name.
   * @param ns The namespace for this ActionClient.
   * @param type The type of the action (which always ends with 'Action'). Example: actionlib_tutorials/FibonacciAction
   * @param name The name of the action server to connect to. This is essentially a base topic.
   * @return An instance of ActionClient.
   */
  Q_INVOKABLE QObject *createActionClient( const QString &ns, const QString &type, const QString &name );

signals:

  //! @copydoc RosQml::initialized
  void initialized();

  //! @copydoc RosQml::shutdown
  void shutdown();

private:
  NodeHandle::Ptr findOrCreateNodeHandle( const QString &ns );

  QJSValue createLogFunction( ros_console_levels::RosConsoleLevel level );

  std::map<QString, NodeHandle::Ptr> node_handles_;
  QJSValue log_function_;
  QJSValue debug_function_;
  QJSValue info_function_;
  QJSValue warn_function_;
  QJSValue error_function_;
  QJSValue fatal_function_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_ROS_H
