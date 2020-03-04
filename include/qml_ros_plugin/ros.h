// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_ROS_H
#define QML_ROS_PLUGIN_ROS_H

#include "qml_ros_plugin/console.h"
#include "qml_ros_plugin/node_handle.h"

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

  bool isInitialized();

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
   * Processes a single round of callbacks.
   * Not needed unless you disable the AsyncSpinner using setThreads(int) with the argument 0.
   */
  void spinOnce();

  /*!
   * Sets the thread count for the AsyncSpinner. If asynchronous spinning is disabled, you have to manually call
   * spinOnce() to receive and publish messages.
   * @param count How many threads the AsyncSpinner will use. Set to zero to disable spinning.
   */
  void setThreads( int count );

  Console console() const;

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
  std::unique_ptr<ros::AsyncSpinner> spinner_;
  int threads_;
  bool initialized_;
};

/*!
 *
 */
class RosQmlSingletonWrapper : public QObject
{
Q_OBJECT

  // @formatter:off
  Q_PROPERTY( qml_ros_plugin::Console console READ console )
  Q_PROPERTY( QJSValue debug READ debug )
  Q_PROPERTY( QJSValue info READ info )
  Q_PROPERTY( QJSValue warn READ warn )
  Q_PROPERTY( QJSValue error READ error )
  Q_PROPERTY( QJSValue fatal READ fatal )
  // @formatter:on
public:
  RosQmlSingletonWrapper();

  ~RosQmlSingletonWrapper() override;

  //! @copydoc RosQml::init(const QString &, quint32)
  Q_INVOKABLE void init( const QString &name, quint32 options = 0 );

  //! @copydoc RosQml::init(const QStringList &, const QString &, quint32)
  Q_INVOKABLE void init( const QStringList &args, const QString &name, quint32 options = 0 );

  //! @copydoc RosQml::ok
  Q_INVOKABLE bool ok() const;

  Console console() const;

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
   * @param ns The namespace for this publisher.
   * @copydoc advertise
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
   * @param ns The namespace for this Subscriber.
   * @copydoc subscribe
   */
  Q_INVOKABLE QObject *subscribe( const QString &ns, const QString &topic, quint32 queue_size );

signals:

  //! @copydoc RosQml::initialized
  void initialized();

  //! @copydoc RosQml::shutdown
  void shutdown();

private:
  QJSValue createLogFunction( ros_console_levels::RosConsoleLevel level );

  std::map<std::string, NodeHandle *> node_handles_;
  QJSValue log_function_;
  QJSValue debug_function_;
  QJSValue info_function_;
  QJSValue warn_function_;
  QJSValue error_function_;
  QJSValue fatal_function_;
};
} // qml_ros_plugin

#endif // QML_ROS_PLUGIN_ROS_H
