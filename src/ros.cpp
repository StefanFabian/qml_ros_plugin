// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/ros.h"
#include "qml_ros_plugin/action_client.h"
#include "qml_ros_plugin/babel_fish_dispenser.h"
#include "qml_ros_plugin/message_conversions.h"
#include "qml_ros_plugin/publisher.h"
#include "qml_ros_plugin/qml_ros_conversion.h"
#include "qml_ros_plugin/subscriber.h"

#include <ros/master.h>
#include <ros/ros.h>

#include <QCoreApplication>
#include <QJSEngine>
#include <thread>

namespace qml_ros_plugin
{

RosQml &RosQml::getInstance()
{
  static RosQml instance;
  return instance;
}

RosQml::RosQml() : threads_( 1 ), initialized_( false )
{
  callback_queue_ = std::make_shared<ros::CallbackQueue>();
  connect( &timer_, &QTimer::timeout, this, &RosQml::checkInitialized );
  if ( ros::isInitialized() ) {
    onInitialized();
  }
  timer_.setInterval( 33 );
  timer_.start();
}

bool RosQml::isInitialized() const { return initialized_; }

void RosQml::init( const QString &name, quint32 options )
{
  const QStringList &arguments = QCoreApplication::arguments();
  init( arguments, name, options );
}

void RosQml::init( const QStringList &argv, const QString &name, quint32 options )
{
  if ( ros::isInitialized() )
    return;
  timer_.stop();
  int argc = argv.size();
  char **cargv = new char *[argc];
  for ( int i = 0; i < argv.size(); ++i ) {
    cargv[i] = new char[argv[i].length() + 1];
    std::string string = argv[i].toStdString();
    std::copy( string.begin(), string.end(), cargv[i] );
  }
  ros::init( argc, cargv, name.toStdString(), options );
  emit initialized();
  for ( int i = 0; i < argv.size(); ++i ) { delete[] cargv[i]; }
  delete[] cargv;
  onInitialized();
  timer_.start();
}

bool RosQml::ok() const { return ros::ok(); }

void RosQml::setThreads( int count )
{
  threads_ = count;
  updateSpinner();
}

QStringList RosQml::queryTopics( const QString &datatype ) const
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics( topic_info );
  QStringList result;
  std::string std_datatype = datatype.toStdString();
  for ( const auto &topic : topic_info ) {
    if ( !std_datatype.empty() && topic.datatype != std_datatype )
      continue;
    result.append( QString::fromStdString( topic.name ) );
  }
  return result;
}

QList<TopicInfo> RosQml::queryTopicInfo() const
{
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics( topic_info );
  QList<TopicInfo> result;
  for ( const auto &topic : topic_info ) {
    result.append(
        { QString::fromStdString( topic.name ), QString::fromStdString( topic.datatype ) } );
  }
  return result;
}

QString RosQml::queryTopicType( const QString &name ) const
{
  if ( name.isEmpty() )
    return {};
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics( topic_info );
  QList<TopicInfo> result;
  std::string std_name = name.toStdString();
  for ( const auto &topic : topic_info ) {
    if ( std_name != topic.name )
      continue;
    return QString::fromStdString( topic.datatype );
  }
  return {};
}

QVariant RosQml::createEmptyMessage( const QString &datatype ) const
{
  try {
    auto message = BabelFishDispenser::getBabelFish().createMessage( datatype.toStdString() );
    return conversion::msgToMap( *message );
  } catch ( ros_babel_fish::BabelFishMessageException &ex ) {
    ROS_WARN_NAMED( "qml_ros_plugin", "Failed to create empty message for datatype '%s': %s",
                    datatype.toStdString().c_str(), ex.what() );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    ROS_WARN_NAMED( "qml_ros_plugin", "Failed to create empty message for datatype '%s': %s",
                    datatype.toStdString().c_str(), ex.what() );
  }
  return QVariant();
}

QVariant RosQml::createEmptyServiceRequest( const QString &datatype ) const
{
  try {
    auto message = BabelFishDispenser::getBabelFish().createServiceRequest( datatype.toStdString() );
    return conversion::msgToMap( *message );
  } catch ( ros_babel_fish::BabelFishMessageException &ex ) {
    ROS_WARN_NAMED( "qml_ros_plugin", "Failed to create empty service request for datatype '%s': %s",
                    datatype.toStdString().c_str(), ex.what() );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    ROS_WARN_NAMED( "qml_ros_plugin", "Failed to create empty service request for datatype '%s': %s",
                    datatype.toStdString().c_str(), ex.what() );
  }
  return QVariant();
}

void RosQml::checkInitialized()
{
  if ( !ros::isInitialized() )
    return;
  onInitialized();
  emit initialized();
}

void RosQml::checkShutdown()
{
  if ( ros::ok() )
    return;
  emit shutdown();
  timer_.stop();
  if ( spinner_ != nullptr )
    spinner_->stop();
}

void RosQml::onInitialized()
{
  if ( initialized_ )
    return;
  initialized_ = true;
  disconnect( &timer_, &QTimer::timeout, this, &RosQml::checkInitialized );
  connect( &timer_, &QTimer::timeout, this, &RosQml::checkShutdown );
  updateSpinner();
}

void RosQml::spinOnce()
{
  if ( !initialized_ )
    return;
  std::thread background_thread( [this]() { callback_queue_->callAvailable(); } );
  background_thread.detach();
}

void RosQml::updateSpinner()
{
  if ( spinner_ )
    spinner_->stop();
  if ( threads_ == 0 ) {
    spinner_.reset();
    return;
  }
  spinner_.reset( new ros::AsyncSpinner( threads_, callback_queue_.get() ) );
  if ( count_start_spinning_ > 0 )
    spinner_->start();
}

Console RosQml::console() const { return {}; }

Package RosQml::package() const { return {}; }

std::shared_ptr<ros::CallbackQueue> RosQml::callbackQueue() { return callback_queue_; }

void RosQml::startSpinning()
{
  if ( count_start_spinning_++ == 0 ) {
    if ( spinner_ )
      spinner_->start();
  }
}

void RosQml::stopSpinning()
{
  int count = --count_start_spinning_;
  if ( count == 0 ) {
    if ( spinner_ )
      spinner_->stop();
  } else if ( count < 0 ) {
    ROS_ERROR_NAMED( "qml_ros_plugin",
                     "Stop spinning was called more often than start spinning! This is a bug!" );
    count_start_spinning_ += -count;
  }
}

/***************************************************************************************************/
/************************************* RosQmlSingletonWrapper **************************************/
/***************************************************************************************************/

RosQmlSingletonWrapper::RosQmlSingletonWrapper() : id_counter_( 0 )
{
  connect( &RosQml::getInstance(), &RosQml::initialized, this, &RosQmlSingletonWrapper::initialized );
  connect( &RosQml::getInstance(), &RosQml::shutdown, this, &RosQmlSingletonWrapper::shutdown );
  RosQml::getInstance().startSpinning();
}

RosQmlSingletonWrapper::~RosQmlSingletonWrapper() { RosQml::getInstance().stopSpinning(); }

bool RosQmlSingletonWrapper::isInitialized() const { return RosQml::getInstance().isInitialized(); }

void RosQmlSingletonWrapper::init( const QString &name, quint32 options )
{
  RosQml::getInstance().init( name, options );
}

void RosQmlSingletonWrapper::init( const QStringList &args, const QString &name, quint32 options )
{
  RosQml::getInstance().init( args, name, options );
}

bool RosQmlSingletonWrapper::ok() const { return RosQml::getInstance().ok(); }

void RosQmlSingletonWrapper::spinOnce() { RosQml::getInstance().spinOnce(); }

void RosQmlSingletonWrapper::setThreads( int count ) { RosQml::getInstance().setThreads( count ); }

QString RosQmlSingletonWrapper::getName()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( ros::this_node::getName() );
}

QString RosQmlSingletonWrapper::getNamespace()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( ros::this_node::getNamespace() );
}

QStringList RosQmlSingletonWrapper::queryTopics( const QString &datatype ) const
{
  return RosQml::getInstance().queryTopics( datatype );
}

QList<TopicInfo> RosQmlSingletonWrapper::queryTopicInfo() const
{
  return RosQml::getInstance().queryTopicInfo();
}

QString RosQmlSingletonWrapper::queryTopicType( const QString &name ) const
{
  return RosQml::getInstance().queryTopicType( name );
}

QVariant RosQmlSingletonWrapper::createEmptyMessage( const QString &datatype ) const
{
  return RosQml::getInstance().createEmptyMessage( datatype );
}

QVariant RosQmlSingletonWrapper::createEmptyServiceRequest( const QString &datatype ) const
{
  return RosQml::getInstance().createEmptyServiceRequest( datatype );
}
void RosQmlSingletonWrapper::waitForMessageAsync( const QString &topic, const QJSValue &callback )
{
  waitForMessageAsync( topic, 0, callback );
}

void RosQmlSingletonWrapper::waitForMessageAsync( const QString &topic, double duration,
                                                  const QJSValue &callback )
{
  using ros::topic::waitForMessage;
  using namespace ros_babel_fish;
  // Insert the callback into a container and pass the id to the message wait thread
  // Passing the callback to the thread would lead to random segfaults because the QJSValue is not
  // supposed to leave the thread of the QJSEngine
  int id = ++id_counter_;
  callbacks_.insert( id, callback );
  std::string std_topic = topic.toStdString();
  std::thread( [this, std_topic, id, duration]() {
    ros::NodeHandle nh;
    nh.setCallbackQueue( RosQml::getInstance().callbackQueue().get() );
    auto msg = waitForMessage<BabelFishMessage>( std_topic, nh, qmlToRosDuration( duration ) );
    if ( msg == nullptr ) {
      QMetaObject::invokeMethod( this, "invokeCallback", Qt::QueuedConnection, Q_ARG( int, id ),
                                 Q_ARG( QVariant, QVariant() ) );
      return;
    }
    TranslatedMessage::Ptr response = BabelFishDispenser::getBabelFish().translateMessage( msg );
    QMetaObject::invokeMethod(
        this, "invokeCallback", Qt::QueuedConnection, Q_ARG( int, id ),
        Q_ARG( QVariant, conversion::msgToMap( response, *response->translated_message ) ) );
  } ).detach();
}

QString RosQmlSingletonWrapper::getEnvironmentVariable( const QString &name ) const
{
  return qEnvironmentVariable( name.toStdString().c_str() );
}

Console RosQmlSingletonWrapper::console() const { return RosQml::getInstance().console(); }

Package RosQmlSingletonWrapper::package() const { return RosQml::getInstance().package(); }

IO RosQmlSingletonWrapper::io() const { return {}; }

QJSValue RosQmlSingletonWrapper::debug()
{
  if ( debug_function_.isCallable() )
    return debug_function_;
  return debug_function_ = createLogFunction( ros_console_levels::Debug );
}

QJSValue RosQmlSingletonWrapper::info()
{
  if ( info_function_.isCallable() )
    return info_function_;
  return info_function_ = createLogFunction( ros_console_levels::Info );
}

QJSValue RosQmlSingletonWrapper::warn()
{
  if ( warn_function_.isCallable() )
    return warn_function_;
  return warn_function_ = createLogFunction( ros_console_levels::Warn );
}

QJSValue RosQmlSingletonWrapper::error()
{
  if ( error_function_.isCallable() )
    return error_function_;
  return error_function_ = createLogFunction( ros_console_levels::Error );
}

QJSValue RosQmlSingletonWrapper::fatal()
{
  if ( fatal_function_.isCallable() )
    return fatal_function_;
  return fatal_function_ = createLogFunction( ros_console_levels::Fatal );
}

void RosQmlSingletonWrapper::logInternal( int level, const QString &function, const QString &file,
                                          int line, const QString &msg, const QString &name ) const
{
  std::string logger_name = name.isEmpty()
                                ? ROSCONSOLE_DEFAULT_NAME
                                : std::string( ROSCONSOLE_NAME_PREFIX ) + "." + name.toStdString();
  ROSCONSOLE_DEFINE_LOCATION( true, static_cast<::ros::console::Level>( level ), logger_name );
  if ( ROS_UNLIKELY( __rosconsole_define_location__enabled ) ) {
    ::ros::console::print( nullptr, __rosconsole_define_location__loc.logger_,
                           static_cast<::ros::console::Level>( level ), file.toStdString().c_str(),
                           line, function.toStdString().c_str(), "%s", msg.toStdString().c_str() );
  }
}

QObject *RosQmlSingletonWrapper::advertise( const QString &type, const QString &topic,
                                            quint32 queue_size, bool latch )
{
  return advertise( QString(), type, topic, queue_size, latch );
}

QObject *RosQmlSingletonWrapper::advertise( const QString &ns, const QString &type,
                                            const QString &topic, quint32 queue_size, bool latch )
{
  NodeHandle::Ptr node_handle = findOrCreateNodeHandle( ns );
  return new Publisher( node_handle, type, topic, queue_size, latch );
}

QObject *RosQmlSingletonWrapper::subscribe( const QString &topic, quint32 queue_size )
{
  return subscribe( QString(), topic, queue_size );
}

QObject *RosQmlSingletonWrapper::subscribe( const QString &ns, const QString &topic,
                                            quint32 queue_size )
{
  NodeHandle::Ptr node_handle = findOrCreateNodeHandle( ns );
  return new Subscriber( node_handle, topic, queue_size );
}

QObject *RosQmlSingletonWrapper::createActionClient( const QString &type, const QString &name )
{
  return createActionClient( QString(), type, name );
}

QObject *RosQmlSingletonWrapper::createActionClient( const QString &ns, const QString &type,
                                                     const QString &name )
{
  NodeHandle::Ptr node_handle = findOrCreateNodeHandle( ns );
  return new ActionClient( node_handle, type, name );
}
void RosQmlSingletonWrapper::invokeCallback( int id, const QVariant &result )
{
  QJSValue callback = callbacks_[id];
  callbacks_.remove( id );
  if ( !callback.isCallable() )
    return;
  QJSEngine *engine = qjsEngine( this );
  callback.call( { engine->toScriptValue( result ) } );
}

NodeHandle::Ptr RosQmlSingletonWrapper::findOrCreateNodeHandle( const QString &ns )
{
  auto it = node_handles_.find( ns );
  if ( it == node_handles_.end() ) {
    node_handles_.insert( { ns, std::make_shared<NodeHandle>( ns.toStdString() ) } );
    it = node_handles_.find( ns );
  }
  return it->second;
}

QJSValue RosQmlSingletonWrapper::createLogFunction( ros_console_levels::RosConsoleLevel level )
{
  auto engine = qjsEngine( this );
  if ( !engine )
    return {};
  // This function extracts the file, method and number which called the log function in order to accurately report it.
  QJSValue func =
      engine->evaluate( R"js((function (__ros_instance) {
  return (function (msg, name) {
    var stack = new Error().stack.split('\n');
    if (stack && stack.length >= 2) {
      var call_info = stack[1].split('@');
      var method = 'unknown', file = 'unknown', line = 0;
      if (call_info && call_info.length >= 2) {
        method = call_info[0];
        var file_info = call_info[1].replace('file://', '');
        var line_sep = file_info.lastIndexOf(':');
        if (line_sep != -1) {
          file = file_info.substr(0, line_sep);
          line = Number(file_info.substr(line_sep + 1));
        }
      }
    }
    __ros_instance.logInternal()js" +
                        QString::number( level ) + R"js(, method, file, Number(line), msg, name);
  });
}))js" );
  return func.call( { engine->newQObject( this ) } );
}
} // namespace qml_ros_plugin
