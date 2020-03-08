// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/ros.h"
#include "qml_ros_plugin/publisher.h"
#include "qml_ros_plugin/subscriber.h"

#include <ros/ros.h>

#include <QCoreApplication>
#include <QJSEngine>

namespace qml_ros_plugin
{

RosQml &RosQml::getInstance()
{
  static RosQml instance;
  return instance;
}

RosQml::RosQml() : threads_( 1 ), initialized_( false )
{
  connect( &timer_, &QTimer::timeout, this, &RosQml::checkInitialized );
  if ( ros::isInitialized())
  {
    onInitialized();
  }
  timer_.setInterval( 33 );
  timer_.start();
}

bool RosQml::isInitialized() { return initialized_; }

void RosQml::init( const QString &name, quint32 options )
{
  const QStringList &arguments = QCoreApplication::arguments();
  init( arguments, name, options );
}

void RosQml::init( const QStringList &argv, const QString &name, quint32 options )
{
  if ( ros::isInitialized()) return;
  timer_.stop();
  int argc = argv.size();
  char **cargv = new char *[argc];
  for ( int i = 0; i < argv.size(); ++i )
  {
    cargv[i] = new char[argv[i].length() + 1];
    std::string string = argv[i].toStdString();
    std::copy( string.begin(), string.end(), cargv[i] );
  }
  ros::init( argc, cargv, name.toStdString(), options );
  emit initialized();
  for ( int i = 0; i < argv.size(); ++i )
  {
    delete[] cargv[i];
  }
  delete[] cargv;
  onInitialized();
  timer_.start();
}

bool RosQml::ok() const
{
  return ros::ok();
}

void RosQml::setThreads( int count )
{
  threads_ = count;
  updateSpinner();
}

void RosQml::checkInitialized()
{
  if ( !ros::isInitialized()) return;
  onInitialized();
  emit initialized();
}

void RosQml::checkShutdown()
{
  if ( ros::ok()) return;
  emit shutdown();
  timer_.stop();
  if ( spinner_ != nullptr ) spinner_->stop();
}

void RosQml::onInitialized()
{
  if ( initialized_ ) return;
  initialized_ = true;
  disconnect( &timer_, &QTimer::timeout, this, &RosQml::checkInitialized );
  connect( &timer_, &QTimer::timeout, this, &RosQml::checkShutdown );
  updateSpinner();
}

void RosQml::spinOnce()
{
  ros::spinOnce();
}

void RosQml::updateSpinner()
{
  if ( spinner_ ) spinner_->stop();
  if ( threads_ == 0 )
  {
    spinner_.reset();
    return;
  }
  spinner_.reset( new ros::AsyncSpinner( threads_ ));
  spinner_->start();
}

Console RosQml::console() const
{
  return {};
}

/***************************************************************************************************/
/************************************* RosQmlSingletonWrapper **************************************/
/***************************************************************************************************/

RosQmlSingletonWrapper::RosQmlSingletonWrapper()
{
  connect( &RosQml::getInstance(), &RosQml::initialized, this, &RosQmlSingletonWrapper::initialized );
  connect( &RosQml::getInstance(), &RosQml::shutdown, this, &RosQmlSingletonWrapper::shutdown );
}

RosQmlSingletonWrapper::~RosQmlSingletonWrapper()
{
  for ( auto &pair : node_handles_ )
  {
    delete pair.second;
  }
  node_handles_.clear();
}

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

Console RosQmlSingletonWrapper::console() const { return RosQml::getInstance().console(); }

QJSValue RosQmlSingletonWrapper::debug()
{
  if ( debug_function_.isCallable()) return debug_function_;
  return debug_function_ = createLogFunction( ros_console_levels::Debug );
}

QJSValue RosQmlSingletonWrapper::info()
{
  if ( info_function_.isCallable()) return info_function_;
  return info_function_ = createLogFunction( ros_console_levels::Info );
}

QJSValue RosQmlSingletonWrapper::warn()
{
  if ( warn_function_.isCallable()) return warn_function_;
  return warn_function_ = createLogFunction( ros_console_levels::Warn );
}

QJSValue RosQmlSingletonWrapper::error()
{
  if ( error_function_.isCallable()) return error_function_;
  return error_function_ = createLogFunction( ros_console_levels::Error );
}

QJSValue RosQmlSingletonWrapper::fatal()
{
  if ( fatal_function_.isCallable()) return fatal_function_;
  return fatal_function_ = createLogFunction( ros_console_levels::Fatal );
}

void RosQmlSingletonWrapper::logInternal( int level, const QString &function,
                                          const QString &file, int line, const QString &msg, const QString &name ) const
{
  std::string logger_name = name.isEmpty() ? ROSCONSOLE_DEFAULT_NAME
                                           : std::string( ROSCONSOLE_NAME_PREFIX ) + "." + name.toStdString();
  ROSCONSOLE_DEFINE_LOCATION( true, static_cast<::ros::console::Level>(level), logger_name );
  if ( ROS_UNLIKELY( __rosconsole_define_location__enabled ))
  {
    ::ros::console::print( nullptr, __rosconsole_define_location__loc.logger_,
                           static_cast<::ros::console::Level>(level),
                           file.toStdString().c_str(), line, function.toStdString().c_str(),
                           "%s", msg.toStdString().c_str());
  }
}

QObject *RosQmlSingletonWrapper::advertise( const QString &type, const QString &topic, quint32 queue_size, bool latch )
{
  return advertise( QString(), type, topic, queue_size, latch );
}

QObject *RosQmlSingletonWrapper::advertise( const QString &ns, const QString &type, const QString &topic,
                                            quint32 queue_size, bool latch )
{
  std::string ns_std = ns.toStdString();
  auto it = node_handles_.find( ns_std );
  if ( it == node_handles_.end())
  {
    node_handles_.insert( { ns_std, new NodeHandle( ns_std ) } );
    it = node_handles_.find( ns_std );
  }
  return new Publisher( it->second, type, topic, queue_size, latch );
}

QObject *RosQmlSingletonWrapper::subscribe( const QString &topic, quint32 queue_size )
{
  return subscribe( QString(), topic, queue_size );
}

QObject *RosQmlSingletonWrapper::subscribe( const QString &ns, const QString &topic, quint32 queue_size )
{
  std::string ns_std = ns.toStdString();
  auto it = node_handles_.find( ns_std );
  if ( it == node_handles_.end())
  {
    node_handles_.insert( { ns_std, new NodeHandle( ns_std ) } );
    it = node_handles_.find( ns_std );
  }
  return new Subscriber( it->second, topic, queue_size );
}

QJSValue RosQmlSingletonWrapper::createLogFunction( ros_console_levels::RosConsoleLevel level )
{
  auto engine = qjsEngine( this );
  if ( !engine ) return {};
  // This function extracts the file, method and number which called the log function in order to accurately report it.
  QJSValue func = engine->evaluate( R"js(function (__ros_instance) {
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
    __ros_instance.logInternal()js" + QString::number( level ) + R"js(, method, file, Number(line), msg, name);
  });
})js" );
  return func.call( { engine->newQObject( this ) } );
}
} // qml_ros_plugin
