//
// Created by Stefan Fabian on 25.03.19.
//

#ifndef QML_ROS_PLUGIN_SUBSCRIBER_H
#define QML_ROS_PLUGIN_SUBSCRIBER_H

#include <QObject>
#include <QVariant>
#include <QMap>

#include <ros_type_introspection/ros_introspection.hpp>

#include <topic_tools/shape_shifter.h>

#include <ros/ros.h>

namespace RosIntrospection
{
class Parser;
}

namespace qml_ros_plugin
{

class Subscriber : public QObject
{
Q_OBJECT
  Q_PROPERTY( QString topic
                READ topic
                WRITE setTopic
                NOTIFY
                topicChanged )
  Q_PROPERTY( bool running
                READ running
                WRITE setRunning
                NOTIFY
                runningChanged )
  Q_PROPERTY( QVariant message
                READ message
                NOTIFY
                messageChanged )
public:
  Subscriber();

  ~Subscriber() override;

  QString topic();

  void setTopic( const QString &value );

  bool running();

  void setRunning( bool value );

  QVariant message();

  Q_INVOKABLE unsigned int getNumPublishers();

signals:

  void topicChanged();

  void runningChanged();

  void messageChanged();

  void newMessage( const QVariant &message );

protected:
  void subscribe();

  void shutdown();

  void messageCallback( const topic_tools::ShapeShifter::ConstPtr &msg );

  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  RosIntrospection::Parser parser_;
  std::vector<uint8_t> buffer_;
  RosIntrospection::FlatMessage flat_message_;
  bool running_;
  bool is_subscribed_;

  QString topic_;
  QVariant message_;
};
} // qml_ros_plugin

#endif //QML_ROS_PLUGIN_SUBSCRIBER_H
