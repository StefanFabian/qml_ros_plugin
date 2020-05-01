//
// Created by Stefan Fabian on 04.03.20.
//

#include "common.h"
#include "message_comparison.h"
//#ifndef _WIN32
//#include <unistd.h>
//#endif

#include <qml_ros_plugin/ros.h>

#include <ros/ros.h>
#include <ros/package.h>

#ifdef _WIN32
int setenv(const char *name, const char *value, int overwrite)
{
  if(!overwrite)
  {
    size_t envsize = 0;
    errno_t errcode = getenv_s(&envsize, NULL, 0, name);
    if(errcode || envsize)
      return errcode;
  }
  return _putenv_s(name, value);
}
#endif

template<typename ContainerA, typename KeyType>
::testing::AssertionResult listEquals( const ContainerA &container, const std::vector<KeyType> &values )
{
  std::map<KeyType, bool> found;
  for ( const auto &key : values )
  {
    found.insert( { key, false } );
  }
  for ( const auto &value : container )
  {
    if ( found.find( value ) == found.end())
      return ::testing::AssertionFailure() << value << " is in container but was not expected!";
    found[value] = true;
  }
  for ( const auto &pair : found )
  {
    if ( !pair.second )
      return ::testing::AssertionFailure() << pair.first << " was not found in container.";
  }
  return ::testing::AssertionSuccess();
}

using namespace qml_ros_plugin;

TEST( Package, package )
{
  RosQmlSingletonWrapper wrapper;
  Package package = wrapper.package();
  QString path = package.getPath( ROS_PACKAGE_NAME );
  ASSERT_FALSE( path.isEmpty());
  EXPECT_EQ( path.toStdString(), ros::package::getPath( ROS_PACKAGE_NAME ));

  char *oldrpp = getenv( "ROS_PACKAGE_PATH" );
  std::string package_path = path.toStdString() + "/test/test_packages";
  setenv( "ROS_PACKAGE_PATH", package_path.c_str(), 1 );
  path = package.getPath( ROS_PACKAGE_NAME );
  EXPECT_TRUE( path.isEmpty());

  EXPECT_TRUE( listEquals( package.getAll(),
                           std::vector<QString>{ "poor_people", "rich_people", "rick_astley", "world" } ));

  QVariantMap plugins = package.getPlugins( "world", "value" );
  std::map<QString, std::vector<QString>> expected_plugins = {{ "rich_people", { "Nothing but despair" }},
                                                              { "rick_astley", { "Never Gonna Give You Up" }}};
  for ( const auto &key : plugins.keys())
  {
    ASSERT_NE( expected_plugins.find( key ), expected_plugins.end()) << key;
    EXPECT_TRUE( listEquals( plugins[key].toStringList(), expected_plugins[key] )) << key;
  }

  plugins = package.getPlugins( "world", "wars" );
  expected_plugins = {{ "rich_people", { "true" }}};

  for ( const auto &key : plugins.keys())
  {
    ASSERT_NE( expected_plugins.find( key ), expected_plugins.end()) << key;
    EXPECT_TRUE( listEquals( plugins[key].toStringList(), expected_plugins[key] )) << key;
  }

  QString output = package.command( "depends-on poor_people" );
  EXPECT_EQ( output.trimmed(), QString( "rich_people" ));

  setenv( "ROS_PACKAGE_PATH", (package_path + "/not_existing_subfolder").c_str(), 1 );
  ASSERT_TRUE( package.getAll().empty());

  setenv( "ROS_PACKAGE_PATH", oldrpp, 1 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  ros::init( argc, argv, "test_logging" );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}
