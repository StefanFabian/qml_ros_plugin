cmake_minimum_required(VERSION 3.10)
project(qml_ros_plugin)
set(CMAKE_CXX_STANDARD 14)

add_definitions(-Wall -Wextra)

#set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED actionlib roscpp image_transport ros_babel_fish tf2_ros)

find_package(Qt5 COMPONENTS Core Multimedia Qml Quick REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP yaml-cpp)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES qml_ros_plugin
  CATKIN_DEPENDS actionlib roscpp image_transport ros_babel_fish tf2_ros
  DEPENDS Qt5Core Qt5Multimedia Qt5Qml YAML_CPP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Multimedia_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
  include/qml_ros_plugin/array.h
  include/qml_ros_plugin/action_client.h
  include/qml_ros_plugin/babel_fish_dispenser.h
  include/qml_ros_plugin/console.h
  include/qml_ros_plugin/goal_handle.h
  include/qml_ros_plugin/image_buffer.h
  include/qml_ros_plugin/image_transport_manager.h
  include/qml_ros_plugin/image_transport_subscriber.h
  include/qml_ros_plugin/io.h
  include/qml_ros_plugin/message_conversions.h
  include/qml_ros_plugin/node_handle.h
  include/qml_ros_plugin/package.h
  include/qml_ros_plugin/publisher.h
  include/qml_ros_plugin/qml_ros_conversion.h
  include/qml_ros_plugin/qobject_ros.h
  include/qml_ros_plugin/ros.h
  include/qml_ros_plugin/service.h
  include/qml_ros_plugin/subscriber.h
  include/qml_ros_plugin/tf_transform.h
  include/qml_ros_plugin/tf_transform_listener.h
  include/qml_ros_plugin/topic_info.h
  include/qml_ros_plugin/time.h
  src/array.cpp
  src/action_client.cpp
  src/babel_fish_dispenser.cpp
  src/console.cpp
  src/goal_handle.cpp
  src/image_buffer.cpp
  src/image_transport_manager.cpp
  src/image_transport_subscriber.cpp
  src/io.cpp
  src/message_conversions.cpp
  src/node_handle.cpp
  src/package.cpp
  src/publisher.cpp
  src/qml_ros_plugin.cpp
  src/qobject_ros.cpp
  src/ros.cpp
  src/service.cpp
  src/subscriber.cpp
  src/tf_transform.cpp
  src/tf_transform_listener.cpp
  src/time.cpp)

add_library(qml_ros_plugin ${SOURCES})
target_link_libraries(qml_ros_plugin ${catkin_LIBRARIES} Qt5::Core Qt5::Multimedia Qt5::Qml Qt5::Quick ${YAML_CPP_LIBRARIES})

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(Qt5Test REQUIRED)
  include_directories(${ros_babel_fish_test_msgs_INCLUDE_DIRS} ${std_srvs_INCLUDE_DIRS} ${Qt5Test_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME}_test_action_server test/action_server.cpp)
  target_link_libraries(${PROJECT_NAME}_test_action_server ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_action_server PROPERTIES OUTPUT_NAME test_action_server PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_communication test/test_communication.test test/communication.cpp)
  target_link_libraries(${PROJECT_NAME}_test_communication ${PROJECT_NAME} Qt5::Test)
  set_target_properties(${PROJECT_NAME}_test_communication PROPERTIES OUTPUT_NAME test_communication PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_image_conversions test/test_image_conversions.test test/image_conversions.cpp)
  target_link_libraries(${PROJECT_NAME}_test_image_conversions ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_image_conversions PROPERTIES OUTPUT_NAME test_image_conversions PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_image_transport_subscriber test/test_image_transport_subscriber.test test/image_transport_subscriber.cpp)
  target_link_libraries(${PROJECT_NAME}_test_image_transport_subscriber ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_image_transport_subscriber PROPERTIES OUTPUT_NAME test_image_transport_subscriber PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_message_conversions test/test_message_conversions.test test/message_conversions.cpp)
  target_link_libraries(${PROJECT_NAME}_test_message_conversions ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_message_conversions PROPERTIES OUTPUT_NAME test_message_conversions PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_ros_life_cycle test/test_ros_life_cycle.test test/ros_life_cycle.cpp)
  target_link_libraries(${PROJECT_NAME}_test_ros_life_cycle ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_ros_life_cycle PROPERTIES OUTPUT_NAME test_ros_life_cycle PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_spinning test/test_spinning.test test/spinning.cpp)
  target_link_libraries(${PROJECT_NAME}_test_spinning ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_spinning PROPERTIES OUTPUT_NAME test_spinning PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_logging test/test_logging.test test/logging.cpp)
  target_link_libraries(${PROJECT_NAME}_test_logging ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_logging PROPERTIES OUTPUT_NAME test_logging PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_package test/test_package.test test/package.cpp)
  target_link_libraries(${PROJECT_NAME}_test_package ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_package PROPERTIES OUTPUT_NAME test_package PREFIX "")

  add_rostest_gtest(${PROJECT_NAME}_test_io test/test_io.test test/io.cpp)
  target_link_libraries(${PROJECT_NAME}_test_io ${PROJECT_NAME})
  set_target_properties(${PROJECT_NAME}_test_io PROPERTIES OUTPUT_NAME test_io PREFIX "")
endif ()

# to run: catkin build --this --no-deps -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -v --catkin-make-args qml_ros_plugin_coverage
# Path to results overview will be printed in the build process
# Big thanks to the moveit people from whose docs I've obtained the information on how to get the coverage
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)   # catkin package ros-*-code-coverage
  include(CodeCoverage)
  APPEND_COVERAGE_COMPILER_FLAGS()
  set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*" "*/src/qml_ros_plugin.cpp")
  add_code_coverage(NAME ${PROJECT_NAME}_coverage)
endif ()

#############
## Install ##
#############

# Install Qml plugin as found here https://github.com/4rtzel/cmake-qml-plugin-example/issues/1
set(URI Ros)
string(REPLACE "." "/" TARGETPATH ${URI})
execute_process(COMMAND qmake -qt5 -query QT_INSTALL_QML OUTPUT_VARIABLE QT_INSTALL_QML_RAW)
string(REPLACE "\n" "" QT_INSTALL_QML ${QT_INSTALL_QML_RAW})
if ("${QT_INSTALL_QML}" STREQUAL "**Unknown**")
  message(FATAL_ERROR "Could not find qml plugin dir. Is qml installed?")
endif ()
message(STATUS "Plugin will be installed to ${QT_INSTALL_QML}")
set(DESTDIR "${QT_INSTALL_QML}/${TARGETPATH}")
install(TARGETS qml_ros_plugin DESTINATION ${DESTDIR})
install(FILES ${CMAKE_CURRENT_LIST_DIR}/qmldir DESTINATION ${DESTDIR})
