cmake_minimum_required(VERSION 3.10)
project(qml_ros_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

add_definitions(-Wall -Wextra)

#set(CMAKE_BUILD_TYPE "Debug")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp image_transport ros_babel_fish tf2_ros)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Core Multimedia Qml Quick REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qml_ros_plugin
  CATKIN_DEPENDS roscpp image_transport ros_babel_fish tf2_ros
  #  DEPENDS system_lib
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
  #  include/qml_ros_plugin/image_subscriber.h
  include/qml_ros_plugin/array.h
  include/qml_ros_plugin/babel_fish_dispenser.h
  include/qml_ros_plugin/image_buffer.h
  include/qml_ros_plugin/image_transport_subscriber.h
  include/qml_ros_plugin/message_conversions.h
  include/qml_ros_plugin/node_handle.h
  include/qml_ros_plugin/publisher.h
  include/qml_ros_plugin/qml_ros_conversion.h
  include/qml_ros_plugin/qobject_ros.h
  include/qml_ros_plugin/ros.h
  include/qml_ros_plugin/service.h
  include/qml_ros_plugin/subscriber.h
  include/qml_ros_plugin/tf_transform.h
  include/qml_ros_plugin/tf_transform_listener.h
  include/qml_ros_plugin/time.h
  #  src/image_subscriber.cpp
  src/array.cpp
  src/babel_fish_dispenser.cpp
  src/image_buffer.cpp
  src/image_transport_subscriber.cpp
  src/message_conversions.cpp
  src/node_handle.cpp
  src/publisher.cpp
  src/qml_ros_plugin.cpp
  src/qobject_ros.cpp
  src/ros.cpp
  src/service.cpp
  src/subscriber.cpp
  src/tf_transform.cpp
  src/tf_transform_listener.cpp
  src/time.cpp)

## Declare a C++ library
add_library(qml_ros_plugin ${SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(qml_ros_plugin ${catkin_LIBRARIES} Qt5::Core Qt5::Multimedia Qt5::Qml Qt5::Quick)

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_babel_fish_test_msgs REQUIRED)
  include_directories(${ros_babel_fish_test_msgs_INCLUDE_DIRS})

  add_rostest_gtest(${PROJECT_NAME}_test_communication test/test_communication.test test/communication.cpp)
  target_link_libraries(${PROJECT_NAME}_test_communication ${PROJECT_NAME})
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

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

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
