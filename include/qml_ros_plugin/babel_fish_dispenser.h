// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS_PLUGIN_BABEL_FISH_DISPENSER_H
#define QML_ROS_PLUGIN_BABEL_FISH_DISPENSER_H

#include <ros_babel_fish/babel_fish.h>

namespace qml_ros_plugin
{

/*!
 * @brief Can be used to obtain a BabelFish that is linked to a description provider that is used for all BabelFish.
 *
 * The ros_babel_fish::DescriptionProvider is responsible for looking up message definitions and caching them, hence,
 * sharing an instance significantly improves performance.
 */
class BabelFishDispenser
{
  BabelFishDispenser();

public:
  BabelFishDispenser( const BabelFishDispenser & ) = delete;

  void operator=( const BabelFishDispenser & ) = delete;

  //! @return A BabelFish instance using the shared description provider.
  static ros_babel_fish::BabelFish getBabelFish();

private:
  ros_babel_fish::BabelFish createBabelFish();

  ros_babel_fish::DescriptionProvider::Ptr description_provider_;
};
} // namespace qml_ros_plugin

#endif // QML_ROS_PLUGIN_BABEL_FISH_DISPENSER_H
