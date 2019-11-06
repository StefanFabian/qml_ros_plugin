// Copyright (c) 2019 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml_ros_plugin/babel_fish_dispenser.h"

namespace qml_ros_plugin
{

BabelFishDispenser::BabelFishDispenser() = default;

ros_babel_fish::BabelFish BabelFishDispenser::getBabelFish()
{
  static BabelFishDispenser dispenser;
  return dispenser.createBabelFish();
}

ros_babel_fish::BabelFish BabelFishDispenser::createBabelFish()
{
  if ( description_provider_ != nullptr ) return ros_babel_fish::BabelFish( description_provider_ );
  ros_babel_fish::BabelFish babel_fish;
  description_provider_ = babel_fish.descriptionProvider();
  return babel_fish;
}
} // qml_ros_plugin
