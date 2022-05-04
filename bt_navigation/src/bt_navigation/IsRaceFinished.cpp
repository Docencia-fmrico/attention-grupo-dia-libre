// Copyright 2021 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>
#include <vector>

#include "bt_navigation/IsRaceFinished.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace bt_navigation
{

using namespace std::chrono_literals;

IsRaceFinished::IsRaceFinished(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
}

void
IsRaceFinished::halt()
{
  std::cout << "Is race finished halt" << std::endl;
}

BT::NodeStatus
IsRaceFinished::tick()
{
  int ind;
  config().blackboard->get("index", ind);

  std::vector<std::string> wp_ids;
  node_->get_parameter("waypoints", wp_ids);

  std::cout << "Index:" << ind << " ,WP size: " << wp_ids.size() << std::endl;

  if (ind == wp_ids.size()) {
    config().blackboard->get("index", ind);
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace bt_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_navigation::IsRaceFinished>("IsRaceFinished");
}