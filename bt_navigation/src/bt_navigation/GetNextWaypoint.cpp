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

#include "bt_navigation/GetNextWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "/opt/ros/foxy/include/nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

namespace bt_navigation
{

using namespace std::chrono_literals;

GetNextWaypoint::GetNextWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);
  map_ocuppancy_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map_occupancy", 10, std::bind(&GetNextWaypoint::map_cb, this, std::placeholders::_1));
}

void
GetNextWaypoint::map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  return;
}

void
GetNextWaypoint::halt()
{
  std::cout << "GNW halt" << std::endl;
}

BT::NodeStatus
GetNextWaypoint::tick()
{
  int ind;
  config().blackboard->get("index", ind);

  std::vector<std::string> wp_ids;
  node_->get_parameter("waypoints", wp_ids);
  std::vector<double> waypoint;
  node_->get_parameter(wp_ids[ind], waypoint);

  geometry_msgs::msg::PoseStamped publishable_wp;
  publishable_wp.pose.position.x = waypoint[0];
  publishable_wp.pose.position.y = waypoint[1];

  std::cerr << "Next waypoint on index " << ind << " is [" << waypoint[0] << "," << waypoint[1] << "]" << std::endl;
  ind++;
  config().blackboard->set("index", ind);
  setOutput("waypoint", publishable_wp);
  
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<bt_navigation::GetNextWaypoint>("GetNextWaypoint");
}