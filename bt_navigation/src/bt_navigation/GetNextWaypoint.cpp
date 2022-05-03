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
#include <memory>

#include "bt_navigation/GetNextWaypoint.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "/opt/ros/foxy/include/nav2_costmap_2d/costmap_2d.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

// som constants usefull for the costmap function
static constexpr unsigned char NO_INFORMATION = 255;
static constexpr unsigned char LETHAL_OBSTACLE = 254;
static constexpr unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
static constexpr unsigned char FREE_SPACE = 0;

// nav2_util constants
static constexpr int8_t OCC_GRID_UNKNOWN = -1;
static constexpr int8_t OCC_GRID_FREE = 0;
static constexpr int8_t OCC_GRID_OCCUPIED = 100;


namespace bt_navigation
{

using std::placeholders::_1;

using namespace std::chrono_literals;

GetNextWaypoint::GetNextWaypoint(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
  // we create a subscriber to the map topic
  map_ocuppancy_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map_occupancy", 10, std::bind(&GetNextWaypoint::map_cb, this, std::placeholders::_1));


  config().blackboard->get("node", node_);
  map_ocuppancy_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map_occupancy", 10, std::bind(&GetNextWaypoint::map_cb, this, std::placeholders::_1));
}

void
GetNextWaypoint::map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{}
void
GetNextWaypoint::Costmap2D(const nav_msgs::msg::OccupancyGrid & map)
//: default_value_(FREE_SPACE)
{
  // fill local variables
  unsigned int size_x_ = map.info.width;
  unsigned int size_y_ = map.info.height;
  double resolution_ = map.info.resolution;
  double origin_x_ = map.info.origin.position.x;
  double origin_y_ = map.info.origin.position.y;

  // create the costmap

  unsigned char * costmap_ = new unsigned char[size_x_ * size_y_];
  // fill the costmap with a data
  int8_t data;
  for (unsigned int it = 0; it < size_x_ * size_y_; it++) {
    data = map.data[it];
    if (data == OCC_GRID_UNKNOWN) {
      costmap_[it] = NO_INFORMATION;
    } else {
      // Linear conversion from OccupancyGrid data range [OCC_GRID_FREE..OCC_GRID_OCCUPIED]
      // to costmap data range [FREE_SPACE..LETHAL_OBSTACLE]
      costmap_[it] = std::round(
        static_cast<double>(data) * (LETHAL_OBSTACLE - FREE_SPACE) /
        (OCC_GRID_OCCUPIED - OCC_GRID_FREE));
    }
  }
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

  std::cerr << "Next waypoint on index " << ind << " is [" << waypoint[0] << "," << waypoint[1] <<
    "]" << std::endl;
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
