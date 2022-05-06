// Copyright 2022 Grupo Día Libre
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

#ifndef ATTENTION__GETMODELS_HPP_
#define ATTENTION__GETMODELS_HPP_

#include <memory>
#include <cmath>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "ros2_knowledge_graph/graph_utils.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"

#define RIGHT -1
#define LEFT 1

namespace attention
{

class GetModels : public rclcpp_lifecycle::LifecycleNode
{
public:
  GetModels();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State & state);

  void do_work();

private:
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  
  std::vector<std::string> model_names;
  
  std::vector<std::string> tfs_name;
  std::vector<geometry_msgs::msg::TransformStamped> tfs_to_graph;

  std::vector<std::string> nodes_in_graph;

  void model_state_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
  void add_nodes_to_graph();
  void place_tfs(const gazebo_msgs::msg::ModelStates::SharedPtr msg);

  bool tfs_placed;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

};

}  // namespace attention

#endif  // ATTENTION__GET MODELS_HPP_
