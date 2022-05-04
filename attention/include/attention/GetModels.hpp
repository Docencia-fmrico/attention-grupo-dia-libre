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
  on_configure(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_activate(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_deactivate(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_cleanup(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_shutdown(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_error(const rclcpp_lifecycle::State & state);

  void do_work();

private:
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr sub_;
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  std::vector<std::string> model_names;
  std::vector<std::vector<float>> model_pose;

  /*
  void add_nodes_to_graph();
  void model_state_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg);
  float get_distance(std::vector<float> a, std::vector<float> b);
  */
};

}  // namespace attention

#endif  // ATTENTION__GET MODELS_HPP_