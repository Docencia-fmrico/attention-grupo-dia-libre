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

#ifndef ATTENTION__ATTENTION_HPP_
#define ATTENTION__ATTENTION_HPP_

#include <memory>
#include <cmath>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

namespace attention
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Observator : public rclcpp_lifecycle::LifecycleNode
{
public:
  Observator();

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void do_work();

protected:


private:
  std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
};
}  // namespace follow_wall

#endif  // FOLLOW_WALL__FOLLOW_WALL_HPP_
