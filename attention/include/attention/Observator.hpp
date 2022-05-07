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

#ifndef ATTENTION__OBSERVATOR_HPP_
#define ATTENTION__OBSERVATOR_HPP_

#include <memory>
#include <cmath>
#include "trajectory_msgs/msg/joint_trajectory.hpp"

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

namespace attention
{

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class Observator : public rclcpp_lifecycle::LifecycleNode
{
public:
  Observator();

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  void do_work();

protected:


private:
  //std::shared_ptr<ros2_knowledge_graph::GraphNode> graph_;
  std::vector<geometry_msgs::msg::TransformStamped> tfs_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  void watch_object(std::string tf);
};
}  // namespace follow_wall

#endif  // FOLLOW_WALL__FOLLOW_WALL_HPP_
