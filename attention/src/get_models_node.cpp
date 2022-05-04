// Copyright 2020 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

rclcpp::Node::SharedPtr node = nullptr;

void callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  std::vector<int> models;
  for (int i = 0; i < msg->name.size(); ++i)
  {
    std::cout << "Model: " << msg->name[i] << std::endl;
    std::cout << msg->pose[i].position.x << " " << msg->pose[i].position.y << std::endl;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("simple_node_sub");
  auto subscription = node->create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 10, callback);
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}