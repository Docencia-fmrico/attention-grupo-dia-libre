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

#include <memory>
#include <math.h>

#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "attention/GetModels.hpp"
#include "ros2_knowledge_graph/GraphNode.hpp"

namespace attention {

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

GetModels::GetModels()
: LifecycleNode("get_models_node")
{
}

CallbackReturnT 
GetModels::on_activate(const rclcpp_lifecycle::State & state) 
{
  RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
  
  //speed_ = get_parameter("speed").get_value<double>();
  
  sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
    "/scan_filtered", 10, std::bind(&GetModels::model_state_cb, this, _1));

  //publicar robot
  
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
GetModels::on_deactivate(const rclcpp_lifecycle::State & state) 
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
  sub_ = nullptr;
  graph_.cleanUp();
  return CallbackReturnT::SUCCESS;
}



void 
GetModels::do_work() 
{
  if (model_names.size() > 0)
  {
      GetModels::add_nodes_to_graph();
  }
}

float 
get_distance(std::vector<float> a, std::vector<float> b)
{
  float dif_x = (a[0]-b[0])*(a[0]-b[0]);
  float dif_y = (a[1]-b[1])*(a[1]-b[1]);
  return abs(sqrt(dif_x + dif_y));
}

std::vector <float>
GetModels::obtain_robot_pos() {
  std::vector <float> empty_vect;
  for (int i = 0; i < model_names.size(); i++) {
    if (model_names[i] == "Tiago") {
      return model_pose[i];
    }
  }
  return empty_vect;
}

void 
GetModels::add_nodes_to_graph()
{
  for (int i = 0; i < model_names.size(); ++i)
  {
      ros2_knowledge_graph_msgs::msg::Node obj;
      ros2_knowledge_graph_msgs::msg::Edge edge;
      ros2_knowledge_graph_msgs::msg::Content position;
      obj.node_name = model_names[i];

      ros2_knowledge_graph::GraphNode * node_to_add = graph_.getInstance(shared_from_this());

      std::vector <float> robot_position = GetModels::obtain_robot_pos();
      if (abs(get_distance(robot_position,model_pose[i])) > 5) //meter las coords del robot aqui
      {
          //graph_->update_edges(robot_position, obj.node_name, "wont see")
      }
      //graph_->update_edges(robot_position, obj.node_name, model_pose[i]);

  }
}

void
GetModels::model_state_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  bool not_there = true;
  std::vector<float> robot_position;
  for (int i = 0; i < msg->name.size(); ++i)
  {
    if (msg->name[i] == "Tiago") {
      robot_position.push_back(msg->pose[i].position.x);
      robot_position.push_back(msg->pose[i].position.y);
      break;
    }
  }

  if (robot_position.size() == 0) {
    std::cerr << "ERROR: did not find tiago robot" << std::endl;
    return;
  }

  for (int i = 0; i < msg->name.size(); i++) {
    for (int j = 0; j <  model_names.size(); j++) {
      if (msg->name[i] == model_names[j]) {
        std::vector<float> model_pos;
        model_pos.push_back(msg->pose[i].position.x);
        model_pos.push_back(msg->pose[i].position.y);
        if (get_distance(model_pos, robot_position) > 5) {
          model_names.erase(model_names.begin() + j);
          model_x_vector.erase(model_x_vector.begin() + j);
          model_y_vector.erase(model_y_vector.begin() + j);
        }
        break;
      }
      model_names.push_back(msg->name[i]);
      model_x_vector.push_back(msg->pose[i].position.x);
      model_y_vector.push_back(msg->pose[i].position.y);
    }
  }

}

}

