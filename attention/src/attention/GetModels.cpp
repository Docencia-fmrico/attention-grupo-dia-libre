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
  RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state. Creating subscriber", get_name(), state.label().c_str());
    
  tfs_placed = false;
  sub_ = create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 10, std::bind(&GetModels::model_state_cb, this, _1));

  broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(shared_from_this());
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());

  auto node_1 = ros2_knowledge_graph::new_node("World", "object");
  graph_->update_node(node_1);
  
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT 
GetModels::on_deactivate(const rclcpp_lifecycle::State & state) 
{
  RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
  sub_ = nullptr;
  return CallbackReturnT::SUCCESS;
}

void 
GetModels::do_work() 
{
  if (tfs_name.size() > 0)
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

void 
GetModels::add_nodes_to_graph()
{
  for (int i = 0; i < tfs_name.size(); ++i)
  {
    bool node_in_list = false;
    std::string node_name;
    auto node_1 = ros2_knowledge_graph::new_node(tfs_name[i], "object");
    graph_->update_node(node_1);

    auto edge_1 = ros2_knowledge_graph::new_edge("World", tfs_name[i], tfs_to_graph[i]);
    graph_->update_edge(edge_1);


    for (int j = 0; j < nodes_in_graph.size(); j++) {
      if (nodes_in_graph[j] ==  tfs_name[i]) {
        node_in_list = true;
        break;
      }
    }

    if (!node_in_list) {
      nodes_in_graph.push_back(tfs_name[i]);
    } 
    
    for (int j = 0; j < nodes_in_graph.size(); j++) {
      bool tf_in_range = false;
      for (int k = 0; k < tfs_name.size(); k++){
        if (nodes_in_graph[j] == tfs_name[k]) {
          tf_in_range = true;
          break;
        }
      }
      if (!tf_in_range) {
        std::cout << "NODE " << nodes_in_graph[i] << " NOT IN RANGE. DELETING" << std::endl; 
        graph_->remove_node(nodes_in_graph[i]);
        graph_->remove_edge(edge_1);
        nodes_in_graph.erase(nodes_in_graph.begin() + i);
      }
    }
  }
}

void
GetModels::place_tfs(const gazebo_msgs::msg::ModelStates::SharedPtr msg) {
  
  for (int i = 0; i < msg->name.size(); i++) {
    if ((msg->name[i] != "tiago") && (msg->name[i] != "ground_plane")) {
      geometry_msgs::msg::TransformStamped tf_to_create;
      
      tf_to_create.header.stamp = now();
      tf_to_create.header.frame_id = "odom";
      tf_to_create.child_frame_id = msg->name[i];

      tf_to_create.transform.translation.x = msg->pose[i].position.x;
      tf_to_create.transform.translation.y = msg->pose[i].position.y;
      tf_to_create.transform.translation.z = 0;

      tf2::Quaternion q;

      q.setRPY(0, 0, 0);

      tf_to_create.transform.rotation.x = q.x();
      tf_to_create.transform.rotation.y = q.y();
      tf_to_create.transform.rotation.z = q.z();
      tf_to_create.transform.rotation.w = q.w();

      broadcaster_->sendTransform(tf_to_create);
      model_names.push_back(msg->name[i]);
      std::cerr << msg->name[i] << std::endl;
    }
  }
}

void
GetModels::model_state_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
{
  
  if (!tfs_placed) {
    place_tfs(msg);
    tfs_placed = true;
  }

  std::string transform_base_to = "odom";
  std::string tiago_tf_from = "base_footprint";

  geometry_msgs::msg::TransformStamped tiago_tf;

  try { tiago_tf = tf_buffer_->lookupTransform(
          transform_base_to, tiago_tf_from,
          tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", transform_base_to.c_str(), tiago_tf_from.c_str(), ex.what());
    return;
  }

  std::vector<float> robot_position;
  robot_position.push_back(tiago_tf.transform.translation.x);
  robot_position.push_back(tiago_tf.transform.translation.y);

  tfs_name.clear();
  tfs_to_graph.clear();

  for (int i = 0; i < model_names.size(); i++) {

    std::string object_transform_from = model_names[i];
    geometry_msgs::msg::TransformStamped tf_to_check;

    try { tf_to_check = tf_buffer_->lookupTransform(
            transform_base_to, object_transform_from,
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", transform_base_to.c_str(), object_transform_from.c_str(), ex.what());
      return;
    }
    
    std::vector<float> object_position;
    object_position.push_back(tf_to_check.transform.translation.x);
    object_position.push_back(tf_to_check.transform.translation.y);

    float distance_between_tfs = get_distance(robot_position, object_position);

    if (distance_between_tfs < 5) {
      tfs_name.push_back(model_names[i]);
      tfs_to_graph.push_back(tf_to_check);
    }
    
  }
  std::cout << "------------------" << std::endl;

  for (int i = 0; i < tfs_name.size(); i++) {
    std::cout << "TF " << tfs_name[i] << " in position (" << tfs_to_graph[i].transform.translation.x  << "," << tfs_to_graph[i].transform.translation.y << ") respect robot is in range" << std::endl;
  }

  std::cout << "------------------" << std::endl;
  
}

}

