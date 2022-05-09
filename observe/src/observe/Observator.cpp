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

#include "lifecycle_msgs/msg/state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "observe/Observator.hpp"
#include "ros2_knowledge_graph/graph_utils.hpp"


#include <iostream>
#include <cstdlib>
#include <unistd.h>

#include <chrono>
#include <thread>


using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace observe

{
  Observator::Observator()
  : LifecycleNode("observator_node")
  {
  }

  CallbackReturnT Observator::on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 10);
    sub_ = create_subscription<ros2_knowledge_graph_msgs::msg::GraphUpdate>(
    "graph_update", 10, std::bind(&Observator::graph_cb, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>(shared_from_this());
    ts0_ = time(0);
    pub_->on_activate();
    indx = 0;

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT Observator::on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    pub_->on_deactivate();
    
    return CallbackReturnT::SUCCESS;
  }
  
  void Observator::do_work() 
  {
    auto edge_list = graph_->get_edges();
    if (edge_list.size() == 0){
      return;
    }

    if (time(0) - ts0_  > 2) {
      ts0_ = time(0);
      indx++;
      if (indx >= edge_list.size()) {
        indx = 0;
      }
    }

    ros2_knowledge_graph_msgs::msg::Edge edge_map = edge_list[indx];
    auto tf_edge = graph_->get_edges<std::string>(edge_map.source_node_id, edge_map.target_node_id);
    auto content_tf_opt = ros2_knowledge_graph::get_content<std::string>(tf_edge[0].content);
    std::string tf_name = content_tf_opt.value();

    if (tf_name != "null")
    {
      watch_object(edge_map.source_node_id);
    } else {
      indx++;
      if (indx >= edge_list.size()) {
        indx = 0;
      }
    }
  }

  void Observator::watch_object(std::string item_name)
  {

    geometry_msgs::msg::TransformStamped tf_to_check;
    std::string base_footprint = "base_footprint";
    std::cout << "Looking at " << item_name << std::endl;

    try { tf_to_check = tf_buffer_->lookupTransform(
            item_name, base_footprint,
            tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", item_name, base_footprint.c_str(), ex.what());
      return;
    }

    float y = tf_to_check.transform.translation.y;
    float x = tf_to_check.transform.translation.x;
    float z = tf_to_check.transform.translation.z;

    float horizontal_angle = atan2(y,x);

    
    if (fabs(horizontal_angle) > 1.57)
    {
      std::cout << item_name << " unreachable" << std::endl;
      return;
    }

    trajectory_msgs::msg::JointTrajectory message;

    message.header.stamp = now();

    message.joint_names.push_back("head_1_joint");
    message.joint_names.push_back("head_2_joint");

    message.points.resize(1);
    message.points[0].positions.resize(2);
    message.points[0].velocities.resize(2);
    message.points[0].accelerations.resize(2);

    message.points[0].positions[0] = horizontal_angle;
    message.points[0].positions[1] = 0; //vertical_angle;

    message.points[0].velocities[0] = 0.2;
    message.points[0].velocities[1] = 0.2;

    message.points[0].accelerations[0] = 0.2;
    message.points[0].accelerations[1] = 0.2;

    message.points[0].time_from_start = rclcpp::Duration(1s);

    pub_->publish(message);

    return;

  }

  void
  Observator::graph_cb(const ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg)
  {
    return;
  }


};

