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
#include <ctime>

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace std::chrono_literals;

namespace observe

{
  Observator::Observator()
  : LifecycleNode("observator_node")
  {
   // declare_parameter("speed", 0.34);
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

    pub_->on_activate();

    
    
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
    if (edge_list.size() > 5)
    {
      for (int i = 0; i < edge_list.size(); i++)
      {
        ros2_knowledge_graph_msgs::msg::Edge edge_map = edge_list[i];
        //std::cout << "Origen: " << edge_map.source_node_id << " Destino: " << edge_map.target_node_id << std::endl;
        auto tf_edge = graph_->get_edges<geometry_msgs::msg::TransformStamped>(edge_map.source_node_id, edge_map.target_node_id);
        auto content_tf_opt = ros2_knowledge_graph::get_content<geometry_msgs::msg::TransformStamped>(tf_edge[0].content);
        geometry_msgs::msg::TransformStamped value_tf = content_tf_opt.value();
        float x = value_tf.transform.translation.x;
        float y = value_tf.transform.translation.y;
        //std::cout << edge_map.source_node_id << ": " << x << " " << y << " " << z << std::endl;
        if ((x != 0) && (y != 0))
        {
          watch_object(edge_map.source_node_id);
        }
      }
    }
    //std::cerr << "PACO TONTO" << std::endl;
  }
  void Observator::watch_object(std::string tf)
  {

    geometry_msgs::msg::TransformStamped tf_to_check;
    std::string base_footprint = "base_footprint";
    std::cout << "---------------------" << tf << "----------------------" << std::endl;
    std::cerr << "1" << std::endl;
    time_t ts0 = time(0);


    std::cerr << "2" << std::endl;

    while (( time(0) - ts0 ) < 4)
    {
      std::cerr << tf << " 3" << std::endl;
      //std::cout << time(0) - ts0 << std::endl;
      try { tf_to_check = tf_buffer_->lookupTransform(
              tf, base_footprint,
              tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", tf.c_str(), base_footprint.c_str(), ex.what());
        return;
      }
      std::cerr << tf << " 4" << std::endl;

      float y = tf_to_check.transform.translation.y;
      float x = tf_to_check.transform.translation.x;
      float z = tf_to_check.transform.translation.z;
      

      float horizontal_angle = atan2(-y,-x);
      //std::cout << horizontal_angle << std::endl;

      //std::cout << tf << std::endl;
      
      if (abs(horizontal_angle) > 1.57)
      {
        return;
      }
      std::cerr << tf << " 5" << std::endl;
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
    }


    return;

  }

  void
  Observator::graph_cb(const ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg)
  {
    return;
  }


};

