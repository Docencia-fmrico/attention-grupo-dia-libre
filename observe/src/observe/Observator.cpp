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
    auto edge_tf = graph_->get_edges();
    std::cout << edge_tf.size() << std::endl;
    /*
    if (graph_->get_nodes().size())
    {   
      for (int i = 0; i < graph_->get_nodes().size(); i++)
      {
        std::string name = graph_->get_node_names()[i];
        std::cout << name << std::endl;
        if (name != "World")
        {
          auto edge_tf = graph_->get_edges();
          std::cout << edge_tf.size() << std::endl;
          /*
          geometry_msgs::msg::TransformStamped tf = edge_tf[0].content.tf_value;
          float x = tf.transform.translation.x;
          float y = tf.transform.translation.y;
          float z = tf.transform.translation.z;
          std::cout << x << " " << y << " " << z << std::endl;
          if ((x != 0) && (y != 0) && (z != 0))
          {
            //watch_object(name);
          }
        }
      }  
    }
    //watch_object("cabinet");
    */
  }
  void Observator::watch_object(std::string tf)
  {

    geometry_msgs::msg::TransformStamped tf_to_check;
    std::string base_footprint = "base_footprint";
    int max_iterations = 20;

    for (int i = 0; i < max_iterations; i++)
    {
      try { tf_to_check = tf_buffer_->lookupTransform(
              tf, base_footprint,
              tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s", tf.c_str(), base_footprint.c_str(), ex.what());
        return;
      }

      float y = tf_to_check.transform.translation.y;
      float x = tf_to_check.transform.translation.x;
      float z = tf_to_check.transform.translation.z;

      float horizontal_angle = atan2(-y,-x);
      
      /*
      float vertical_angle = atan2(z,x);
      float vertical_angle2 = atan2(-z,x);
      float vertical_angle3 = atan2(z,-x);
      float vertical_angle4 = atan2(-z,-x);
      //float horizontal_angle = atan2(y,x);
      //float horizontal_angle1 = atan2(-y,x);
      //float horizontal_angle2 = atan2(y,-x);
      std::cout << vertical_angle << std::endl;
      std::cout << vertical_angle2 << std::endl;
      std::cout << vertical_angle3 << std::endl;
      std::cout << vertical_angle4 << std::endl;*/
      
      if (abs(horizontal_angle) > 1.57)
      {
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

    }

    return;

  }

  void
  Observator::graph_cb(const ros2_knowledge_graph_msgs::msg::GraphUpdate::SharedPtr msg)
  {
    std::cout << msg->graph.edges.size() << std::endl;
    return;
  }


};

