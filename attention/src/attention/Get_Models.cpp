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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;
using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GetModels : public rclcpp_lifecycle::LifecycleNode
{
public:
  GetModels::GetModels()
  : rclcpp_lifecycle::LifecycleNode("get_models_node")
  {
   // declare_parameter("speed", 0.34);
  }

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(get_logger(), "[%s] Configuring from [%s] state...", get_name(), state.label().c_str());
   
    //speed_ = get_parameter("speed").get_value<double>();
    sub_= node->create_subscription<gazebo_msgs::msg::ModelStates>(
    "/gazebo/model_states", 10, model_state_cb);
    graph_ = std::make_shared<ros2_knowledge_graph::GraphNode>("get_models");
    graph_->start();
    ros2_knowledge_graph_msgs::msg::Node robot;
    robot.node_name = "tiago";
    graph_->update_node(robot);
    //publicar robot
   
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Activating from [%s] state...", get_name(), state.label().c_str());
    
    sub_->on_activate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Deactivating from [%s] state...", get_name(), state.label().c_str());
    
    sub_->on_deactivate();
    
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Cleanning Up from [%s] state...", get_name(), state.label().c_str());
    
    sub_.reset();

    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    
    sub_.reset();
    
    return CallbackReturnT::SUCCESS;
  }

    CallbackReturnT on_error(const rclcpp_lifecycle::State & state) 
  {
    RCLCPP_INFO(get_logger(), "[%s] Shutting Down from [%s] state...", get_name(), state.label().c_str());
    return CallbackReturnT::SUCCESS;
  }
  
  void do_work() 
  {
    if (model_names.size() > 0)
    {
        add_nodes_to_graph();
    }
  }

  float get_distance(std::vector<float> a, std::vector<float> b)
  {
    float dif_x = (a[0]-b[0])*(a[0]-b[0]);
    float dif_y = (a[1]-b[1])*(a[1]-b[1]);
    float dif_z = (a[2]-b[2])*(a[2]-b[2]);
    return sqrt(dif_x+dif_y+dif_z);
  }

  void add_nodes_to_graph()
  {
    for (int j = 0; j < model_names.size(); ++j)
    {
        ros2_knowledge_graph_msgs::msg::Node obj;
        ros2_knowledge_graph_msgs::msg::Edge edge;
        ros2_knowledge_graph_msgs::msg::Content position;
        obj.node_name = msg->name[i];
        graph_->update_node(obj);
        if (get_distance(x,model_pose[i]); > 5) //meter las coords del robot aqui
        {
           graph_->update_edges(x, msg->name[i], "wont see")
        }
        graph_->update_edges(x, msg->name[i], model_pose[i]);

    }
  }

  void model_state_cb(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    bool not_there = true;
    for (int i = 0; i < msg->name.size(); ++i)
    {
        if (model_names.size() > 0)
        {
            for (int j = 0; j < model_names.size(); ++j)
            {
                if (model_names[j] == msg->name[i]) not_there=false;
            }
            if (not_there)
            {
                model_names.push_back(msg->name[i]);
                model_pose[i].push_back(msg->pose[i].position.z);
                model_pose[i].push_back(msg->pose[i].position.y);
                model_pose[i].push_back(msg->pose[i].position.x);
            }
            not_there = true;
        }
        else
        {
            model_names.push_back(msg->name[i]);
            model_pose[i].push_back(msg->pose[i].position.z);
            model_pose[i].push_back(msg->pose[i].position.y);
            model_pose[i].push_back(msg->pose[i].position.x);
        }
        
    }
  }


};

