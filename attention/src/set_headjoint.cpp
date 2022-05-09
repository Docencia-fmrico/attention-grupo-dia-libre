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
#include "trajectory_msgs/msg/joint_trajectory.hpp"


using namespace std::chrono_literals;

class MyNodePublisher : public rclcpp::Node
{
public:
  MyNodePublisher()
  : Node("head_pub")
  {
    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>("/head_controller/joint_trajectory", 10);
    //std::cerr << "1" << std::endl;
    names.push_back("head_1_joint");
    names.push_back("head_2_joint");
  }

  void doWork()
  {
    
    
    //std::cerr << "2" << std::endl;
    trajectory_msgs::msg::JointTrajectory message;
    //std::cerr << "2.1" << std::endl;
    message.header.stamp = now();
    //std::cerr << "2.1.1" << std::endl;

    message.joint_names.push_back("head_1_joint");
    message.joint_names.push_back("head_2_joint");
    //std::cerr << "2.1.2" << std::endl;
    message.points.resize(1);
    message.points[0].positions.resize(2);
    message.points[0].velocities.resize(2);
    message.points[0].accelerations.resize(2);
    //std::cerr << "2.2" << std::endl;
    message.points[0].positions[0] = 0;
    message.points[0].positions[1] = 0;
    message.points[0].velocities[0] = 0.2;
    message.points[0].velocities[1] = 0.2;
    message.points[0].accelerations[0] = 0.2;
    message.points[0].accelerations[1] = 0.2;
    message.points[0].time_from_start = rclcpp::Duration(1s);
    //std::cerr << "2.2" << std::endl;

    pub_->publish(message);
    //std::cerr << "3" << std::endl;
  }

private:
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  std::vector<std::string> names;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MyNodePublisher>();

  rclcpp::Rate loop_rate(500ms);
  while (rclcpp::ok()) {
    //std::cerr << "0" << std::endl;
    node->doWork();

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}