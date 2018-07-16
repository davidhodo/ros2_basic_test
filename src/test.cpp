// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MyTestNode : public rclcpp::Node
{
public:
  MyTestNode()
  : Node("my_test"), count_(0)
  {
    rmw_qos_profile_t pub_qos_profile = rmw_qos_profile_default;
    rmw_qos_profile_t sub_qos_profile = rmw_qos_profile_default;


    pub_qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    pub_qos_profile.depth = 300;
    pub_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    sub_qos_profile.depth = 300;
    // sub_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic",pub_qos_profile);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MyTestNode::timer_callback, this));
    
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", std::bind(&MyTestNode::topic_callback, this, std::placeholders::_1),sub_qos_profile);
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str())
    publisher_->publish(message);
  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str())
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTestNode>());
  rclcpp::shutdown();
  return 0;
}
