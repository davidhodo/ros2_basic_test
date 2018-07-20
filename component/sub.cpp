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

#include "sub.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

MyTestSub::MyTestSub()
: Node("sub"), count_(0)
{
  rmw_qos_profile_t sub_qos_profile = rmw_qos_profile_default;

  sub_qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  sub_qos_profile.depth = 10;
  sub_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

  subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image", std::bind(&MyTestSub::topic_callback, this, std::placeholders::_1),sub_qos_profile);
}

void MyTestSub::topic_callback(const sensor_msgs::msg::Image::UniquePtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "I heard:")
}
