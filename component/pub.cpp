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
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "pub.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

MyTestPub::MyTestPub()
: Node("pub"), count_(0)
{
  rmw_qos_profile_t pub_qos_profile = rmw_qos_profile_default;


  pub_qos_profile.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  pub_qos_profile.depth = 10;
  pub_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  
  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image",pub_qos_profile);
  timer_ = this->create_wall_timer(
    33ms, std::bind(&MyTestPub::timer_callback, this));
}
void MyTestPub::timer_callback()
{
  static cv::Mat frame_(1920,1080,CV_8UC3);
  sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
  msg->header.frame_id = "camera_frame";
  msg->height = frame_.rows;
  msg->width = frame_.cols;
  msg->encoding = "bgr8";
  msg->is_bigendian = false;
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
  msg->data.assign(frame_.datastart, frame_.dataend);
  // RCLCPP_INFO(this->get_logger(), "Publishing:");
  publisher_->publish(msg);
}
