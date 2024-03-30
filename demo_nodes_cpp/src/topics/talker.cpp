// Copyright 2014 Open Source Robotics Foundation, Inc.
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
#include <cstdio>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/string.hpp"

#include "demo_nodes_cpp/visibility_control.h"

using namespace std::chrono_literals;

namespace demo_nodes_cpp
{
// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit Talker(const rclcpp::NodeOptions & options)
  : Node("talker", options)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("chatter", rclcpp::QoS(1));
    timer_ = this->create_wall_timer(10ms, std::bind(&Talker::onTimer, this));
  }

private:
  size_t count_ = 1;
  std::unique_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  void onTimer();
  void main_logic();
  double sub_logic_calc(size_t repeat_num = 10000000);
  double sub_logic_math(uint64_t repeat_num = 10000000);
  uint8_t sub_logic_mem(uint64_t repeat_num = 10000000, uint64_t array_size = 1000);
};

void Talker::onTimer()
{
  msg_ = std::make_unique<std_msgs::msg::String>();
  msg_->data = "Hello World: " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
  main_logic();
  pub_->publish(std::move(msg_));
}

void Talker::main_logic()
{
  auto start_func_1 = std::chrono::steady_clock::now();
  sub_logic_calc();
  auto end_func_1 = std::chrono::steady_clock::now();

  auto start_func_2 = std::chrono::steady_clock::now();
  sub_logic_math();
  auto end_func_2 = std::chrono::steady_clock::now();

  auto start_func_3 = std::chrono::steady_clock::now();
  sub_logic_mem();
  auto end_func_3 = std::chrono::steady_clock::now();

  RCLCPP_INFO(this->get_logger(),
    "sub_logic_calc: %ld [ms], sub_logic_math: %ld [ms], sub_logic_mem: %ld [ms]",
    std::chrono::duration_cast<std::chrono::milliseconds>(end_func_1 - start_func_1).count(),
    std::chrono::duration_cast<std::chrono::milliseconds>(end_func_2 - start_func_2).count(),
    std::chrono::duration_cast<std::chrono::milliseconds>(end_func_3 - start_func_3).count()
  );
}

double Talker::sub_logic_calc(size_t repeat_num)
{
  double sum = 0;
  for (uint64_t i = 0; i < repeat_num; ++i) {
    sum += i;
  }
  return sum;
}

double Talker::sub_logic_math(uint64_t repeat_num)
{
  double sum = 0;
  for (uint64_t i = 0; i < repeat_num; ++i) {
    sum += sqrt(sin(i));
  }
  return sum;
}

uint8_t Talker::sub_logic_mem(uint64_t repeat_num, uint64_t array_size)
{
  uint8_t temp = 0;
  auto src = std::make_unique<uint8_t[]>(array_size);
  auto dst = std::make_unique<uint8_t[]>(array_size);
  for (uint64_t i = 0; i < repeat_num; ++i) {
    memcpy(&dst[0], &src[0], array_size * sizeof(uint8_t));
    temp = dst[0];
  }
  return temp;
}

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::Talker)
