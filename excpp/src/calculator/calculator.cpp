// Copyright 2021 OROCA
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
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "calculator/calculator.hpp"


Calculator::Calculator(const rclcpp::NodeOptions & node_options)
: Node("calculator", node_options),
  argument_a_(0.0),
  argument_b_(0.0)
{
  RCLCPP_INFO(this->get_logger(), "Run calculator");

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  arithmetic_argument_subscriber_ = this->create_subscription<ArithmeticArgument>(
    "arithmetic_argument",
    QOS_RKL10V,
    [this](const ArithmeticArgument::SharedPtr msg) -> void
    {
      argument_a_ = msg->argument_a;
      argument_b_ = msg->argument_b;

      RCLCPP_INFO(
        this->get_logger(),
        "Timestamp of the message: sec %ld nanosec %ld",
        msg->stamp.sec,
        msg->stamp.nanosec);

      RCLCPP_INFO(this->get_logger(), "Subscribed argument a: %.2f", argument_a_);
      RCLCPP_INFO(this->get_logger(), "Subscribed argument b: %.2f", argument_b_);
    }
  );
}

Calculator::~Calculator()
{
}
