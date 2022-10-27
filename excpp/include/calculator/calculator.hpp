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

#ifndef CALCULATOR__CALCULATOR_HPP_
#define CALCULATOR__CALCULATOR_HPP_

#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "msg_srv_action_interface_example/msg/arithmetic_argument.hpp"

class Calculator : public rclcpp::Node
{
public:
  using ArithmeticArgument = msg_srv_action_interface_example::msg::ArithmeticArgument;

  explicit Calculator(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~Calculator();

private:
  rclcpp::Subscription<ArithmeticArgument>::SharedPtr
    arithmetic_argument_subscriber_;

  float argument_a_;
  float argument_b_;
};
#endif  // CALCULATOR__CALCULATOR_HPP_
