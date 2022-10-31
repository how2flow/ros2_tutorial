#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "arithmetic/argument.hpp"

using namespace std::chrono_literals;

Argument::Argument(const rclcpp::NodeOptions & node_options)
  : Node("argument", node_options),
  min_random_num_(0.0),
  max_random_num_(9.0)
{
  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  arithmetic_argument_publisher_ =
    this->create_publisher<ArithmeticArgument>("arithmetic_argument", QOS_RKL10V);

  timer_ =
    this->create_wall_timer(1s, std::bind(&Argument::publish_random_arithmetic_arguments, this));
}

Argument::~Argument()
{
}

void Argument::publish_random_arithmetic_arguments()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> distribution(min_random_num_, max_random_num_);

  msg_srv_action_interface_example::msg::ArithmeticArgument msg;
  msg.stamp = this->now();
  msg.argument_a = distribution(gen);
  msg.argument_b = distribution(gen);
  arithmetic_argument_publisher_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Published argument_a %.2f", msg.argument_a);
  RCLCPP_INFO(this->get_logger(), "Published argument_b %.2f", msg.argument_b);
}

void print_help()
{
  printf("For argument node:\n");
  printf("node_name [-h]\n");
  printf("Options:\n");
  printf("\t-h Help           : Print this help function.\n");
}

int main(int argc, char * argv[])
{
  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_help();
    return 0;
  }

  rclcpp::init(argc, argv);
  auto argument = std::make_shared<Argument>();
  rclcpp::spin(argument);
  rclcpp::shutdown();

  return 0;
}
