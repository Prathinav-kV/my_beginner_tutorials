// add_two_ints_service.cpp

/**
 * @file add_two_ints_service.cpp
 * @brief ROS 2 Service Node that adds two integers and returns the result.
 * 
 * This node provides a service to add two integers. The service takes two integer inputs
 * and returns their sum.
 */

// Copyright 2024 Prathinav Karnala Venkata

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

/**
 * @class AddTwoIntsService
 * @brief A service node that adds two integers and returns the result.
 */
class AddTwoIntsService : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the AddTwoIntsService node.
   */
  AddTwoIntsService() : Node("add_two_ints_service") {
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints",
        std::bind(&AddTwoIntsService::handle_add_two_ints, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service 'add_two_ints' is ready to add two numbers.");
  }

 private:
  /**
   * @brief Callback function to handle the add_two_ints service requests.
   * 
   * @param request The request containing the two integers to be added.
   * @param response The response containing the sum of the two integers.
   */
  void handle_add_two_ints(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
                           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    RCLCPP_DEBUG(this->get_logger(), "Received request: a=%ld, b=%ld", request->a, request->b);
    response->sum = request->a + request->b;
    RCLCPP_INFO(this->get_logger(), "Sending response: %ld", response->sum);
    RCLCPP_WARN(this->get_logger(), "Warning: ensure inputs are within valid range.");
    RCLCPP_ERROR(this->get_logger(), "Simulating an error if inputs were invalid (hypothetical).");
    RCLCPP_FATAL(this->get_logger(), "Fatal error simulation (hypothetical).");
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsService>());
  rclcpp::shutdown();
  return 0;
}
