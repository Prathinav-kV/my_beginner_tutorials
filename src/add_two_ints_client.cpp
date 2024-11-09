// add_two_ints_client.cpp

/**
 * @file add_two_ints_client.cpp
 * @brief ROS 2 Client Node that requests the addition of two integers from a
 * service.
 *
 * This node acts as a client that sends two integers to be added by the service
 * and receives the sum in response.
 */

// Copyright 2024 Prathinav Karnala Venkata

#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class AddTwoIntsClient
 * @brief A client node that requests the addition of two integers from the
 * service.
 */
class AddTwoIntsClient : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the AddTwoIntsClient node.
   * @param a First integer to add.
   * @param b Second integer to add.
   */
  AddTwoIntsClient(int64_t a, int64_t b)
      : Node("add_two_ints_client"), a_(a), b_(b) {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
        "add_two_ints");
    // wait for service waiting for 1 second.
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for the service to be available...");
    }

    // setting the request body
    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a_;
    request->b = b_;
    // receiving result
    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Result: %ld", result.get()->sum);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
    }
  }

 private:
  int64_t a_;
  int64_t b_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Default values for the arguments
  int64_t a = 10;
  int64_t b = 20;
  if (argc > 2) {
    a = std::stoll(argv[1]);
    b = std::stoll(argv[2]);
  }

  rclcpp::spin(std::make_shared<AddTwoIntsClient>(a, b));
  rclcpp::shutdown();
  return 0;
}
