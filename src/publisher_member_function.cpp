// Copyright 2024 Prathinav Karnala Venkata
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

/**
 * @file publisher_node.cpp
 * @brief Node that publishes messages on the "chatter" topic at a configurable
 * frequency.
 *
 * This node also provides a service to toggle publishing on or off. The
 * publishing frequency can be set via command-line arguments.
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"  // Include service header for SetBool

/**
 * @class MinimalPublisher
 * @brief A simple publisher node that publishes a message on the "chatter"
 * topic.
 *
 * This class provides a service to toggle message publishing and logs
 * messages at different levels. The frequency of publishing can be specified
 * via a command-line argument.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher.
   * @param frequency Frequency at which to publish messages (in Hz).
   */
  explicit MinimalPublisher(double frequency)
      : Node("minimal_publisher"), count_(0), is_publishing_(true) {
    // Create publisher on "chatter" topic with a queue size of 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);

    // Set the timer frequency based on the input argument or default
    auto interval =
        std::chrono::milliseconds(static_cast<int>(1000.0 / frequency));
    timer_ = this->create_wall_timer(
        interval, std::bind(&MinimalPublisher::timer_callback, this));

    // Create a service to toggle publishing
    service_ = this->create_service<std_srvs::srv::SetBool>(
        "toggle_publishing",
        std::bind(&MinimalPublisher::handle_service, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "MinimalPublisher node has been started with frequency: "
                           << frequency << " Hz.");
  }

 private:
  /**
   * @brief Timer callback function that publishes messages if publishing is
   * enabled.
   */
  void timer_callback() {
    if (!is_publishing_) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing is paused.");
      return;
    }
    auto message = std_msgs::msg::String();
    message.data = "Terps love to count till: " + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: '" << message.data << "'");
    publisher_->publish(message);
  }

  /**
   * @brief Service handler to toggle publishing on or off.
   * @param request Boolean value to enable (true) or disable (false)
   * publishing.
   * @param response Response with success status and message.
   */
  void handle_service(const std_srvs::srv::SetBool::Request::SharedPtr request,
                      std_srvs::srv::SetBool::Response::SharedPtr response) {
    if (request->data) {
      if (is_publishing_) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Publishing is already started.");
        response->success = false;
        response->message = "Already publishing.";
      } else {
        is_publishing_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing resumed.");
        response->success = true;
        response->message = "Publishing started.";
      }
    } else {
      if (!is_publishing_) {
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Publishing is already stopped.");
        response->success = false;
        response->message = "Already stopped.";
      } else {
        is_publishing_ = false;
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing stopped.");
        response->success = true;
        response->message = "Publishing stopped.";
      }
    }
  }

  rclcpp::TimerBase::SharedPtr
      timer_;  ///< Timer for scheduling publishing intervals
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      publisher_;  ///< Publisher to publish messages
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr
      service_;         ///< Service to toggle publishing on/off
  size_t count_;        ///< Counter for messages published
  bool is_publishing_;  ///< Flag indicating whether publishing is active
};

/**
 * @brief Main function that initializes and spins the MinimalPublisher node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Default frequency is set to 2 Hz
  double frequency = 2.0;

  if (argc > 1) {
    try {
      frequency = std::stod(argv[1]);
      RCLCPP_INFO_STREAM(
          rclcpp::get_logger("rclcpp"),
          "Frequency set to " << frequency << " Hz via command line argument.");
    } catch (const std::invalid_argument& e) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                          "Invalid frequency argument. Using default 2 Hz.");
    }
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                      "Initializing MinimalPublisher node with frequency: "
                          << frequency << " Hz.");

  try {
    rclcpp::spin(std::make_shared<MinimalPublisher>(frequency));
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"),
                        "Caught runtime error: " << e.what());
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                        "Unexpected error occurred.");
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("rclcpp"),
                      "Shutting down MinimalPublisher node.");
  rclcpp::shutdown();
  return 0;
}
