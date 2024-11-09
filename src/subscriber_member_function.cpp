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
 * @file subscriber_node.cpp
 * @brief Node that subscribes to the "chatter" topic and provides a client
 * service to toggle publishing in the publisher node.
 *
 * This node subscribes to messages from the "chatter" topic and can send a
 * request to start or stop message publishing on the publisher node through the
 * "toggle_publishing" service.
 */

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief A simple subscriber node that listens to messages on the "chatter"
 * topic and provides a client service to toggle the publisher's publishing
 * state.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalSubscriber.
   * @param start_publishing Initial state for the publisher's publishing; if
   * true, starts publishing, otherwise stops.
   *
   * Initializes the subscription to the "chatter" topic and creates a client
   * for the "toggle_publishing" service. Sets a timer to attempt sending the
   * toggle request shortly after node startup.
   */
  explicit MinimalSubscriber(bool start_publishing)
      : Node("minimal_subscriber"), start_publishing_(start_publishing) {
    // Create subscription to "chatter" topic with a queue size of 10
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "chatter", 10,
        std::bind(&MinimalSubscriber::chatter_callback, this, _1));

    // Initialize client for the "toggle_publishing" service
    client_ = this->create_client<std_srvs::srv::SetBool>("toggle_publishing");

    // Set a timer to attempt sending the toggle request after a delay to ensure
    // the publisher node is up
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MinimalSubscriber::send_toggle_request, this));
  }

 private:
  /**
   * @brief Callback function that logs messages received on the "chatter"
   * topic.
   * @param msg The message received from the publisher on the "chatter" topic.
   */
  void chatter_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  /**
   * @brief Attempts to send a request to toggle publishing on the publisher
   * node.
   *
   * Waits for the "toggle_publishing" service to be available and sends a
   * request based on the `start_publishing_` flag. This function logs the
   * success or failure of the service call.
   */
  void send_toggle_request() {
    // Check if the service is available, with a 1-second wait
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(
          this->get_logger(),
          "Waiting for the toggle_publishing service to be available...");
      return;
    }

    // Prepare the service request
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = start_publishing_;

    // Send the request asynchronously and handle the response
    client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
          auto response =
              future.get();  // Dereference the future to get the response
          if (response->success) {
            RCLCPP_INFO_STREAM(this->get_logger(),
                               "Service call succeeded: " << response->message);
          } else {
            RCLCPP_ERROR_STREAM(this->get_logger(),
                                "Service call failed: " << response->message);
          }
        });

    // Cancel the timer to prevent repeated requests
    timer_->cancel();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      subscription_;  ///< Subscription to listen for messages on the "chatter"
                      ///< topic
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr
      client_;  ///< Client to toggle publishing in the publisher node
  rclcpp::TimerBase::SharedPtr
      timer_;              ///< Timer to delay sending the toggle request
  bool start_publishing_;  ///< Flag to indicate whether to start or stop
                           ///< publishing in the publisher
};

/**
 * @brief Main function that initializes and spins the MinimalSubscriber node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status code.
 *
 * This function parses command-line arguments to determine the initial
 * publishing state (start or stop) and then starts the MinimalSubscriber node
 * accordingly.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // Default behavior is to start publishing
  bool start_publishing = true;

  // Parse command-line arguments to set start_publishing flag
  if (argc > 1) {
    std::string arg(argv[1]);
    if (arg == "stop") {
      start_publishing = false;
    } else if (arg != "start") {
      RCLCPP_WARN(
          rclcpp::get_logger("rclcpp"),
          "Unknown argument. Use 'start' or 'stop'. Defaulting to 'start'.");
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Starting MinimalSubscriber with toggle request: %s",
              start_publishing ? "start" : "stop");

  // Spin the MinimalSubscriber node
  rclcpp::spin(std::make_shared<MinimalSubscriber>(start_publishing));
  rclcpp::shutdown();
  return 0;
}
