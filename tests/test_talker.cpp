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
 * @file test_talker.cpp
 * @brief Integration test for the talker node's publisher functionality.
 *
 * This test verifies that the `talker` node publishes messages on the "chatter" topic
 * using the Catch2 framework.
 */

#include <catch2/catch.hpp>
#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief Test case to verify that the `talker` node publishes messages.
 *
 * This test spins up the `talker` node and a test subscriber. It waits for a
 * message on the "chatter" topic and validates that the message matches the
 * expected content.
 */
TEST_CASE("Talker node publishes static messages", "[publisher]") {
  // Initialize ROS2
  rclcpp::init(0, nullptr);

  // Create an executor
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Launch the `talker` node
  auto talker_node = std::make_shared<rclcpp::Node>("talker");

  // Variable to store the received message
  std::string received_message;

  // Create a test subscriber node to listen to "chatter"
  auto subscriber_node = rclcpp::Node::make_shared("test_subscriber");
  auto subscription = subscriber_node->create_subscription<std_msgs::msg::String>(
      "chatter", 10, [&received_message](const std_msgs::msg::String::SharedPtr msg) {
        received_message = msg->data;
      });

  // Add nodes to the executor
  executor->add_node(talker_node);
  executor->add_node(subscriber_node);

  // Run the executor in a separate thread
  std::thread executor_thread([&executor]() {
    executor->spin();
  });

  // Wait for a message to be received
  rclcpp::sleep_for(std::chrono::milliseconds(1000));  // Wait for publisher to send messages

  // Check that a message was received
  REQUIRE(!received_message.empty());
  REQUIRE(received_message == "Terps love to count");

  // Clean up
  executor->cancel();
  executor_thread.join();
  rclcpp::shutdown();
}
