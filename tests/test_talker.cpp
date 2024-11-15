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
 * @brief Integration test for the MinimalPublisher node using Catch2 framework.
 *
 * This test verifies the functionality of the MinimalPublisher node, including
 * the topic publisher, TF broadcast, and service toggling.
 */

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

/**
 * @brief Test case for verifying the functionality of the Talker node.
 *
 * This test case includes sections for verifying the topic publisher, TF frame
 * broadcast, and the service for toggling publishing.
 */
TEST_CASE("Talker Node Test", "[talker]") {
  auto node = rclcpp::Node::make_shared("test_node");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  /**
   * @brief Section to verify the chatter topic publishing functionality.
   *
   * This section subscribes to the "chatter" topic and checks if the received
   * messages contain the expected text.
   */
  SECTION("Verify chatter topic publishing") {
    auto chatter_sub = node->create_subscription<std_msgs::msg::String>(
        "chatter", 10, [](const std_msgs::msg::String::SharedPtr msg) {
          REQUIRE(msg->data.find("Terps love to count till:") != std::string::npos);
        });

    // Spin for a short time to allow messages to be received
    rclcpp::Rate rate(2);  // 2 Hz
    for (int i = 0; i < 5; ++i) {
      executor.spin_some();
      rate.sleep();
    }
  }

  /**
   * @brief Section to verify the TF frame broadcast functionality.
   *
   * This section listens for the TF transform between "world" and "talk" frames
   * and checks if the transform values match the expected values.
   */
  SECTION("Verify TF frame broadcast") {
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Allow some time for the transform to be broadcast
    rclcpp::Rate rate(1);  // 1 Hz
    for (int i = 0; i < 3; ++i) {
      executor.spin_some();
      rate.sleep();
    }

    // Attempt to lookup the transform from "world" to "talk"
    REQUIRE_NOTHROW(tf_buffer.lookupTransform("world", "talk", tf2::TimePointZero));

    auto transform = tf_buffer.lookupTransform("world", "talk", tf2::TimePointZero);
    REQUIRE(transform.transform.translation.x == Approx(1.0));
    REQUIRE(transform.transform.translation.y == Approx(2.0));
    REQUIRE(transform.transform.translation.z == Approx(3.0));
    REQUIRE(transform.transform.rotation.w == Approx(1.0));
  }

  /**
   * @brief Section to verify the service for toggling publishing.
   *
   * This section calls the "toggle_publishing" service to stop publishing and
   * verifies the response from the service.
   */
  SECTION("Verify service toggle publishing") {
    auto client = node->create_client<std_srvs::srv::SetBool>("toggle_publishing");
    REQUIRE(client->wait_for_service(std::chrono::seconds(5)));

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    auto result_future = client->async_send_request(request);

    // Wait for the result
    REQUIRE(rclcpp::spin_until_future_complete(node, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS);

    auto response = result_future.get();
    REQUIRE(response->success == true);
    REQUIRE(response->message == "Publishing stopped.");
  }
}
