cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Catch2 2 REQUIRED)

# Talker executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs std_srvs tf2_ros geometry_msgs)

# Listener executable
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs std_srvs)

# Test executable for talker
add_executable(test_talker tests/test_talker.cpp)
ament_target_dependencies(test_talker rclcpp std_msgs std_srvs tf2_ros geometry_msgs)
target_link_libraries(test_talker Catch2::Catch2WithMain)

# Install executables
install(TARGETS
  talker
  listener
  test_talker
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Register the test
  add_test(NAME talker_publisher_test
    COMMAND test_talker
  )
endif()

ament_package()
