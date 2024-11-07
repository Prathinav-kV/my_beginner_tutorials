# ROS Publisher and Subscriber tutorial

## Description

The `beginner_tutorials` package is a simple ROS 2 package developed as part of the ROS 2 Humble beginner tutorials. This package includes a publisher node (`talker`) and a subscriber node (`listener`). The publisher sends a custom string message, while the subscriber receives and logs the message. These nodes demonstrate basic publisher-subscriber communication in ROS 2.

## Dependencies

This package relies on the following dependencies:

- **ROS 2 Humble** - The version of ROS 2 used to build and run the nodes.
- **rclcpp** - The C++ API for ROS 2, required for writing ROS 2 nodes.
- **std_msgs** - Provides standard message types, including the `String` type used in this package.

Ensure these dependencies are installed in your ROS 2 workspace.


## Clone the Repository

To get started, clone the repository into your ROS 2 workspace:

```bash
cd ~/ros_ws/src
git clone https://github.com/Prathinav-kV/my_beginner_tutorials.git
```

## File Structure

To set up the workspace, clone the repository into your `ros_ws/src` directory. The resulting file structure should look like this:
```
ros_ws/
 └── src/
  └── beginner_tutorials/
   ├── package.xml
   ├── CMakeLists.txt 
   ├── src/ 
   │ ├── talker.cpp 
   │ └── listener.cpp 
   └── README.md
```
## Build the Package

Navigate to the workspace root and build the package using colcon:
```bash
cd ~/ros_ws
colcon build --packages-select beginner_tutorials
```
## Running the Nodes
After building, you can run the publisher and subscriber nodes in separate terminals.

### Run the Publisher:

In a new terminal, source your workspace, and then run the publisher:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials talker
```
### Run the Subscriber:

In another terminal, source the workspace, and then run the subscriber:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials listener
```
## Summary

With these steps, you should see the publisher sending messages and the subscriber receiving and displaying them. This setup demonstrates basic communication between nodes using the ROS 2 client library in C++.

