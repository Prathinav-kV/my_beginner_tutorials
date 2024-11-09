# ROS Publisher and Subscriber tutorial

## Description

The 'beginner_tutorials' package is a simple ROS 2 package developed as part of the ROS 2 Humble tutorials. This package includes a service node (add_two_ints_service) and a client node (add_two_ints_client). The service node provides a service to add two integers, while the client node sends a request to add two integers and receives the result. These nodes demonstrate basic service-client communication in ROS 2.

## Dependencies

This package relies on the following dependencies:

- **ROS 2 Humble** - The version of ROS 2 used to build and run the nodes.
- **rclcpp** - The C++ API for ROS 2, required for writing ROS 2 nodes.
- **example_interfaces** - Provides standard service types, including the AddTwoInts service used in this package.
- **std_msgs** - Provides standard message types, including the String type used in this package.


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
   │ ├── add_two_ints_service.cpp 
   │ ├── add_two_ints_client.cpp 
   │ ├── talker.cpp 
   │ └── listener.cpp 
   ├── launch/
   │ └── add_two_ints_launch.py
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

### Run the Service Node:

In a new terminal, source your workspace, and then run the service node:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials add_two_ints_service
```
### Run the Client Node:

In another terminal, source the workspace, and then run the client node. By default, the client will request the addition of two integers (e.g., 10 and 20). You can also specify different integers as arguments.

**Example without arguments:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials add_two_ints_client
```
**Example without arguments:**
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials add_two_ints_client 100 500

```
### Using the Launch File
You can also run both nodes simultaneously using the provided launch file, with options to modify the integers being sent by the client node:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch beginner_tutorials add_two_ints_launch.py a:=5 b:=10
```
## Summary

With these steps, you should see the talker node publishing messages, the listener node receiving them, and be able to control the publishing state with a service call. This setup demonstrates basic publisher-subscriber communication and service control in ROS 2, using the ROS 2 client library in C++.

