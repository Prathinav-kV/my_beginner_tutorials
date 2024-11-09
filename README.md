# ROS Publisher and Subscriber tutorial

## Description

The beginner_tutorials package is a simple ROS 2 package developed to demonstrate basic publisher-subscriber communication in ROS 2. This package includes a publisher node (talker) and a subscriber node (listener). The publisher sends messages over the "chatter" topic, which the subscriber node listens to. Additionally, a service is provided to toggle the publisher on and off.

## Dependencies

This package relies on the following dependencies:

- **ROS 2 Humble** - The version of ROS 2 used to build and run the nodes.
- **rclcpp** - The C++ API for ROS 2, required for writing ROS 2 nodes.
- **example_interfaces** - Provides standard service types, including the AddTwoInts service used in this package.
- **std_msgs** - Provides standard message types, including the String type used in this package.
- **std_srvs** - Provides service types, including SetBool, used for toggling publishing on and off.


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
   │ ├── publisher_member_function.cpp 
   │ └── subscriber_member_function.cpp 
   ├── launch/
   │ └── service_launch.py
   └── README.md
```
## Build the Package

Navigate to the workspace root and build the package using colcon:
```bash
cd ~/ros_ws
colcon build --packages-select beginner_tutorials
```
## Running the Nodes
After building, you can run the publisher, subscriber, service, and client nodes in separate terminals.

### Run the Publisher Node (talker):

In a new terminal, source your workspace, and then run the publisher node:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials talker
```
You can also specify a custom frequency for publishing (e.g., 5 Hz) by adding a parameter:
```bash
ros2 run beginner_tutorials talker --ros-args -p frequency:=5.0
```

### Run the Subscriber Node (listener):

In another terminal, source the workspace, and then run the subscriber node:
```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 run beginner_tutorials listener
```
### Toggle the Publisher (Service Call to talker)
To toggle the talker node on or off, you can use the ROS 2 service command to call the toggle_publishing service.

To stop publishing:

```bash
ros2 service call /toggle_publishing std_srvs/srv/SetBool "{data: false}"
```
To start publishing:

```bash
ros2 service call /toggle_publishing std_srvs/srv/SetBool "{data: true}"
```
### Using the Launch File
You can also run both the publisher and subscriber nodes simultaneously using the provided launch file. Additionally, you can specify the publishing frequency as an argument for the talker node:

```bash
source /opt/ros/humble/setup.bash
source ~/ros_ws/install/setup.bash
ros2 launch beginner_tutorials service_launch.py frequency:=5.0
```
## Summary

With these steps, you should see the service node receiving requests and the client node receiving responses with the sum. This setup demonstrates basic service-client communication between nodes using the ROS 2 client library in C++.

