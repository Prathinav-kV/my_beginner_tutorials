# ROS 2 Beginner Tutorials

This project demonstrates basic ROS 2 functionality, including a publisher node (`talker`), a subscriber node (`listener`), a Catch2-based test for the publisher, and a launch file that optionally records ROS 2 bag files. It highlights concepts like topic communication, services, and TF2 integration.

---

## **Technologies and Libraries Used**

- **ROS 2 (Humble):** Core framework for robot development.
- **C++:** Language for node and test implementation.
- **Catch2:** Framework for unit testing.
- **TF2:** Library for handling transformations.
- **std_msgs:** ROS 2 package for standard message types.
- **std_srvs:** ROS 2 package for standard services.
- **rosbag2:** ROS 2 package for recording and replaying data.

---

## **File Structure**

```
my_beginner_tutorials
├── CMakeLists.txt
├── launch
│   └── service_launch.py
├── package.xml
├── README.md
├── results
│   ├── Assignment_1
│   │   ├── clang_tidy.txt
│   │   └── cpplint.txt
│   ├── Assignment_2
│   │   ├── clang_tidy.txt
│   │   ├── cpplint.txt
│   │   └── rqt_new.png
│   └── Assignment_3
│       ├── bag_record
│       │   ├── bag_record_0.db3
│       │   └── metadata.yaml
│       ├── clang_tidy.txt
│       ├── cpplint.txt
│       └── frames_2024-11-15_17.52.29.pdf
├── src
│   ├── publisher_member_function.cpp
│   └── subscriber_member_function.cpp
└── tests
    └── test_talker.cpp

8 directories, 17 files
```
## Build Instructions
Ensure your ROS 2 environment is sourced:

```bash
source /opt/ros/humble/setup.bash
```
Clone the repository and navigate to the project directory:

```bash
git clone <repository-url>
cd <repository-directory>
```
Build the project using colcon:

```bash
colcon build
```
Source the setup file to overlay your workspace:

```bash
source install/setup.bash
```
## How to Run the Project
Run Instructions
### 1. Run the Publisher Node (talker)
Start the publisher node to begin publishing messages on the /chatter topic:

```bash
ros2 run beginner_tutorials talker
```
Optional Argument: Specify the frequency (in Hz) of message publishing:
```bash
ros2 run beginner_tutorials talker 5.0
```
(Default frequency is 2 Hz.)

### 2. Run the Subscriber Node (listener)
Start the subscriber node to listen to messages on the /chatter topic:

```bash
ros2 run beginner_tutorials listener
```

### 3. Inspect TF Frames
The talker node broadcasts a TF2 transform (talk -> world). Use the following command to inspect the frames:

```bash
ros2 run tf2_tools view_frames
```
This generates a frames.pdf file visualizing the TF tree. You can also echo the transform:

```bash
ros2 topic echo /tf
```

### 4. Run Tests
To validate the functionality of the talker node, run the test_talker executable after starting the talker node:

Start the talker node:

```bash
ros2 run beginner_tutorials talker
```
Run the test in a separate terminal:

```bash
./build/beginner_tutorials/test_talker
```
Expected Output: The test checks if the talker node publishes the expected message ("Terps love to count").

## ROS 2 Bag Files
### 1. Record Bag File Using Launch File
Use the launch file to record all topics into a bag file:

```bash
ros2 launch beginner_tutorials service_launch.py record:=true
```
Bag File Location: results/bag_record
Behavior:

Records all topics, including /chatter.
Stops recording automatically after 15 seconds.

### 2. Disable Bag File Recording
To run the launch file without recording a bag file:

```bash
ros2 launch beginner_tutorials service_launch.py record:=false
```
### 3. Inspect the Bag File
Verify the contents of the recorded bag file:

```bash
ros2 bag info results/bag_record
```
Expected Output: Information about the topics recorded:
```mathematica
Files:             results/bag_record/bag_record_0.db3
Duration:          15.0s
Messages:          300
Topics with Message Count:
    /chatter          300
    /rosout           20
```
### 4. Play Back the Bag File
Replay the recorded data from the bag file:

```bash
ros2 bag play results/bag_record
```

### 5. Run the Listener Node to Verify Playback
In another terminal, start the listener node to verify playback:

```bash
ros2 run beginner_tutorials listener
```
Expected Output: The listener node should print messages from the bag file:
```css
[INFO] [listener]: I heard: 'Terps love to count'
```

## Key Features

### **Publisher Node (`talker`)**
- Publishes static messages (`"Terps love to count"`) on the `/chatter` topic.
- Offers a service (`toggle_publishing`) to start or stop publishing dynamically.
- Broadcasts a TF2 transform (`talk` -> `world`).

### **Subscriber Node (`listener`)**
- Subscribes to the `/chatter` topic.
- Sends a service request to toggle publishing in the `talker` node.

### **Integration Testing (`test_talker.cpp`)**
- Validates that the `talker` node publishes the correct messages.

### **Launch File (`service_launch.py`)**
- Starts the `talker` and `listener` nodes.
- Optionally records ROS 2 bag files.

### **Bag File Recording and Playback**
- Records all topics to a bag file for analysis or replay.


## Conclusion
This project serves as an introduction to ROS 2 programming with publishers, subscribers, services, TF2 integration, and bag file recording. It also demonstrates testing with Catch2 to ensure node functionality.


---

### **What’s New in This Update**
1. **Assumptions and Dependencies:** Explicit mention of ROS 2 Humble, OS, and required libraries.
2. **TF Frame Inspection:** Added steps for inspecting TF frames using `tf2_tools`.
3. **Disable Bag Recording:** Explained how to disable recording using `record:=false`.
4. **Structured Steps for Bag File Operations:**
   - Recording bag files.
   - Disabling bag file recording.
   - Inspecting bag files.
   - Playing back bag files with the listener node.

This updated `README.md` ensures compliance with the assignment requirements while providing a clear guide to users. Let me know if you need further adjustments!
