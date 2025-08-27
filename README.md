# Stretch OpenXR

Connect Stretch robot and ROS 2 with OpenXR seamlessly.

## 0. Stretch
```bash
stretch_system_check.py
```
System check

```bash
stretch_robot_home.py
```
Home the robot

```bash
stretch_free_robot_process.py
```
Turning off Gamepad Teleoperation

```bash
ros2 launch stretch_core stretch_driver.launch.py mode:=navigation
```
Start stretch driver

```bash
# In another terminal, test your stretch
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```
Test stretch navigation mode. This command makes robot move forward a bit. Once success, you can close those processes.

Clone this repo to robot workspace, usually it is `~/ament_ws/src`.
```bash
cd ~/ament_ws/src
git clone https://github.com/AlfredMoore/stretch_openxr.git
```

## 1. External Device
Scan all connected `Realsense` cameras on the robot and select one to publish frames to an address through `ZeroMQ` instead of ROS 2 image topics, and achive lower latency.

### Step a. Install and test realsenseZMQ
```bash
cd cd ~/ament_ws/src/stretch_openxr
git submodule update --init --remote
cd realsenseZMQ
mkdir build && cd build
cmake ..
cmake --build . -j 16   # make -j16
```
Compile and Build. The built executable is `realsenseZMQ/build/rs_zmq_publisher`

```bash
./rs_zmq_publisher
```
Raw scan Realsense camera. You can get your Realsense serial here.

```bash
./rs_zmq_publisher --serial <RealsenseSerial>
```
Run with your desired serial number.

```bash
./rs_zmq_publisher --serial <RealsenseSerial> --show
```
Run and show in local screen.

### Step b. Manage and Run realsenseZMQ by ROS 2
```bash
cd ~/ament_ws # <YOUR ROS2 WORKSPACE>
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Then
```bash
ros2 launch external_dev external_realsense.launch.py serial:=<RealsenseSerial> path:=<path to realsenseZMQ/build/rs_zmq_publisher>
```
Default path: `~/ament_ws/src/stretch_openxr/realsenseZMQ/build/rs_zmq_publisher`

## 2. Teleoperation UDP
Connect Meta Quest controller to teleoperate Stretch movement.
Open one terminal, run following to start stretch driver and switch to `navigation` mode
```bash
ros2 launch stretch_core stretch_driver.launch.py mode:=navigation
```

In another terminal, run following to enble quest controller teleoperation.
```bash
ros2 launch teleop_udp quest_teleop.launch.py port:=12345
```