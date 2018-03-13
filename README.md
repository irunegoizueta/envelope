# Robotic string-envelope opening
This is a ROS package to untie the string of an envelope that has been previously tied arbitrarily. 
## Hardware requirements
- Universal Robot UR10
- Robotiq FT300 wrist force/torque sensor
- Robotiq 3 finger gripper
- String envelope:
    - Distance between pivots: 4,25 cm
    - Pivots' radius: 4 mm

 Initially, the robot is grasping the end of the string in the middle of pivots.
## Software prerequisites
- ROS kinetic. 
- Universal robot package (communication with UR10 controllers).
- Robotiq package (communication with FT300 force/torque sensor and 3 finger gripper). 
- Software tested on Ubuntu 16.04.3 LTS.
## How to install it
Copy the "envelope" repository inside your workspace and compile the executables using these commands:
```
  catkin_make
  source/devel setup.bash
```

## Protocol
First, make the connection with UR10 robot by launching these files in three different terminals:
```
  roslaunch ur_modern_driver ur10_bringup.launch limited:=true robot_ip:=192.168.1.102 [reverse_port:=REVERSE_PORT]
  roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true
  roslaunch ur10_moveit_config moveit_rviz.launch config:=true
```
In another terminal, run the executable so the robot goes to the initial position:
```
  rosrun envelope initial_position.py
```
Make the connection with the force/torque sensor by typing in two different terminals:
```
  sudo chmod 777 /dev/ttyUSB1
  rosrun robotiq_force_torque_sensor rq_sensor
```
Make the connection with the 3 finger gripper by typing in other two different terminals:
```
  rosrun robotiq_s_model_control SModelTcpNode.py 192.168.1.11
  rosrun robotiq_s_model_control SModelSimpleController.py
```
Press "a" to activate the gripper, then press "p" to put it in pinch mode and press "c" to close the gripper and grasp the end of the string. 

Finally, run the executable to open the envelope:
```
  rosrun envelope open_envelope.py
```

## Example
In the link below, you can find a video of how the algorithm to open the envelope works: 

https://youtu.be/gpvwUnB7S3o
