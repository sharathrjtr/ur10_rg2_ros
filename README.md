# ur10_rg2_ros
This repository provides codes for urdf, control and interface of On-Robot's RG2 gripper with ROS, MoveIt using UR10 manipulator.
Link1: https://www.universal-robots.com/plus/product/rg2-gripper-22723/
Link2: http://onrobot.com/products/

- Author: Sharath Jotawar
- Email: sharathrjtr@gmail.com 
- Video: https://www.youtube.com/watch?v=lCxMGvCKe_g

--------------------------------------
Downloads: 
+ Clone the dependencies
	- https://github.com/ros-industrial/universal_robot

--------------------------------------
Packages:
+ robot_descriptions: Provides urdf xacro files for ur10 manipulator mounted with the RG2 gripper and kinect sensor.
+ ur10_rg2_moveit_config: Moveit config package for the ur10 manipulator.
+ ur_control: package for control operations for the manipulator. Provides service files, scripts for control and gripping action detection for RG2 gripper
+ ur_modern_driver: Modified ur_ros_wrapper, ur_driver to provide ROS service server for interfacing with the gripper

--------------------------------------
About the urdf:
+ The fingers of RG2 gripper cannot be moved in simulation(RViz or Gazebo) as the mesh file for the body and fingers are combined together.
+ I shall modify and add a joint for the finger movement once i get separate STEP files for the fingers of the gripper.
+ The urdf in its present can be utilized for collision detection during motion plans for manipulator.

--------------------------------------
Visualize in RViz: 
+ $ roslaunch ur10_rg2_moveit_config demo.launch
- or
+ $roslaunch urdf_tutorial display.launch model:=robot_descriptions/description/ur10_robot_rg2_arc.urdf.xacro gui:=true

Connect the RG2 gripper with the UR manipulator. Now control opening width of Gripper in real, checking gripping action with ur_modern_driver:
+ $ roslaunch ur_control ur10_control.launch robot_ip:=${robot_ip}
+ $ rosrun ur_control rg2_server_node.py

Control opening width in mm. Usual range 110 to 0 mm
+ Open: $ rosservice call /rg2_gripper/control_width ur_control/RG2 110
+ Close: $ rosservice call /rg2_gripper/control_width ur_control/RG2 0

To check whether an object has been gripped or not
+ $ rosservice call /rg2_gripper/grip_detect ur_control/RG2_Grip





