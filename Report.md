## Problem Definition
In commercial alligator farms, the harvesting of alligator eggs is a costly and dangerous operation. In existing solutions, human operators are typically suspended in the air and descended into the desired location using helicopters. The present project aims to develop a cognitive robot capable of harvesting alligator eggs. 

Figure 1. Crocoloco Farm, HaArava, Israel (left); Ingwelala, NZ (right).
## Table of Contents
- [World Simulation](#World Simulation)
- [SLAM Operation](#SLAM Operation)
- [Localization and Navigation](#Localization and Navigation)
- [Arm inverse kinematics using MoveIt](#Arm inverse kinematics using MoveIt)
- [Computer Vision](#Computer Vision)
- [Preparing the virtual environment (RViz)](#Preparing the virtual environment (RViz))
- [References](#References)

## World Simulation
Since we do not have access to a physical alligator farm environment, we design a simulated environment using Gazebo. The environment comprises a fenced off environment where various types of obstacles are scattered around. The goal alligator egg is positioned at one corner while the robot starting position is at another. 

In the process of designing the Gazebo environment, we experienced technical errors in the process of importing external STL models, as such we were limited to the built-in 3D model components in Gazebo. Further, our graphical computation power limits the size of the world we can simulate. 

Figure 2. Simulated alligator farm environment in Gazebo.
## SLAM Operation
Using a TurtleBot equipped with a laser sensor, we perform Simultaneous Localization and Mapping (SLAM) operation on the Gazebo-simulated alligator farm environment using the GMapping package and visualise it in RViz.

By placing the robot in the simulated environment and by teleoping the robot around, we created a 2D occupancy grid map in the format of a .pgm image file and a .yaml file. We save the map files for future navigation purposes. 


Figure 2. 2D occupancy grid map visualised in RViz
## Localization and Navigation
Using the map obtained in the previous section, which is broadcasted by the map_server package as a ROS service, we provide a 2D pose estimate and teleoping the robot around to perform localization using amcl. 

We then provide a navigation goal, i.e. the location of the alligator egg. Path planning is performed using move_base and the robot is navigated to the egg location, upon which it will perform the egg harvesting procedure. Visualisation in RViz.


Figure 3. Robot in navigation, visualised in RViz.


## Arm inverse kinematics using MoveIt
Robot arm inverse kinematics is a crucial aspect of robot motion planning that involves calculating the joint angles required to position the end effector of a robot arm at a desired location in space. We used it to solve the inverse kinematics problem for a 6-degree-of-freedom robot arm. The objective was to move the end effector of the robot arm to a desired position and orientation in space. We first created a MoveIt package for our robot arm, which included the robot description, joint limits, and kinematic solver. We then used the MoveIt RViz plugin to visualize the robot arm and define the desired end effector pose. Finally, we utilized MoveIt's inverse kinematics solver to generate joint angles for the robot arm, allowing it to reach the desired pose.  we define a set of goal state for the robot arm, such as the desired end effector position and orientation to collect the egg, and the software will automatically calculate the joint angles necessary to achieve those goals. 


Figure 4. Panda Robot arm (6 DOF) and the egg in RViz environment


Steps: 
Locating the egg with CV
Calculation of the IK using rviz to collect the egg in the desired pose from the CV publisher.
Extension of the arm, grabbing motion and retracting


## Computer Vision
In our project, we faced the challenge of locating an egg in a visual scene without access to a dataset of egg images to train a neural network for image recognition. To solve this problem, we turned to using OpenCV and ArUco markers stamped on the egg. By detecting the markers in the visual scene and using their known orientation and position, we were able to accurately locate the egg in real-time. This approach proved to be effective and allowed us to successfully complete our project without the need for extensive image training data. We also faced some challenges integrating the openCV script and ROS virtual cameras, we found a library called CV_bridge that serves as translator from ROS Image topic to RGB images that can be used in openCV and viceversa. Using this library we succeed in identifying the virtual ArUco marker using the virtual camera located on the arm. This enabled us to estimate the pose (location and orientation in 3D) of the marker/egg , which publishes in object_pose_topic the necessary information for the arm goal state.




Figure 5. Virtual arUco marker placed on the egg and the position and orientation vector respect to the virtual camera.

## Preparing the virtual environment (RViz)
An unexpected challege came from creting the virtual environment RViz in order to add the ArUco marker we had to use a library called rviz_textured_quads by lucasw that creates a mesh in RViz that subscribes to an image topic, and then create a node that publishes the image using the CV_bridge mentioned before to convert from RGB to Image topic message. Then something similar to create the camera publisher we referenced to a package  rviz_camera_stream that creates the camera publisher node and lastly we created a node to add the egg mesh dynamically based on moveit python interface source code, we had to dig deep in order to find a way to add meshes dynamically to RViz, and something that we couldnt do is add the appropriate colliders in order for the arm to be able to pick up the egg mesh.

Figure 6. Cv_bridge scheme


Figure 7.Rviz Mesh streaming demo from lucasw
## References
ROS packages: move_base, amcl, map_server, turtlebot3_gazebo, turtlebot3_teleop, turtlebot3_navigation, gmapping, spawn_urdf, controller_spawner, robot_state_publisher
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch 
rosrun map_server map_saver -f ~/map

roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
Lucas W. rviz_textured_quads [Internet]. GitHub; 2019 [cited 2023 Apr 15]. Available from: https://github.com/lucasw/rviz_textured_quads
Lucas W. rviz_camera_stream [Internet]. GitHub; 2020 [cited 2023 Apr 15]. Available from: https://github.com/lucasw/rviz_camera_stream
ROS-Planning. moveit_tutorials [Internet]. GitHub; 2021 [cited 2023 Apr 15]. Available from: https://github.com/ros-planning/moveit_tutorials/tree/master/doc/move_group_python_interface
OpenCV. ArUco Detection [Internet]. OpenCV documentation; 2021 [cited 2023 Apr 15]. Available from: https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
ROS. Converting between ROS images and OpenCV images (Python) [Internet]. ROS Wiki; 2021 [cited 2023 Apr 15]. Available from: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython




