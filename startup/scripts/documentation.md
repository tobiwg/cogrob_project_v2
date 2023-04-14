# Documentation
## add_egg.py
The add_egg.py script uses the MoveIt! Python API to interface with MoveIt! and add an "egg" object to the planning scene.
### Functionality
-	The MoveGroupPythonInterfaceTutorial class is defined.
-	Necessary modules and packages are imported.
-	In the constructor, the moveit_commander and rospy nodes are initialized, and instances of RobotCommander, PlanningSceneInterface, and MoveGroupCommander are created.
-	The wait_for_state_update() and add_egg() methods are defined in the class.
-	The wait_for_state_update() method ensures that the collision updates are received.
-	The add_egg() method adds an object to the planning scene.
### Usage
To use the script, run it in a ROS environment with MoveIt! installed. The script adds an "egg" object to the planning scene.
## display_image_test.py
The display_image_test.py script publishes an image to a ROS (Robot Operating System) topic using the ROS Python library, rospy.
### Functionality
-	Necessary ROS message types are imported.
-	The CvBridge class from cv_bridge is used for image processing.
-	The pub_image() function initializes a ROS node and a publisher to the "/targets" topic.
-	The cv2.imread() function loads an image file from the package directory into a NumPy array, which is then converted into a ROS image message using CvBridge.
-	The while loop continuously publishes the image message at a rate of 30Hz.
### Usage
To use the script, run it in a ROS environment with rospy installed. The script can be used to test the display of images in RViz, a 3D visualization tool for ROS, or as a template for publishing images in ROS applications.
## move_group_python_interface.py
The move_group_python_interface.py script is a Python script that utilizes the MoveIt! package to plan and execute motion plans for a robot arm.
### Functionality
-	The script initializes the MoveIt! library and creates a node in ROS (Robot Operating System) using moveit_commander.roscpp_initialize().
-	The RobotCommander() class is used to obtain information about the robot's current joint states and kinematic model.
-	The PlanningSceneInterface() class is used to interact with the robot's environment.
-	The MoveGroupCommander() class is used to create a motion planning interface for a specified group of joints (in this case, the "panda_arm").
-	The DisplayTrajectory ROS publisher is used to display trajectories in RViz, a visualization tool for ROS.
-	The go_to_pose_goal() method is defined to plan and execute a motion plan to reach a desired pose (position and orientation) for the end-effector of the robot arm.
-	The current end-effector pose is obtained using the get_current_pose() method, and the orientation is preserved while updating the position component with the desired pose's position. The orientation component is set to be the same as the current orientation of the end-effector.
-	The plan() method is called to plan a motion path to the desired pose.
-	The execute() method is called to execute the planned path.
-	The all_close() method is a helper function that checks if two poses (position and orientation) are within a specified tolerance of each other.
### Usage
To use move_group_python_interface.py, follow these steps:

- Install the MoveIt! package and its dependencies.
- Copy the script to your workspace or project directory.
- Modify the script to suit your robot's joint and group names, and desired poses.
- Run the script using the Python interpreter.
## vision.py
The vision.py script is used for ArUco marker detection and pose estimation relative to the camera. ArUco markers are 2D barcodes commonly used in robotics and computer vision applications for object tracking and localization.
### Functionality
-	Imports the necessary ROS libraries and messages such as Image and PoseStamped, as well as OpenCV libraries and tools for image processing and ArUco marker detection.
-	Loads the camera matrix and distortion coefficients from a YAML file. The camera matrix contains information about the intrinsic parameters of the camera, such as focal length and principal point, while the distortion coefficients account for lens distortion.
-	Defines the ArUco dictionary and detector parameters. The ArUco dictionary is a set of predefined marker patterns, while the detector parameters specify the algorithm settings for marker detection.
-	Initializes the ROS node and publisher for publishing the pose of the ArUco marker to a ROS topic, as well as the CvBridge class for converting ROS Image messages to OpenCV Image format.
-	Defines the show_image function to display the processed image in an OpenCV window.
-	Defines the image_callback function as the callback function for the image topic. This function is called every time a new image message is received from the camera.
- In the image_callback function, the script attempts to convert the ROS Image message to a CV2 image. If successful, the script detects the ArUco marker in the image using the ArUco detector. If the marker is detected, the script estimates the pose of the marker relative to the camera using the estimatePoseSingleMarkers function.
- The script then publishes the pose of the marker to the ROS topic and displays the image with the translation and rotation vectors added. It also draws the coordinate axes on the marker to visualize its pose in 3D space.

### Usage 
To use the vision.py script, you need to have the necessary ROS libraries and messages installed, as well as OpenCV libraries and tools. You also need to have a YAML file containing the camera matrix and distortion coefficients.
