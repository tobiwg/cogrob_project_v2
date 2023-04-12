#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image
from cv2 import aruco
from geometry_msgs.msg import PoseStamped
import numpy as np
# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError

#Load the camera matrix and distortion coefficients from YAML file
fs = cv2.FileStorage("camera_calibration.yml", cv2.FILE_STORAGE_READ)
camera_matrix = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0],[ 0.0, 0.0, 1.0] ])
dist_coeffs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])


# Define the Aruco dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
parameters = aruco.DetectorParameters_create()

# Initialize the ROS node and publisher
rospy.init_node('aruco_pose_estimation')
pub = rospy.Publisher('object_pose', PoseStamped, queue_size=1)

# Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("Hello ROS!")

# Initialize the CvBridge class
bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
	cv2.imshow("Image Window", img)
	cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg):
	# log some info about the image topic
	#rospy.loginfo(img_msg.header)

	# Try to convert the ROS Image message to a CV2 Image
	try:
		frame = bridge.imgmsg_to_cv2(img_msg, "bgr8")          
	except CvBridgeError as e:
		rospy.logerr("CvBridge Error: {0}".format(e))


	# Detect the Aruco marker
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	if ids is not None:
		# Estimate the pose of the Aruco marker
		rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
		#tvecs[0][0][0] = tvecs[0][0][0]+0.02
		#tvecs[0][0][1] = tvecs[0][0][1]+0.12
		#tvecs[0][0][2] = tvecs[0][0][2]+0.03
		
		# Publish the pose of the Aruco marker to the ROS topic
		pose_msg = PoseStamped()
		pose_msg.header.stamp = rospy.Time.now()
		pose_msg.header.frame_id = 'camera'
		pose_msg.pose.position.x = round(tvecs[0][0][0], 3)
		pose_msg.pose.position.y = round(tvecs[0][0][1], 3)
		pose_msg.pose.position.z = round(tvecs[0][0][2], 3)
		pose_msg.pose.orientation.x = round(rvecs[0][0][0], 3)
		pose_msg.pose.orientation.y = round(rvecs[0][0][1], 3)
		pose_msg.pose.orientation.z = round(rvecs[0][0][2], 3)
		pose_msg.pose.orientation.w = 1.0
		pub.publish(pose_msg)

		# Add the translation and rotation vectors to the video feed
		tvec_text = "Translation vector: ({:.2f}, {:.2f}, {:.2f})".format(tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2])
		rvec_text = "Rotation vector: ({:.2f}, {:.2f}, {:.2f})".format(rvecs[0][0][0], rvecs[0][0][1], rvecs[0][0][2])
		cv2.putText(frame, tvec_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
		cv2.putText(frame, rvec_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

		# Visualize the pose of the Aruco marker
		frame = cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvecs, tvecs, 0.1)
		#print("Translation vector:")
		#print(tvecs)
		#print("Rotation vector:")
		#print(rvecs)
	# Show the converted image
	show_image(frame)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/image", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
	rospy.spin()


