#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import copy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospkg
import tf
import math

# TODO(lucasw) these no longer exist, demo.launch supersedes this test
# is there any reason to keep this around?
# from rviz_textured_quads.msg import TexturedQuad, TexturedQuadArray

bridge = CvBridge()
def pub_image():

    rospy.init_node('rviz_display_image_test', anonymous=True)
    rospack = rospkg.RosPack()

    image_pub = rospy.Publisher("/targets", Image, queue_size=10)

    rospack = rospkg.RosPack()
    image_path = rospack.get_path('startup') + '/scripts/'
    img1 = cv2.imread(image_path +'textures/6x6_1000-0.jpg', cv2.IMREAD_COLOR)
    img_msg1 = bridge.cv2_to_imgmsg(img1, encoding="bgr8")

    

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        image_pub.publish(img_msg1)
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass
