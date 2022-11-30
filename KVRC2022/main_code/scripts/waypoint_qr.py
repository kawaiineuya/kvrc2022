#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar
import cv2
import matplotlib.pyplot as plt

def get_image (image_message):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')
    cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    cv_image_decoded = pyzbar.decode(cv_image_gray)
    for d in cv_image_decoded:
        print(d.data.decode("utf-8"))


rospy.init_node('waypoint_qr')

sub = rospy.Subscriber('/d435i/depth/rgb_image_raw', Image, get_image)

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()

