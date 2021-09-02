#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

pub_img = rospy.Publisher('vr_image', Image, queue_size=1)
bridge = CvBridge()


def callback(image_left, image_right):
    rospy.loginfo('callback')
    left = bridge.imgmsg_to_cv2(image_left, "bgr8")
    right = bridge.imgmsg_to_cv2(image_right, "bgr8")
    img_concat = cv2.hconcat([left, right])
    pub_img.publish(bridge.cv2_to_imgmsg(img_concat, "bgr8"))


rospy.init_node('image_combine')
rospy.loginfo('starting')
image_left = message_filters.Subscriber('/virtual_cam/usb_cam/image_raw', Image)
image_right = message_filters.Subscriber('/virtual_cam/usb_cam/image_raw', Image)
ts = message_filters.TimeSynchronizer([image_left, image_right], 1)
ts.registerCallback(callback)
rospy.spin()
