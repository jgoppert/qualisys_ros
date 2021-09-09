#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2


def callback(image_left, image_right):
    left = bridge.imgmsg_to_cv2(image_left, "bgr8")
    right = bridge.imgmsg_to_cv2(image_right, "bgr8")
    img_concat = cv2.hconcat([left, right])
    try:
        pub_img.publish(bridge.cv2_to_imgmsg(img_concat, "bgr8"))
    except CvBridgeError as e:
        print(e)

rospy.init_node('image_combine')
pub_img = rospy.Publisher('vr_image', Image, queue_size=1)
bridge = CvBridge()
rospy.loginfo('image combine starting')
image_left = message_filters.Subscriber('/virtual_cam/left/image_raw', Image)
image_right = message_filters.Subscriber('/virtual_cam/right/image_raw', Image)
ts = message_filters.TimeSynchronizer([image_left, image_right], 1)
ts.registerCallback(callback)
rospy.spin()
