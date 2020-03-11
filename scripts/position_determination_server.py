#!/usr/bin/env python
# --------------------------------------
# file:      position_determination_server.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     service for calculating the position of object
# --------------------------------------------------


import rospy
from murc_robot.srv import CalculateObjectPosition
from geometry_msgs.msg import Point, PointStamped
import cv_bridge
import datetime
import cv2
# import sys
# sys.path.append('/home/blackie/catkin_ws/src/beginner_tutorials/scripts/Profilerkennung')
from profile_detection_v1_08 import *
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


def position_determinator(req):
    print("---")
    print("time: " + str(datetime.datetime.now()) + "\n")
    try:
        cimg = bridge.imgmsg_to_cv2(req.color_img, "bgr8")
        dimg = bridge.imgmsg_to_cv2(req.depth_img, "passthrough")
        aimg = bridge.imgmsg_to_cv2(req.aligned_depth_img, "passthrough")
    except cv_bridge.CvBridgeError as e:
        print(e)

    # cv2.imshow("Image 1",cimg)
    # cv2.imshow("Image 2",dimg)
    # cv2.imshow("Image 3",aimg)
    # cv2.waitKey(1)
    point, detimg, gamma, length_vector = object_detector(cimg, dimg, aimg)
    detected_img = bridge.cv2_to_imgmsg(detimg, "passthrough")
    detimg_pub.publish(detected_img)
    gamma_pub.publish(gamma)

    length_vector_stamped = PointStamped()
    length_vector_stamped.header.stamp = rospy.Time.now()
    length_vector_stamped.header.frame_id = "length_vector"
    length_vector_stamped.point.x = length_vector[0]
    length_vector_stamped.point.y = length_vector[1]
    length_vector_stamped.point.z = length_vector[2]
    object_length_direction_pub.publish(length_vector_stamped)
    # # cv2.waitKey(1)
    destination = Point()
    destination.x = point[0]
    destination.y = point[1]
    destination.z = point[2]
    x = float(point[0]*1000)
    y = float(point[1]*1000)
    z = float(point[2]*1000)
    print("\nresponse of server:\n  x: {:.4} mm\n  y: {:.4} mm\n  z: {:.4} mm".format(x,y,z))
    return destination

# def profile_detection(color_img,depth_img,aligned_depth_img):


if __name__ == '__main__':
    print("server starts...")
    bridge = cv_bridge.CvBridge()
    rospy.init_node("position_determination_server")
    service = rospy.Service("/position_determination", CalculateObjectPosition, position_determinator)
    detimg_pub = rospy.Publisher("object_img", Image, queue_size=10)
    gamma_pub = rospy.Publisher("angle_gamma", Float64, queue_size=10)
    object_length_direction_pub = rospy.Publisher("length_vector", PointStamped, queue_size=10)
    rospy.spin()
