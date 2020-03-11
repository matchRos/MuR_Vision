#!/usr/bin/env python
# --------------------------------------
# file:      position_publisher.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     invoke the service to calculate the position of object
#            and publish the results
# --------------------------------------------------

import rospy
from sensor_msgs.msg import Image
import cv2
import message_filters
from murc_robot.srv import CalculateObjectPosition
from geometry_msgs.msg import Point,PointStamped


class ImagesTransfer:

    def __init__(self):
        self.color_img = Image()
        self.depth_img = Image()
        self.aligned_depth_img = Image()

        rospy.wait_for_service("/position_determination")
        try:
            self.position_server = rospy.ServiceProxy("/position_determination", CalculateObjectPosition)
            print("Invoking service...")
        except rospy.ServiceException as e:
            rospy.loginfo("Service failed: " + str(e))

        self.color_sub = message_filters.Subscriber("camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber("camera/depth/image_rect_raw", Image)
        self.aligned_depth_sub = message_filters.Subscriber("camera/aligned_depth_to_color/image_raw", Image)
        print("Getting images... ")
        ts = message_filters.TimeSynchronizer([self.color_sub, self.depth_sub, self.aligned_depth_sub], queue_size=10)
        ts.registerCallback(self.callback)
        self.position_pub = rospy.Publisher("object_position", PointStamped, queue_size=10)
        print("Publishing object coordinates in camera cs...")

    def callback(self, img1, img2, img3):

        global bridge
        # print("callback succeed")
        self.color_img = img1
        self.depth_img = img2
        self.aligned_depth_img = img3
        response = self.position_server(self.color_img, self.depth_img, self.aligned_depth_img, 0.00)
        point = PointStamped()
        point.header.stamp = rospy.Time.now()
        point.header.frame_id = "p_(CS)c"
        point.point = response.destination_point
        # print("response is {}".format(response.destination_point))
        self.position_pub.publish(point)
        # cv2.waitKey(1)

    # def color_callback(self, ros_img):
    #     self.color_img = ros_img
    #
    # def depth_callback(self, ros_img):
    #     self.depth_img = ros_img
    #
    # def aligned_depth_callback(self, ros_img):
    #     self.aligned_depth_img = ros_img
    #     response = self.position_server(self.color_img, self.depth_img, self.aligned_depth_img, 0.00)
    #     point = response.destination_point
    #     # print("response is {}".format(response.destination_point))
    #     self.position_pub.publish(point)


if __name__ == '__main__':

    rospy.init_node("position_publisher")  # , anonymous=True
    ImagesTransfer()
    try:
        rospy.spin()
    except KeyboardInterrupt or cv2.waitKey(1) & 0xFF == ord('q'):
        print("Stop capturing images from live stream!")
