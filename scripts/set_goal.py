#!/usr/bin/env python
# --------------------------------------
# file:      set_goal.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     calculates following goals:
#            goal2catch: the goal for robot arm to grip the object
#            length_vector2catch: length vector in ur_base coordinate system for gripping of profile's second end
#            goal2approach: the goal for mobile platform to approach first end of profile
#            place4skateboard: the goal to place the skateboard
# --------------------------------------------------
import rospy
import message_filters
from geometry_msgs.msg import Point, PointStamped, Transform, TransformStamped, Pose
import numpy as np
import tf
import math


def matrix_from_t_and_eulerangles():
    # transformation matrix from base MiR to base UR
    t = [0.2443, -0.1402, 0.4505]
    RPY = [0, 0, math.pi - 0.01549]
    matrix = tf.transformations.euler_matrix(RPY[0], RPY[1], RPY[2])
    matrix[0][3] = t[0]
    matrix[1][3] = t[1]
    matrix[2][3] = t[2]
    return matrix


class GoalTransfer:
    def __init__(self):
        self.point = Point()
        self.bTc = Transform()
        # coordinates of object in camera coordinate system
        self.objectCSc_sub = message_filters.Subscriber("/object_position", PointStamped)
        # transformation matrix from ur base to camera
        self.bTc_sub = message_filters.Subscriber("/transformation_bTc", TransformStamped)
        self.object_length_direction_sub = message_filters.Subscriber("/length_vector", PointStamped)
        ts = message_filters.ApproximateTimeSynchronizer([self.objectCSc_sub, self.bTc_sub,
                                                          self.object_length_direction_sub], 10, 0.1)
        ts.registerCallback(self.callback_move2goal)
        # coordinates of object in ur_base coordinate system
        self.goal2catch = Point()
        # service to execute motion ------following is temporary to test
        self.goal2catch_pub = rospy.Publisher("goal2catch", Point, queue_size=1)
        # length vector in ur_base coordinate system for gripping of profile's back end 
        self.goal2catch_lengthvector = Point()
        self.goal2catch_lv_pub = rospy.Publisher("length_vector2catch", Point, queue_size=1)
        
        print("Calculating the goal point coordinates in ur_base CS...")
        # color_T_depth   c70 - d75
        self.cTd_matrix = np.array([[0.999976, 0.000364764, 0.00694771, 0.0147239],
                                    [-0.000363598, 1, -0.000169137, 0.000300613],
                                    [-0.00694777, 0.000166607, 0.999976, 0.000210366],
                                    [0., 0., 0., 1.]])
        self.goal2approach_pub = rospy.Publisher("goal2approach", Pose, queue_size=1)
        self.goal2approach = Pose()
        self.place4skateboard_pub = rospy.Publisher("place4skateboard", Pose, queue_size=1)
        self.place4skateboard = Pose()
        self.baseMiR_T_baseUR = matrix_from_t_and_eulerangles()
        self.tl = tf.TransformListener()

    def callback_move2goal(self, point_stamped, bTc_stamped, length_vector_stamped):
        try:
            self.tl.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(5)) # original:5s
            (trans, rot_quat) = self.tl.lookupTransform('/map', '/base_link', rospy.Time(0))
        except Exception as e:
            rospy.logerr(
                'Failed during tf.TransformListener : ' + str(e) + ' \n\n Check if correct topics are being published!')
        map_T_baseMiR = tf.transformations.quaternion_matrix(rot_quat)
        map_T_baseMiR[0][3] = trans[0]
        map_T_baseMiR[1][3] = trans[1]
        map_T_baseMiR[2][3] = trans[2]

        self.point = point_stamped.point
        self.bTc = bTc_stamped.transform
        if self.point.z > 0:
            # print("z>0, z = {}".format(self.point.z))
            p_CSd_vector = np.array([[self.point.x], [self.point.y], [self.point.z], [1]])
        else:  # No depth information z = -1 m
            p_CSd_vector = np.array([[0], [0], [0], [0]])
            # print("z<0, z = {}".format(self.point.z))
        p_CSc_vector = np.dot(self.cTd_matrix, p_CSd_vector)

        translation_bTc = bTc_stamped.transform.translation
        rotation_bTc = bTc_stamped.transform.rotation
        rot_quat_bTc = [rotation_bTc.x, rotation_bTc.y, rotation_bTc.z, rotation_bTc.w]
        bTc_matrix = tf.transformations.quaternion_matrix(rot_quat_bTc)
        bTc_matrix[0][3] = translation_bTc.x
        bTc_matrix[1][3] = translation_bTc.y
        bTc_matrix[2][3] = translation_bTc.z

        p_CSb_vector = np.dot(bTc_matrix, p_CSc_vector)
        self.goal2catch.x = p_CSb_vector[0]
        self.goal2catch.y = p_CSb_vector[1]
        self.goal2catch.z = p_CSb_vector[2]
        self.goal2catch_pub.publish(self.goal2catch)
        p_baseUR_vector = p_CSb_vector

        # calculation of the pose to go for MiR, including position and orientation
        p_baseMiR_vector = np.dot(self.baseMiR_T_baseUR, p_baseUR_vector)
        # calculte relative displacement from baseMiR to the object
        # -0.4m and 0.5m are the offsets for sake of reseve space to graspe the object
        displacemnet_vector_CS_baseMiR = np.array([[p_baseMiR_vector[0]-0.2443], [p_baseMiR_vector[1]+0.5402+0.1], [p_baseMiR_vector[2]], [1]])
        displacemnet_vector_CS_map = np.dot(map_T_baseMiR, displacemnet_vector_CS_baseMiR)
        self.goal2approach.position.x = displacemnet_vector_CS_map[0] # + 0.0
        self.goal2approach.position.y = displacemnet_vector_CS_map[1] # + 0.5
        self.goal2approach.position.z = displacemnet_vector_CS_map[2] # ought to be 0.0

        # transformation of vector !! not point                                    ------------>0
        length_vector = length_vector_stamped.point
        length_vector_CSd = np.array([[length_vector.x], [length_vector.y], [length_vector.z], [0]])
        length_vector_CSc = np.dot(self.cTd_matrix, length_vector_CSd)
        length_vector_CSbUR = np.dot(bTc_matrix, length_vector_CSc)
        length_vector_CSbMiR = np.dot(self.baseMiR_T_baseUR, length_vector_CSbUR)  # y axis of object cs
        # object coordinate system
        y_axis = length_vector_CSbMiR[0:3] 
        y_axis = y_axis/np.linalg.norm(y_axis) # normailization
        z_axis = np.array([[0], [0], [1]])
        x_axis = np.cross(y_axis, z_axis, axis=0)
        # orientation of MiR to go
        orientation_x = y_axis
        orientation_y = -x_axis
        orientation_z = z_axis
        # T is the homogenous matrix of roataion matrix of object CS (rotated like gripper CS) with respect to baseUR CS 
        bMiR_T_o = np.zeros((4, 4))
        bMiR_T_o[0:3, 0] = y_axis.reshape(3)
        bMiR_T_o[0:3, 1] = x_axis.reshape(3)
        bMiR_T_o[0:3, 2] = -z_axis.reshape(3)
        bMiR_T_o[3, 3] = 1
        bUR_T_o= np.dot(np.linalg.inv(self.baseMiR_T_baseUR), bMiR_T_o)
        # T is the homogenous matrix of roataion matrix of object CS (rotated like baseMiR CS) with respect to baseMiR CS
        # due to the quaternion_from_matrix needs a 4x4 matrix
        bMiR_T = np.zeros((4, 4))
        bMiR_T[0:3, 0] = orientation_x.reshape(3)
        bMiR_T[0:3, 1] = orientation_y.reshape(3)
        bMiR_T[0:3, 2] = orientation_z.reshape(3)
        bMiR_T[3, 3] = 1
        # print("bMiR_T:")
        # print(bMiR_T)
        map_T = np.dot(map_T_baseMiR, bMiR_T)
        orientation = tf.transformations.quaternion_from_matrix(map_T)
        self.goal2approach.orientation.x = orientation[0]  # ought to be 0.0
        self.goal2approach.orientation.y = orientation[1]  # ought to be 0.0
        self.goal2approach.orientation.z = orientation[2]
        self.goal2approach.orientation.w = orientation[3]

        self.goal2approach_pub.publish(self.goal2approach)

        self.goal2catch_lengthvector.x = length_vector_CSbUR[0]
        self.goal2catch_lengthvector.y = length_vector_CSbUR[1]
        self.goal2catch_lengthvector.z = length_vector_CSbUR[2]
        self.goal2catch_lv_pub.publish(self.goal2catch_lengthvector)

        self.place4skateboard.position.x = p_CSb_vector[0]
        self.place4skateboard.position.y = p_CSb_vector[1]
        self.place4skateboard.position.z = p_CSb_vector[2]
        print("bUR_T_o:")
        print(bUR_T_o)
        bUR_orientation = tf.transformations.quaternion_from_matrix(bUR_T_o)
        self.place4skateboard.orientation.x = bUR_orientation[0]  # ought to be 0.0
        self.place4skateboard.orientation.y = bUR_orientation[1]  # ought to be 0.0
        self.place4skateboard.orientation.z = bUR_orientation[2]
        self.place4skateboard.orientation.w = bUR_orientation[3]
        self.place4skateboard_pub.publish(self.place4skateboard)


if __name__ == '__main__':
    rospy.init_node("object_position_calculator")
    GoalTransfer()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop transferring goal to the MiR robot!")
