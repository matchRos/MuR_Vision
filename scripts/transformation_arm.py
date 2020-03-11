#!/usr/bin/env python
# --------------------------------------
# file:      transformation_arm.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     take the transformation matrix camera_T_urtcp into transformation chain
# --------------------------------------------------
import rospy
import tf
from geometry_msgs.msg import TransformStamped, Point
import numpy as np
from murc_robot.msg import TcpPose
from RPY2RotationVector import rpy2rv


class tf_arm_node:
    def __init__(self):
        self.tl = tf.TransformListener()
        self.run_mode = True  # 0:simulation 1:real hardware
        if self.run_mode:
            self.tool = '/tool0_controller'  # for real hardware
            print("Listening the transformation of REAL HARDWARE...")
        else:
            self.tool = '/tool0'  # for simluation
            print("Listening the transformation in SIMULATION...")
        rospy.Subscriber("/handeye", TransformStamped, self.transformation_handler)
        self.bTc_pub = rospy.Publisher("transformation_bTc", TransformStamped, queue_size=10)
        self.TCP_pose_pub = rospy.Publisher("tcp_pose", TcpPose, queue_size=10)
        self.target_offset_pub = rospy.Publisher("target_offset", Point, queue_size=10)

    def transformation_handler(self, camera_T_urtcp):
        try:
            self.tl.waitForTransform('/base', self.tool, rospy.Time(), rospy.Duration(5)) # original:5s
            (trans, rot_quat) = self.tl.lookupTransform('/base', self.tool, rospy.Time(0))
        except Exception as e:
            rospy.logerr(
                'Failed during tf.TransformListener : ' + str(e) + ' \n\n Check if correct topics are being published!')

        urbase_T_urtcp = tf.transformations.quaternion_matrix(rot_quat)
        urbase_T_urtcp[0][3] = trans[0]
        urbase_T_urtcp[1][3] = trans[1]
        urbase_T_urtcp[2][3] = trans[2]

        ## 1.generate tcp pose in euler angele and publish in rotation vector [default angle-axis representation in UR]
        tcp_pose_msg = TcpPose()
        euler = tf.transformations.euler_from_quaternion(rot_quat)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        (rx, ry, rz) = rpy2rv(roll, pitch, yaw)
        tcp_pose_msg.tcp_pose = [trans[0], trans[1], trans[2], rx, ry, rz]
        # print(tcp_pose_msg.tcp_pose)
        self.TCP_pose_pub.publish(tcp_pose_msg)

        ## 2.publish x,y offset between target point and gripping point
        target_offset = Point()
        tcp_delta_vector = np.array([[0.050], [-0.015], [0.0], [0.0]]) # calculte relative displacement
        urbase_delta_vector = np.dot(urbase_T_urtcp, tcp_delta_vector)
        target_offset.x = urbase_delta_vector[0]
        target_offset.y = urbase_delta_vector[1]
        target_offset.z = urbase_delta_vector[2]
        self.target_offset_pub.publish(target_offset)

        ## 3.publish transformation matrix bTc
        translation_cTe = camera_T_urtcp.transform.translation
        rotation_cTe = camera_T_urtcp.transform.rotation
        rot_quat_cTe = [rotation_cTe.x, rotation_cTe.y, rotation_cTe.z, rotation_cTe.w]
        cTe = tf.transformations.quaternion_matrix(rot_quat_cTe)
        cTe[0][3] = translation_cTe.x
        cTe[1][3] = translation_cTe.y
        cTe[2][3] = translation_cTe.z

        # print(urbase_T_urtcp)
        # print(cTe)
        eTc = np.linalg.inv(cTe)
        bTc = np.dot(urbase_T_urtcp, eTc)

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "bTc"
        msg.child_frame_id = "None"
        msg.transform.translation.x = bTc[0][3]
        msg.transform.translation.y = bTc[1][3]
        msg.transform.translation.z = bTc[2][3]
        rotation = tf.transformations.quaternion_from_matrix(bTc)
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        self.bTc_pub.publish(msg)
        # print(bTc)


if __name__ == '__main__':

    rospy.init_node("transformation_arm_node")
    tf_arm_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop calculating the transformation matrixs")
