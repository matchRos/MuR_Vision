#!/usr/bin/env python
# --------------------------------------
# file:      transformation_handeye.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     transformation matrix camera_T_urtcp --- result of hand eye calibration
# --------------------------------------------------
import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf

if __name__ == '__main__':
    rospy.init_node("handeye_publisher")
    pub = rospy.Publisher("/handeye", TransformStamped, queue_size=1)
    rate = rospy.Rate(30)
    print("Publishing hand-eye transformation...")
    # the hand eye calibration is done in the case under default installation where the gripper port and flange port
    # are on the same side. However, in murc installation the TCP setting is so. Two ports are
    # are on the two sides. Therefore, x and y need to be multiplied by -1
    sign = -1 # use installation murc
    while not rospy.is_shutdown():
        # camera_T_urtcp
        transformation = np.array([[ sign*-0.0118, sign*-0.9998, -0.0177, 0.0409051],
                                   [ sign*0.9999, sign*-0.0115, -0.0120, 0.0635426],
                                   [ sign*0.0118, sign*-0.0179,  0.9998, 0.0907452],
                                   [ 0.    ,  0.    ,  0.    , 1.       ]])
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "cTe"
        msg.child_frame_id = "None"
        msg.transform.translation.x = 0.0409051
        msg.transform.translation.y = 0.0635426
        msg.transform.translation.z = 0.0907452


        rotation = tf.transformations.quaternion_from_matrix(transformation)
        msg.transform.rotation.x = rotation[0]
        msg.transform.rotation.y = rotation[1]
        msg.transform.rotation.z = rotation[2]
        msg.transform.rotation.w = rotation[3]
        pub.publish(msg)
        rate.sleep()
#
# #!/usr/bin/env python
#
# import rospy
# from geometry_msgs.msg import Transform
# import numpy as np
# import tf
#
# if __name__ == '__main__':
#     rospy.init_node("handeye_publisher")
#     pub = rospy.Publisher("/handeye", Transform, queue_size=10)
#     rate = rospy.Rate(2)
#     while not rospy.is_shutdown():
#         # camera_T_urtcp
#         transformation = np.array([[-0.0118, -0.9998, -0.0177, 0.0409051],
#                                    [ 0.9999, -0.0115, -0.0120, 0.0635426],
#                                    [ 0.0118, -0.0179,  0.9998, 0.0907452],
#                                    [ 0.    ,  0.    ,  0.    , 1.       ]])
#         msg = Transform()
#         msg.translation.x = 0.0409051
#         msg.translation.y = 0.0635426
#         msg.translation.z = 0.0907452
#
#
#         rotation = tf.transformations.quaternion_from_matrix(transformation)
#         msg.rotation.x = rotation[0]
#         msg.rotation.y = rotation[1]
#         msg.rotation.z = rotation[2]
#         msg.rotation.w = rotation[3]
#         pub.publish(msg)
#         rate.sleep()

