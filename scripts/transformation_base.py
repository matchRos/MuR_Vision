#!/usr/bin/env python
# --------------------------------------
# file:      transformation_base.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     example to obtain a transformation matrix from frames
# --------------------------------------------------
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_base_listener')

    listener = tf.TransformListener()

    # rospy.wait_for_service('spawn')
    # spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    # spawner(4, 2, 0, 'turtle2')

    # turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        baseMiR_T_baseUR = tf.transformations.quaternion_matrix(rot)
        baseMiR_T_baseUR[0][3] = trans[0]
        baseMiR_T_baseUR[1][3] = trans[1]
        baseMiR_T_baseUR[2][3] = trans[2]
        print("trans:"+str(trans))
        print("---")
        print(baseMiR_T_baseUR)

        rate.sleep()