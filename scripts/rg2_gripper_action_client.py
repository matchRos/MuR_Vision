#! /usr/bin/env python
# --------------------------------------
# file:      rg2_gripper_action_client.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     action client of gripper
# --------------------------------------------------
import roslib
import rospy
import actionlib
from onrobot_rg2_gripper import OnRobotGripperRG2
from murc_robot.msg import RG2GripperAction, RG2GripperGoal
from std_msgs.msg import Int8

from ur_msgs.msg import IOStates


class GripperControl:
    def __init__(self):
        print("GripperControl starts...")
        self.last_state = 0
        self.current_state = 0
        self.state_sub = rospy.Subscriber('/gripping_state', Int8, self.width_control)
        self.client = actionlib.SimpleActionClient('rg2gripper', RG2GripperAction)

        self.goal = RG2GripperGoal()
        self.goal.target_width = int(100)
        self.goal.target_force = int(20)
        self.goal.payload = int(20)
        self.goal.set_payload = False
        self.goal.depth_compensation = False
        self.goal.slave = False

    def width_control(self, gripping_state_msg):
        state = gripping_state_msg.data
        self.current_state = state
        if not self.current_state == self.last_state:
            self.client.wait_for_server()

            if state == 4:
                self.goal.target_width = int(0)
            else:
                self.goal.target_width = int(100)
            self.client.send_goal(self.goal)
            self.client.wait_for_result(rospy.Duration.from_sec(10.0))
            res = self.client.get_result()
            print(res)
            self.last_state = state
            if state == 4:
                print("##Gripping finished.")


if __name__ == '__main__':
    rospy.init_node('rg2_gripper_action_automatic_control')
    GripperControl()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Stop controlling the gripper!")

    # try:
    #     client = actionlib.SimpleActionClient('rg2gripper', RG2GripperAction)
    #     client.wait_for_server()
    #     print "Client is connected to  server"
    #     width=raw_input("Width: ")
    #
    #     goal = RG2GripperGoal()
    #     goal.target_width=int(width)
    #     goal.target_force=int(20)
    #     goal.payload=int(20)
    #     goal.set_payload=False
    #     goal.depth_compensation=False
    #     goal.slave=False
    #
    #
    #     # Fill in the goal here
    #
    #     client.send_goal(goal)
    #     client.wait_for_result(rospy.Duration.from_sec(10.0))
    #
    #     res= client.get_result()
    #     print res
    # except:
    #     print "Could not connect to Action server"
