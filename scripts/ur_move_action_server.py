#!/usr/bin/env python
# --------------------------------------
# file:      ur_move_action_server.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     action server of robot arm (ur)
# --------------------------------------------------
import roslib
# roslib.load_manifest('my_pkg_name')
import rospy
import actionlib
from ur_msgs.msg import RobotModeDataMsg
from std_msgs.msg import String
from murc_robot.msg import RobotMoveAction, RobotMoveResult, RobotMoveFeedback
from geometry_msgs.msg import Point


class URMoveServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('robot_move', RobotMoveAction, self.execute, auto_start=False)
        self.server.start()
        self.move_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
        self._result = RobotMoveResult()
        self._feedback = RobotMoveFeedback()
        print("Robot movement action server starts...\nWaiting for calls of action client...")

    def execute(self, goal):
        print("---")
        print("Received Goal: {}".format(goal))
        # Do lots of awesome groundbreaking robot stuff here
        print("##Movement starts...")
        self.move_pub.publish(goal.move_command)
        self._result.movement_finished = False
        robot_is_moving = True
        while robot_is_moving:  # add a timeout watchdog
            rospy.sleep(0.1)
            msg = rospy.wait_for_message('/ur_driver/robot_mode_state', RobotModeDataMsg)
            robot_is_moving = msg.is_program_running
            # self._feedback = rospy.wait_for_message('TCP_xyz', Point)
            # self.server.publish_feedback(self._feedback)
        print("##Movement finished!")
        self._result.movement_finished = True
        print(self._result)
        self.server.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('robot_move_action_server')
    server = URMoveServer()
    rospy.spin()
