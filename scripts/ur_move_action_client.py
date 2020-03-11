#!/usr/bin/env python
# --------------------------------------
# file:      ur_move_action_client.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     action client of robot arm (ur)
# --------------------------------------------------
import roslib
import rospy
import actionlib

from murc_robot.msg import RobotMoveAction, RobotMoveGoal

if __name__ == '__main__':
    rospy.init_node('robot_move_action_client')
    client = actionlib.SimpleActionClient('robot_move', RobotMoveAction)
    client.wait_for_server()

    goal = RobotMoveGoal()
    # Fill in the goal here
    # move in joint-space:
    # ur_command_1='movej( [1.2573, -1.3248, -1.9772, -1.4116, 1.5742, 1.8150], a=0.1, v=0.2, t=0, r=0)'
    # move in task-space:
    # ur_command_2='movel(p[0.3947, -0.0088,  0.1701, -3.1415, 0.0002, 0.0157], a=0.2, v=0.05, t=0, r=0)'
    ur_command_1 = 'movej([1.57, -0.7855, -2.356, -1.047, 1.745, 0], a=0.1, v=1, t=0, r=0)'
    # ur_command_2 = 'movel(p[0.18863909945393595, 0.4505259522741935, 0.25256029492727017, -0.7290084169635158, -2.6782387225466064, 0.1389190610921433], a=0.1, v=1, t=0, r=0)'
    ur_home = 'movej([0.04593, -0.8294, -2.1438, -1.7402, 1.5759, -1.5227], a=0.1, v=1, t=0, r=0)'
    goal.move_command = ur_home
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
