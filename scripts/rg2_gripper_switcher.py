#! /usr/bin/env python
# --------------------------------------
# file:      rg2_gripper_switcher.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     open/close the gripper
# --------------------------------------------------
import roslib
import rospy
import actionlib
from onrobot_rg2_gripper import OnRobotGripperRG2
from murc_robot.msg import RG2GripperAction, RG2GripperGoal
from std_msgs.msg import Int8

from ur_msgs.msg import IOStates


class GripperController:
    def __init__(self):
        print("GripperControl starts...")
        self.last_state = 0
        self.current_state = 0
        # self.state_sub = rospy.Subscriber('/gripping_state', Int8, self.width_control)
        self.client = actionlib.SimpleActionClient('rg2gripper', RG2GripperAction)

        self.goal = RG2GripperGoal()
        self.goal.target_width = int(100)
        self.goal.target_force = int(20)
        self.goal.payload = int(4) # important while placing skateboard
        self.goal.set_payload = True
        self.goal.depth_compensation = False
        self.goal.slave = False

        self.gripper_switch_finished = False
    
    def open_gripper(self):
        self.gripper_switch_finished = False
        result = self.width_control(100)
        if result.Success == 100:
            self.gripper_switch_finished = True
        return self.gripper_switch_finished
    
    def close_gripper(self):
        self.gripper_switch_finished = False
        result = self.width_control(0)
        if result.Success == 100:
            self.gripper_switch_finished = True
        return self.gripper_switch_finished
    
    def width_control(self, width):
        self.gripper_switch_finished = False #reset
        self.client.wait_for_server()
        self.goal.target_width = int(width)
        self.client.send_goal(self.goal)
        self.client.wait_for_result(rospy.Duration.from_sec(10.0))
        res = self.client.get_result()
        return res

def choose():
	choice='q'
	rospy.loginfo("|-----------------------------|")
	rospy.loginfo("|CHOOSE A COMMAND:")
	rospy.loginfo("|'1': open the gripper")
	rospy.loginfo("|'2': close the gripper")
	rospy.loginfo("|-----------------------------|")
	rospy.loginfo("|WHAT TO DO?")
	choice = input()
	return choice


if __name__ == '__main__':
    rospy.init_node('gripper_control_node', anonymous=True)
    choice = choose()
    try:
        gc = GripperController()
        rospy.sleep(0.5) # without 0.5s, the msg will not reach the subscriber
        if choice == 1:
            print(gc.open_gripper())
        elif choice == 2:
            print(gc.close_gripper())
    except rospy.ROSInterruptException:
        pass
