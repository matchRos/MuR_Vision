#!/usr/bin/env python
# --------------------------------------
# file:      ur_pose_maker.py
# author:    Guannan Cen
# date:      2020-01-06
# brief:     change pose of the ur in joint space using movej
# --------------------------------------------------
import rospy
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg

class ur_pose_maker:
	def __init__(self):
		self.pose0_home = 'movej([0.0461, -0.8294, -2.1438, -1.7401, 1.5759, -1.5233], a=0.2, v=1, t=0, r=0)'
		self.pose1_searching ='movej([3.1416, -0.7855, -2.3562, -0.5236, 1.5708, -1.5708], a=0.2, v=1, t=0, r=0)'
		self.pose2_grasping_preparation ='movej([1.57, -1.0472, -2.0944, -1.3963, 1.745, 3.1416], a=0.2, v=1, t=0, r=0)'

		self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
		self.pose_finished = False
	
	def make_pose(self, pose_id):
		if pose_id == 0:
			self.pub.publish(self.pose0_home)
		elif pose_id == 1:
			self.pub.publish(self.pose1_searching)
		elif pose_id == 2:
			self.pub.publish(self.pose2_grasping_preparation)
		else:
			rospy.logerr("No such pose!")

		while not self.pose_finished:
			rospy.sleep(0.1)
			msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
			self.pose_finished= not msg.is_program_running
		return self.pose_finished
		
	
# def ur_pose_maker(pose_id):
# 	rospy.init_node('ur_test_node', anonymous=True)

# 	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
# 	rospy.sleep(0.5)
# 	pose0_home = 'movej([0.0461, -0.8294, -2.1438, -1.7401, 1.5759, -1.5233], a=0.2, v=1, t=0, r=0)'
# 	pose1_searching='movej([3.1416, -0.7855, -2.3562, -0.5236, 1.5708, -1.5708], a=0.2, v=1, t=0, r=0)'
# 	pose2_grasping_preparation   ='movej([1.57, -0.7855, -2.356, -1.3963, 1.745, 0], a=0.2, v=1, t=0, r=0)'

# 	ur_command_2='movel(p[0.18863909945393595, 0.4505259522741935, 0.25256029492727017, -0.7290084169635158, -2.6782387225466064, 0.1389190610921433], a=0.1, v=1, t=0, r=0)'
	
# 	pub.publish(pose0_home)

# 	break_loop=True
	
# 	while(break_loop):
# 	  rospy.sleep(0.01)
# 	  msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
# 	  break_loop=msg.is_program_running
	
# 	print "return to detecting point!"

def choose():

	choice='q'

	rospy.loginfo("|-----------------------------|")
	rospy.loginfo("|CHOOSE A POSE:")
	rospy.loginfo("|'0': pose0_home ")
	rospy.loginfo("|'1': pose1_searching")
	rospy.loginfo("|'2': pose2_grasping_preparation")
	rospy.loginfo("|-----------------------------|")
	rospy.loginfo("|WHERE TO GO?")
	choice = input()
	return choice	

if __name__ == '__main__':
	rospy.init_node('ur_pose_maker_node', anonymous=True)
	choice = choose()
	try:
		upm = ur_pose_maker()
		rospy.sleep(0.5) # without 0.5s, the msg will not reach the subscriber
		if choice == 0:
			print(upm.make_pose(0))
		elif choice == 1:
			print(upm.make_pose(1))
		elif choice == 2:
			print(upm.make_pose(2))
	except rospy.ROSInterruptException:
		pass
