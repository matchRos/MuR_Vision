#!/usr/bin/env python
# --------------------------------------
# file:      RG2Gripper_action_server.py
# author:    Florian
# date:      2020-01-06
# brief:     action server of the gripper
# --------------------------------------------------
import roslib
import rospy
import urx
import actionlib
from std_msgs.msg import String
from onrobot_rg2_gripper import OnRobotGripperRG2
from murc_robot.msg import RG2GripperAction, RG2GripperResult, RG2GripperFeedback
from ur_msgs.msg import IOStates

class RG2GripperActionServer:
	def __init__(self):
		self.server = actionlib.SimpleActionServer('rg2gripper', RG2GripperAction, self.execute, False)
		self.server.start()
		self._result = RG2GripperResult()
		self._feedback = RG2GripperFeedback()
		self.initialization_success = False
		print("RG2Gripper action server started. \nWaiting for calls from client...")
		#Load URX Driver and initialize RG2-Conncetion
		
		self.robot_ip=rospy.get_param("/ur_driver/robot_ip_address")
		self.rob = urx.Robot(self.robot_ip)
		self.gripper = OnRobotGripperRG2(self.rob)

	def execute(self, goal):
		print("---")
		print("Received Goal: {}".format(goal))

		print("Gripper acts...")
		self._feedback.Progress = int(1)
		self.server.publish_feedback(self._feedback)
		# #Load URX Driver and initialize RG2-Conncetion
		# try:
		# 	robot_ip=rospy.get_param("/ur_driver/robot_ip_address")
		# 	rob = urx.Robot(robot_ip)
		# 	gripper = OnRobotGripperRG2(rob)
		# 	self._feedback.Progress = int(23)
		# 	self.server.publish_feedback(self._feedback)
		# 	self.initialzation_success = True
		# except Exception as e:
		# 	self.intialization_success = False
		# 	print("Failed to Load urx_driver")
		# 	rospy.loginfo("Initialization of RG2 failed: " + str(e))
		# 	self.server.set_aborted()

		# if self.initialization_success:
		try:
			self.gripper.open_gripper(target_width=goal.target_width)
			self._feedback.Progress = int(50)
			self.server.publish_feedback(self._feedback)
		except Exception as e:
			print("Failed to send gripping command to robot arm")
			rospy.loginfo("Action of gripper failed: " + str(e))

		rospy.sleep(0.2)

		is_grippper_waiting = False
		while is_grippper_waiting == False:
			rospy.sleep(0.1)
			io_states = rospy.wait_for_message('/ur_driver/io_states',IOStates)
			is_grippper_waiting = io_states.digital_out_states[16].state
			print(is_grippper_waiting)

			# gripping_contact_force= io_states.digital_out_states[17].state
			# if gripping_contact_force==True:
			# 	# Gripper contact: setting success as result
			# 	rob.close()
			# 	self._feedback.Progress = int(100)
			# 	print "Contact was made. Gripper Done"
			# 	self._result.Success=int(100)
			# 	self.server.set_succeeded(self._result)
			#
			# 	break

		# print("Grip completed")
		# '''
		# if io_states.digital_out_states[17].state ==False:
		# 	print "No Contact Force detected."
		# '''

		# if self.initialization_success:
		# 	rob.close()
		# 	self.initialization_succes = False
		self._feedback.Progress = int(100)
		self.server.publish_feedback(self._feedback)

		
		self._result.Success=int(100)
		self.server.set_succeeded(self._result)
		print("Opening/Closing of gripper is Done")

if __name__ == '__main__':
	rospy.init_node('rg2gripper_action_server')
	server = RG2GripperActionServer()
	rospy.spin()
