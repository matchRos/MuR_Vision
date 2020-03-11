#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg

def talker():
	rospy.init_node('ur_test_node', anonymous=True)

	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
	rospy.sleep(0.5)
	
	ur_command_1='movej([1.57, -0.7855, -2.356, -1.047, 1.745, 0], a=0.1, v=1, t=0, r=0)'
	ur_command_2='movel(p[0.18863909945393595, 0.4505259522741935, 0.25256029492727017, -0.7290084169635158, -2.6782387225466064, 0.1389190610921433], a=0.1, v=1, t=0, r=0)'
	
	pub.publish(ur_command_1)

	break_loop=True
	
	while(break_loop):
	  rospy.sleep(0.01)
	  msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
	  break_loop=msg.is_program_running
	
	print "return to detecting point!"

	'''
	ur_command_2='movel(p[0.39471752, -0.008867, 0.1701457, -3.14152104323, 0.0002959532274488721, 0.015747549420834665], a=0.2, v=0.25, t=0, r=0)'
	
	pub.publish(ur_command_2)

	break_loop=True
	
	while(break_loop):
	  rospy.sleep(0.01)
	  msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
	  break_loop=msg.is_program_running
	
	print "Done with movel!"
	'''




	

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
