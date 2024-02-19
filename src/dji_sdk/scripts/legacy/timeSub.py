#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :17/12/2019

# license removed for brevity

import rospy
import os
from std_msgs.msg import String


def clb(msg):
    os.system("sudo date " + msg.data)


def listener(dji_name = "matrice210v2"):
	timeSub = rospy.Subscriber('android/time', String , clb)	

	if not rospy.is_shutdown(): 			
		rospy.spin()

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetsonm210v2Time', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice210v2") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
