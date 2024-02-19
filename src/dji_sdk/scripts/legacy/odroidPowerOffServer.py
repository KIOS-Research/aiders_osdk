#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :20/06/2019

# license removed for brevity
import rospy
import traceback
import sys
from termcolor import colored
import re
#import tf
from multiprocessing import Process
from dji_sdk.srv import OdroidPowerOff
from dji_sdk.msg import MissionHotpointTask
#import subprocess
import os
	

def odroid_power_off_callback(req):
	#subprocess.call(["shutdown ", "'now'"])
    os.system("shutdown now")
    return True, 0, 0, 0

def listener(dji_name = "matrice300"):
	service = rospy.Service('matrice300/odroid_power_off', OdroidPowerOff, odroid_power_off_callback)
	
	rateHz = 1
	rate = rospy.Rate(rateHz)  
	rate.sleep()

	if not rospy.is_shutdown(): 			
		rospy.spin()


		

'''
Main Function
'''
if __name__ == '__main__':
	try:
		rospy.init_node('jetson300Power', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice300") 
		listener(dji_name = private_param)
	except rospy.ROSInterruptException: 
		pass
