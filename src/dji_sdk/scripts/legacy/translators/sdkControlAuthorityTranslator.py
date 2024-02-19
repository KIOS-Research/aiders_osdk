#!/usr/bin/env python

# Author: Andreas
# Email: anastasiou.antreas@ucy.ac.cy
# Date :26/03/2021

# license removed for brevity


import sys
from dji_sdk.srv import ObtainControlAuthority, SDKControlAuthority
import rospy
import random
import math

dji_name = "matrice300"

def authorityActionCallback(req):
	global dji_name

	try:
		obtnAuth = rospy.ServiceProxy(dji_name + '/obtain_release_control_authority', ObtainControlAuthority)
		
		if (req.control_enable == req.RELEASE_CONTROL):
			resp = obtnAuth(False)
		elif (req.control_enable == req.REQUEST_CONTROL):
			resp = obtnAuth(True)
		print(resp.result)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e
	finally:
		return resp.result, 0, 0, 0


def listener():
	global dji_name

	rospy.wait_for_service(dji_name + '/obtain_release_control_authority') 
	service = rospy.Service(dji_name+'/sdk_control_authority', SDKControlAuthority, authorityActionCallback)
	
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
		rospy.init_node('jetson300AuthorityActionControl', anonymous=False)
		listener()
	except rospy.ROSInterruptException:
		pass

