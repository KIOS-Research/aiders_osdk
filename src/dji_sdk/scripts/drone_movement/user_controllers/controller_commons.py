
# import rospy
# import threading
# import time
# import math
# import datetime

# from std_msgs.msg import String, Bool, Header
# from dji_sdk.msg import ScanArea, MissionWaypoint, JoystickParams
# from kios.msg import Telemetry, MissionDji, GpsInput, MissionCommandDJI
# from dji_sdk.srv import SetJoystickMode, JoystickAction, ObtainControlAuthority
# from dji_sdk.srv import FlightTaskControl
# from sensor_msgs.msg import Joy
# from geometry_msgs.msg import Vector3Stamped, Vector3
# from math import sin, cos, atan2, sqrt



# def setupJoystickMode(joystick_enable):
#     global joystickModeSrv

#     if joystick_enable:
#         yaw_mode = 1
#     else:
#         yaw_mode = 0

#     horizontal_mode = 1
#     vertical_mode = 0
#     #yaw_mode = joystick_enable
#     hoorizontal_coordinate = 1
#     stable_mode = 1

#     response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)

#     print('Set Joystick Mode Response:', response)

# def publishAction(x, y, z, yaw):
#     global JoystickActionPub

#     actionParams = JoystickParams()
#     actionParams.x = x
#     actionParams.y = y
#     actionParams.z = z
#     actionParams.yaw = yaw
    

#     #print(x, y, z, yaw)    
#     #response = joystickActionSrv(actionParams)
    
#     JoystickActionPub.publish(actionParams)


# def init():
#     global joystickModeSrv, ControlAuthorityPub, JoystickActionPub

#     if rospy.is_shutdown():
#         print(nodename,'- init()','- rospy is not ready')
#     else:
#         rospy.wait_for_service(dronename+'/set_joystick_mode')
#         joystickModeSrv=rospy.ServiceProxy(dronename + '/set_joystick_mode', SetJoystickMode)

#         ControlAuthorityPub = rospy.Publisher(dronename+'/ControlAuthority/Set_State_Enable', Bool, queue_size=1)
#         JoystickActionPub = rospy.Publisher(dronename+'/Joystick/PublishJoystickAction', JoystickParams, queue_size=1)

#         print(nodename,'- init()','- DONE')


# def listener(dji_name = "/matrice300", as_module = False):
#     global dronename, nodename

#     dronename = dji_name
#     nodename = dronename.replace('/', '') + '_controller_commons'
#     #print(nodename)

#     if not as_module:
#         rospy.init_node(nodename)
#         init()
#     else:
#         print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

#     if not as_module:
#         rospy.spin()


# print('Initializing Controller Commons...')
# f = open("/var/lib/dbus/machine-id", "r")
# boardId = f.read().strip()[0:4]

# dronename = '/matrice300' + '_' + boardId

# if __name__ == '__main__':
#     as_module = False
# else:
#     as_module = True

# listener(dji_name=dronename, as_module=as_module)