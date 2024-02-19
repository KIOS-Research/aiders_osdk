import rospy

from std_msgs.msg import Bool, Header, Time, String

from dji_sdk.msg import Resolution
from dji_sdk.srv import SetupCameraStream, SetupCameraH264, CameraStartShootSinglePhoto, GimbalAction


print('Initializing Camera Handler...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = 'matrice300' + '_' + boardId
gimbalSrv=rospy.ServiceProxy(dronename+'/gimbal_task_control', GimbalAction)


def resetAngleCamera():
    gimbalCmd = GimbalAction()
    gimbalCmdHeader = Header()
    gimbalCmdHeader.frame_id = 'body_FLU'

    gimbalCmd.header = gimbalCmdHeader


    gimbalCmd.is_reset = True
    gimbalCmd.payload_index = 0

    gimbalCmd.rotationMode = 1

    gimbalCmd.pitch = 0.0
    gimbalCmd.roll = 0.0
    gimbalCmd.yaw = 0.0

    gimbalCmd.time = 0.5

    #print('header', gimbalCmd.header)
    #print('is_reset', gimbalCmd.is_reset)
    #print('payload_index', gimbalCmd.payload_index)
    #print('rotationMode', gimbalCmd.rotationMode)

    #print('pitch', gimbalCmd.pitch)
    #print('roll', gimbalCmd.roll)
    #print('yaw', gimbalCmd.yaw)
    #print('time', gimbalCmd.time)
    
    rospy.wait_for_service(dronename+'/gimbal_task_control')
    response = gimbalSrv(gimbalCmd.header, gimbalCmd.is_reset, gimbalCmd.payload_index, gimbalCmd.rotationMode,
                         gimbalCmd.pitch, gimbalCmd.roll, gimbalCmd.yaw, gimbalCmd.time)
    
    print('Reset Camera Angle Response:', response)


def angleCameraOrthophotography():
    # args should be ['header', 'is_reset', 'payload_index', 'rotationMode', 'pitch', 'roll', 'yaw', 'time']
    print('Angling camera downwards')
    

    gimbalCmd = GimbalAction()
    gimbalCmdHeader = Header()
    gimbalCmdHeader.frame_id = 'body_FLU'

    gimbalCmd.header = gimbalCmdHeader

    gimbalCmd.is_reset = False
    gimbalCmd.payload_index = 0

    gimbalCmd.rotationMode = 0

    gimbalCmd.pitch = -90.0
    gimbalCmd.roll = 0.0
    gimbalCmd.yaw = 0.0

    gimbalCmd.time = 0.5

    print('header', gimbalCmd.header)
    print('is_reset', gimbalCmd.is_reset)
    print('payload_index', gimbalCmd.payload_index)
    print('rotationMode', gimbalCmd.rotationMode)

    print('pitch', gimbalCmd.pitch)
    print('roll', gimbalCmd.roll)
    print('yaw', gimbalCmd.yaw)
    print('time', gimbalCmd.time)

    print(gimbalCmd)
    
    rospy.wait_for_service(dronename+'/gimbal_task_control')
    response = gimbalSrv(gimbalCmd.header, gimbalCmd.is_reset, gimbalCmd.payload_index, gimbalCmd.rotationMode,
                         gimbalCmd.pitch, gimbalCmd.roll, gimbalCmd.yaw, gimbalCmd.time)
    
    print('Angled Camera Response:', response)
    print('Camera Setup READY')


def angle_gimbal_smoothly(roll, pitch, yaw):
    # args should be ['header', 'is_reset', 'payload_index', 'rotationMode', 'pitch', 'roll', 'yaw', 'time']
    print('Angling Gimbal')
    

    gimbalCmd = GimbalAction()
    gimbalCmdHeader = Header()
    gimbalCmdHeader.frame_id = 'body_FLU'

    gimbalCmd.header = gimbalCmdHeader

    gimbalCmd.is_reset = False
    gimbalCmd.payload_index = 0

    gimbalCmd.rotationMode = 1

    gimbalCmd.pitch = pitch
    gimbalCmd.roll = roll
    gimbalCmd.yaw = yaw

    gimbalCmd.time = 0.5

    print('header', gimbalCmd.header)
    print('is_reset', gimbalCmd.is_reset)
    print('payload_index', gimbalCmd.payload_index)
    print('rotationMode', gimbalCmd.rotationMode)

    print('pitch', gimbalCmd.pitch)
    print('roll', gimbalCmd.roll)
    print('yaw', gimbalCmd.yaw)
    print('time', gimbalCmd.time)

    print(gimbalCmd)
    
    rospy.wait_for_service(dronename+'/gimbal_task_control')
    response = gimbalSrv(gimbalCmd.header, gimbalCmd.is_reset, gimbalCmd.payload_index, gimbalCmd.rotationMode,
                         gimbalCmd.pitch, gimbalCmd.roll, gimbalCmd.yaw, gimbalCmd.time)
    
    print('Angled Camera Response:', response)
    print('Camera Setup READY')