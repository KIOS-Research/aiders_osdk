import rospy
import threading

from std_msgs.msg import Bool
from dji_sdk.srv import ObtainControlAuthority, SetJoystickMode


control_authority_mutex = threading.Lock()


#Type: dji_sdk/ObtainControlAuthority
#Args: enable_obtain
haveControlAuthority = False
def obtainControlAuthority(enable_obtain):
    print('obtainControlAuthority')
    global control_authority_mutex, haveControlAuthority
    global controlAuthoritySrv

    if control_authority_mutex.locked():
        print('control_authority_mutex is locked by another thread. Will not obtainControlAuthority')
        return False
    else:
        control_authority_mutex.acquire()

    enable_obtain = enable_obtain.data
    print('obtainControlAuthority called', 'enable_obtain:', enable_obtain)  
    response = controlAuthoritySrv(enable_obtain)

    print('Desired Control Authority:', enable_obtain, 'controlAuthoritySrv response:', response.result)

    if response.result:
        haveControlAuthority = enable_obtain

    control_authority_mutex.release()

    return response.result


def publishControlAuthorityState():
    global controlAuthStatePub
    global haveControlAuthority

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        controlAuthStatePckt = Bool()
        controlAuthStatePckt.data = haveControlAuthority

        controlAuthStatePub.publish(controlAuthStatePckt)
        rate.sleep()


def set_joy_mode(set_state_autonomous):
    setupJoystickMode(autonomous_mode=set_state_autonomous.data)


#   Args: horizontal_mode vertical_mode yaw_mode horizontal_coordinate stable_mode
#   FlightController::JoystickMode
#   FlightController::HorizontalLogic::HORIZONTAL_VELOCITY,
#   FlightController::VerticalLogic::VERTICAL_VELOCITY,
#   FlightController::YawLogic::YAW_ANGLE,
#   FlightController::HorizontalCoordinate::HORIZONTAL_BODY,
#   FlightController::StableMode::STABLE_ENABLE,
#
#	autonomous_mode=True: Yaw values set the desired heading angle
#	autonomous_mode=False: Yaw values set rotation speed around its axis
def setupJoystickMode(autonomous_mode=True):
    global control_authority_mutex, joystickModeSrv

    if control_authority_mutex.locked():
        print('control_authority_mutex is locked by another thread. Will not setupJoystickMode')
        return False
    else:
        control_authority_mutex.acquire()

    horizontal_mode = 1
    vertical_mode = 0

    yaw_mode = 0
    if autonomous_mode:
        yaw_mode = 0
    else:
        yaw_mode = 1

    hoorizontal_coordinate = 1
    stable_mode = 1
    
    response = joystickModeSrv(horizontal_mode, vertical_mode, yaw_mode, hoorizontal_coordinate, stable_mode)
    print('Set Joystick Mode Response:', response)

    control_authority_mutex.release()

    return response.result


def listener(dji_name = "/matrice300", as_module = False):
    global responsePub, joystickModeSrv, joystickActionSrv
    global dronename
    global rate
    global x, y, z, yaw
    global controlAuthoritySrv, controlAuthStatePub

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_controlAuthorityHandler'
    
    print(nodename)

    if not as_module:
        rospy.init_node(nodename)

    # Joystick Mode Service
    rospy.wait_for_service(dronename+'/set_joystick_mode')
    joystickModeSrv=rospy.ServiceProxy(dronename+'/set_joystick_mode', SetJoystickMode)

    rospy.Subscriber(dronename+'/joystick_mode/Set_State_Autonomous', Bool, set_joy_mode)
    controlAuthStatePub = rospy.Publisher(dronename+'/Joystick_Mode/Get_State_Autonomous', Bool, queue_size=1, latch=True)

    # Control Authority
    rospy.wait_for_service(dronename+'/obtain_release_control_authority')
    controlAuthoritySrv=rospy.ServiceProxy(dronename+'/obtain_release_control_authority', ObtainControlAuthority)
    
    rospy.Subscriber(dronename+'/ControlAuthority/Set_State_Enable', Bool, obtainControlAuthority)
    controlAuthStatePub = rospy.Publisher(dronename+'/ControlAuthority/Current_State', Bool, queue_size=1, latch=True)

    pubThread = threading.Thread(target=publishControlAuthorityState)
    pubThread.start()

    if not as_module:
        rospy.spin()


try:
    print('Initializing controlAuthorityHandler')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]

    dronename = '/matrice300' + '_' + boardId

    if __name__ == '__main__':
        as_module = False
    else:
        as_module = True

    listener(dronename, as_module=as_module)
except rospy.ROSInterruptException:
    pass
