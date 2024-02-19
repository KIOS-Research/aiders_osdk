import rospy

from std_msgs.msg import Bool
from dji_sdk.srv import ObtainControlAuthority, SetJoystickMode


haveControlAuthority = False
def controlAuthorityStateCB(controlAuthorityState):
    global haveControlAuthority
    haveControlAuthority = controlAuthorityState.data

def get_control_authority_state():
    global haveControlAuthority
    return haveControlAuthority

def obtain_ControlAuthority(enable_obtain):
    global ControlAuthorityPub
    
    thread_id = '[' + str(threading.current_thread().ident) + ']'
    print('\n' + str(thread_id),'- obtainControlAuthority:', enable_obtain)
    
    c = 0
    c_max = 5
    retry_rate = rospy.Rate(1)
    print('obtain_ControlAuthority - get_control_authority_state():', get_control_authority_state())
    print('obtain_ControlAuthority - enable_obtain:', enable_obtain)
    while get_control_authority_state() != enable_obtain and c < c_max and not rospy.is_shutdown():
        if not enable_obtain:
            publishAction(0, 0, 0, yaw)
    
        ControlAuthorityPub.publish(enable_obtain)
        
        if get_control_authority_state() == enable_obtain:
            print('\n' + str(thread_id),'- Set_ControlAuthority:', 'Success', str(c) + '/' + str(c_max))
        else:
            print('\n' + str(thread_id),'- Set_ControlAuthority:', 'Fail', str(c) + '/' + str(c_max))
        
        c += 1
        retry_rate.sleep()



#	autonomous_mode=True: Yaw values set the desired heading angle
#	autonomous_mode=False: Yaw values set rotation speed around its axis
def set_joy_mode(autonomous_mode=True):
    global joy_mode_pub   
    joy_mode_pub.publish(autonomous_mode)


def init():
    global joy_mode_pub

    # Joystick Mode Service
    joy_mode_pub = rospy.Publisher(dronename+'/joystick_mode/Set_State_Autonomous', Bool, queue_size=1)

    # Control Authority
    rospy.Subscriber(dronename+'/ControlAuthority/Set_State_Enable', Bool, controlAuthorityStateCB)
    ControlAuthorityPub = rospy.Publisher(dronename+'/ControlAuthority/Set_State_Enable', Bool, queue_size=1)

    #JoystickActionPub = rospy.Publisher(dronename+'/Joystick/PublishJoystickAction', JoystickParams, queue_size=1)


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, nodename

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_controlAuthorityHandler'
    
    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

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
