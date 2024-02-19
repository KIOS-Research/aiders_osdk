import rospy


from std_msgs.msg import Bool
from eddi_messages.msg import ConSertOutput

def simulate_spoof_CB(spoof):
    global forth_spoof_Pub
    #print('simulate_spoof_CB received:', spoof.data)
    forth_spoof_Pub.publish(spoof.data)    


han_val = True
man_val = True
guarantee_fail = False
def consert_output_CB(consert_val):
    global han_val, man_val
    global telemetry2_state_Pub
    global guarantee_fail

    #print(consert_val)

    for guarantee in consert_val.guarantees:
        if guarantee.id == 'han':
            han_val = guarantee.active
        elif guarantee.id == 'man':
            man_val = guarantee.active

    if not han_val and not man_val:
        if not guarantee_fail:
            print('han and man guarantee fail')
            telemetry2_state_Pub.publish(False)
            guarantee_fail = True


def crps_state_CB(crps_state):
    global crps_state_consert_Pub
    crps_state_consert_Pub.publish(crps_state.data)



def init():
    global dronename
    global forth_spoof_Pub, telemetry2_state_Pub
    global crps_state_consert_Pub

    rospy.init_node(dronename + '_consert_handler')
    
    # Dummy Forth Publisher
    forth_spoof_Pub = rospy.Publisher(dronename+'/forth/gnss', Bool, queue_size=1)
    
    # Activate CRPS
    telemetry2_state_Pub = rospy.Publisher(dronename+'/telemetry2/Set_State_Enable', Bool, queue_size=1)

    # Receive CRPS state
    rospy.Subscriber(dronename+'/crps/initiate_collaboration_request/Get_State_Enable', Bool, crps_state_CB)

    

    # Output from consert
    rospy.Subscriber(dronename+'/drone/navigation', ConSertOutput, consert_output_CB)
    
    # Input to consert
    crps_state_consert_Pub = rospy.Publisher(dronename+'/kios/clengaged', Bool, queue_size=1)

    # Initialize simulated spoofing
    rospy.Subscriber(dronename+'/simulate_spoof', Bool, simulate_spoof_CB)
    print('Consert Handler Initialization Complete')


def listener(dji_name = 'matrice300'):
    global dronename, localIp, platformIp, platformIpPub, componentListPub
    
    print('Initializing ConSert handler...')
    dronename = dji_name
    
    

    init()
    rospy.spin()


print('Initializing Fault Detection...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]
    
if __name__=='__main__':
    listener('matrice300_' + boardId)
