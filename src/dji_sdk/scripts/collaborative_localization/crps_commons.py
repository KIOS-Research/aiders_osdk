import os, sys
import rospy
from std_msgs.msg import Bool
#from dji_sdk.msg import Telemetry, CrpsRequest
from kios.msg import CrpsRequest

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/drone_movement/')
import flightCommons as flc

crps_state_enabled = False
def crps_state_CB(crps_state_enable):
    global crps_state_enabled
    crps_state_enabled = crps_state_enable.data


def get_crps_request_state():
    global crps_state_enabled
    return crps_state_enabled


def set_crps_request_state(set_state_enable):
    global set_crps_request_state_pub

    crps_request = CrpsRequest()
    crps_request.serialVersionUID = boardId
    crps_request.request_active = set_state_enable
    crps_request.telemetry = flc.get_telemetry()

    set_crps_request_state_pub.publish(crps_request)


def init():
    global set_crps_request_state_pub

    rospy.Subscriber(dronename+'/crps/initiate_collaboration_request/Get_State_Enable', Bool, crps_state_CB)
    set_crps_request_state_pub = rospy.Publisher(dronename+'/crps/initiate_collaboration_request/Set_State_Enable', CrpsRequest, queue_size=1, latch=True)

    flc.init()


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, boardId, nodename

    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename.replace('/', '') + '_crps_commons'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing CRPS commons...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = 'matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)