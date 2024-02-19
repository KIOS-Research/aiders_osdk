import rospy
import time

from std_msgs.msg import String, Bool
from dji_sdk.msg import RangefinderData

rangefinder_state_active = False
def set_rangefinder_state_CB(state_enable):
    global rangefinderStatePub
    global rangefinder_state_active
    global rangefinder_state_pub

    desired_state = state_enable.data
    print('desired_state', desired_state)
    if desired_state:
        rangefinderStatePub.publish('START')
    else:
        rangefinderStatePub.publish('STOP')

    

    rangefinder_state_active = desired_state
    rangefinder_state_pub.publish(rangefinder_state_active)


rangefinder_json = None
def rangefinderDataCB(rf_data):
    global rangefinder_json
    rangefinder_json = rf_data.data
    rangefinder_json = rangefinder_json.replace('"', '')
    
    # print('Received from rangefinder:', rangefinder_json)
    # {"Latitude":"35.09866","Longitude":"33.426697","Altitude":"-0.8","TargetDistance":"4.6","LaserMeasureInfo":"0.207"}

    rfd_packet = RangefinderData()
    rfd_packet.latitude = float(rangefinder_json.split('Latitude:')[1].split(',')[0])
    rfd_packet.longitude = float(rangefinder_json.split('Longitude:')[1].split(',')[0])
    rfd_packet.altitude = float(rangefinder_json.split('Altitude:')[1].split(',')[0])
    rfd_packet.target_distance = float(rangefinder_json.split('TargetDistance:')[1].split(',')[0])
    rfd_packet.laser_measure_info = float(rangefinder_json.split('LaserMeasureInfo:')[1].replace('}', ''))

    rangefinder_data_pub.publish(rfd_packet)


def listener(dji_name = "/matrice300"):
    global dronename
    global rangefinderStatePub, rangefinder_state_pub, rangefinder_data_pub
    global rangefinder_state_active

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_rangefinder_handler'

    print('rangefinder Handler', nodename)

    if __name__=='__main__':
        rospy.init_node(nodename)

    # Autoland Mobile App
    rangefinderStatePub = rospy.Publisher(dronename+'/rangefinder'+'/StartOrStopRangeFinder', String, queue_size=1)
    rospy.Subscriber(dronename+'/rangefinder'+'/RangeFinderData', String, rangefinderDataCB)
    
    # ROS Interfacing
    rospy.Subscriber(dronename+'/Rangefinder'+'/Set_State_Enable', Bool, set_rangefinder_state_CB)
    rangefinder_state_pub = rospy.Publisher(dronename+'/Rangefinder'+'/Current_State', Bool, queue_size=1, latch=True)
    rangefinder_data_pub = rospy.Publisher(dronename+'/Rangefinder'+'/Data', RangefinderData, queue_size=1)
    
    
    rangefinder_state_pub.publish(rangefinder_state_active)

    if __name__=='__main__':
        rospy.spin()


def init():
    print('Initializing rangefinder Handler')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]

    dronename = 'matrice300' + '_' + boardId

    listener(dronename)


if __name__=='__main__':
    init()
else:
    init()