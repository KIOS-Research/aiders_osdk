import rospy
from std_msgs.msg import String
from kios.msg import Telemetry
import json
import time

dronename=" "
pub=None
pub2=None
    
def GetBoardID():
    f = open("/var/lib/dbus/machine-id", "r")
    boardID = f.read().strip()[0:4]
    # print(boardID)
    return boardID

dronename = '/matrice300_' + GetBoardID()

def callback(msg):
    global c
    lat_long = str(msg.latitude) + "," + str(msg.longitude)
    lora_msg = String()
    lora_msg.data = lat_long
    pub_name = dronename + "/loremetry"
    pub = rospy.Publisher(pub_name, String, queue_size=1)
    # if lora_msg:
    rospy.sleep(0.1)
    pub.publish(lora_msg)

def get_data():
    global pub

    sub_name = dronename + "/crps/Telemetry"
    print(sub_name)
    
    rospy.Subscriber(sub_name, Telemetry, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('loremetry_translator2', anonymous=True)
    try:
        get_data()
    except rospy.ROSInterruptException:
        pass
