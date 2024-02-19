import sys
from dji_sdk.msg import EscData, ESCStatusIndividual
from sensor_msgs.msg import BatteryState

from kios.msg import DroneHardware
from std_msgs.msg import String, Int8

from geometry_msgs.msg import Vector3Stamped, QuaternionStamped


import rospy

boardId = ""
translator= None

home_latitude = 0.0
home_longitude = 0.0

droneState='Landed'
# In_Mission
# Paused_Mission
# Flying
# Going_Home
# Armed
# Landed

def escCB(motors):
    global escCount, escSpeed, escVoltage, escTemperature
    
    escCount = 0
    escSpeed = []
    escVoltage = []
    escTemperature = []
    for motor in motors.esc:
        if motor.speed:
            escSpeed.append(motor.speed)
            escVoltage.append(motor.voltage)
            escTemperature.append(motor.temperature)
            escCount += 1


def batteryStateCB(battery):
    global batteryVoltage, batteryCurrent, batteryCapacity, batteryPercentage
    batteryVoltage = battery.voltage
    batteryCurrent = battery.current
    batteryCapacity = battery.capacity
    batteryPercentage = battery.percentage


def publishDroneHardware(seq):
    droneHwPacket = DroneHardware()
    
    droneHwPacket.seq = seq
    droneHwPacket.uid = boardId
    
    droneHwPacket.batteryThreshold = 10
    droneHwPacket.batteryPercentage = batteryPercentage
    droneHwPacket.batteryTemperature = batteryTemperature
    
    droneHwPacket.batteryVoltage = batteryVoltage
    droneHwPacket.batteryCurrent = batteryCurrent
    droneHwPacket.batteryCapacity = batteryCapacity
    
    droneHwPacket.escCount = escCount
    droneHwPacket.escSpeed = escSpeed
    droneHwPacket.escVoltage = escVoltage
    droneHwPacket.escTemperature = escTemperature
    
    droneHwPub.publish(droneHwPacket)
    	

def listener(dji_name = "matrice300"):
    global boardId, droneHwPub, home_latitude, home_longitude

    boardId = dji_name.split('_', 1)[1]
    print('droneTranslator boardId:', boardId)
    
    rospy.Subscriber(dji_name+'/battery_state', BatteryState, batteryStateCB)
    rospy.Subscriber(dji_name+'/gimbal_angle', EscData, escCB)
    
    droneHwPub = rospy.Publisher(dji_name+'/DroneHardware', DroneHardware, queue_size=10) 
    print('Drone Translator READY')
    
    rate=rospy.Rate(1)        
    seq = 0
    while not rospy.is_shutdown():
        rate.sleep()
        publishDroneHardware(seq)
        seq += 1
        

if __name__ == '__main__':
	try:
		rospy.init_node('jetson300translator', anonymous=False)
		private_param = rospy.get_param('~dji_name', "matrice300")
		listener(dji_name = private_param)
	except rospy.ROSInterruptException:
		pass
