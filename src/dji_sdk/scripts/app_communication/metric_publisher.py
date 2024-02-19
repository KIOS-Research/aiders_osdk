#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import signal
import subprocess

import time
import datetime
import socket

import threading

import rospy

import numpy as np
import math

#from scipy.stats import norm
#from scipy.optimize import leastsq,least_squares
#from ekf_drps import EKF

from dji_sdk.msg import telemetry2, Packet, Packet2, Gimbal
from dji_sdk.msg import EscData, ESCStatusIndividual, WindData
from dji_sdk.srv import GetSingleBatteryDynamicInfo

from kios.msg import Telemetry, TerminalHardware, DroneHardware, DroneMovement
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Imu, NavSatFix, BatteryState
from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

ros_version = os.system('rosversion -d')
if ros_version == 'melodic':
    from multimaster_msgs_fkie.msg import LinkStatesStamped
elif ros_version == 'noetic':
    from fkie_multimaster_msgs.msg import LinkStatesStamped

from tf.transformations import euler_from_quaternion
from kios.msg import trisonica_msg

def GetBoardID():
    f = open("/var/lib/dbus/machine-id", "r")
    boardID = f.read().strip()[0:4]
    #print(boardID)
    return boardID

boardId = GetBoardID()
dronename = '/matrice300' + '_' + boardId



project_path = os.path.realpath(__file__)
project_path = project_path.split('aiders_osdk', 1)[0] + 'aiders_osdk/'

script_path = project_path + 'src/dji_sdk/scripts/'
log_path = project_path + 'flightlogs/'

if len(sys.argv) > 1:
    flightlog_path = log_path + sys.argv[1]
else:
    flightlog_path = log_path + dronename.replace('/', '') + '-' + datetime.datetime.now().strftime("%Y-%m-%d-T%H-%M") + '/'

if not os.path.exists(log_path):
    os.makedirs(log_path)
if not os.path.exists(flightlog_path):
    os.makedirs(flightlog_path)

print('project_path:', project_path)
print('script_path:', script_path)
print('log_path:', log_path)
print('flightlog_path:', flightlog_path)

#/home/kios/Documents/multimaster_db_matrice300_q3_demo/DRPS/ekffusion_drps_no_hackrf.py
sys.path.append(script_path)



print(dronename)

#dronename2='/matrice210RTK'

rawLat = 0
rawLon = 0
angularx=0
angulary=0
linearaccellxy=0
headingangle=0
gpsdat=np.zeros((2,2))
velocitylinear=0
altitude=0
#dataset=np.zeros((699,25))
#def velocitycallback(messagedata):

# Drone
d_quat_x = 0
d_quat_y = 0
d_quat_z = 0
d_quat_w = 0
# Gimbal
g_yaw = 0
g_pitch = 0
g_roll = 0

meters_sum, x_sum, y_sum = 0, 0, 0
RPSarray2=[0,0]


batteryPercentage = 0
batteryVoltage = 0
batteryCurrent = 0
batteryCapacity = 0
def batteryCB(batteryStatus):
    global batteryPercentage, batteryVoltage, batteryCurrent, batteryCapacity   
    batteryPercentage = batteryStatus.percentage
    batteryVoltage = batteryStatus.voltage
    batteryCurrent = batteryStatus.current
    batteryCapacity = batteryStatus.capacity


escCount = 0
escSpeed = []
escVoltage = []
escTemperature = []
def escMotorCB(escMotorStatus):
    global escCount, escSpeed, escVoltage, escTemperature
    #/matrice300_5807/ESC_data
    escCount = 0
    escSpeed = []
    escVoltage = []
    escTemperature = []
    
    for escMotor in escMotorStatus.esc:
        if escMotor.speed > 0:
            escCount += 1
            escSpeed.append(escMotor.speed)
            escVoltage.append(escMotor.voltage)
            escTemperature.append(escMotor.temperature)
    

gpssig = 0
satnumber = 0

#sequence number to count packet loss
seq = 0

droneIdAck = False
droneIDpub = None


hostname = hostname = socket.getfqdn()
local_ip = socket.gethostbyname_ex(hostname)[2][0]
platform_ip = None
# print("Hostname:", hostname, "Local IP:", local_ip)


def signal_handler(sig, frame): 
    global droneIDpub
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"False\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    
    if droneIDpub:
        droneIDpub.publish(hello_msg)
    #oc.rtmpHandler.deinitStream()
    print('Connected: False sent to platform.')
    sys.exit(0)


def EKF2_callback(rpsdata2):
    global RPSarray2
    RPSarray2  =  rpsdata2.data


flightLogicState = 'Idle-Hover'
def flightLogicStateCB(fls):
    global flightLogicState   
    flightLogicState = fls.data


def vision_callback(flowdata):
    global meters_sum, x_sum, y_sum
    meters_sum, x_sum, y_sum = flowdata.data
    print("DRPS Vision data: ",meters_sum, x_sum, y_sum,flowdata.data)
    
#/matrice300_5807/velocity
velocityX = 0.0
velocityY = 0.0
velocityZ = 0.0
def velocityCB(velocity):
    global velocityX, velocityY, velocityZ
    velocityX = velocity.vector.x
    velocityY = velocity.vector.y
    velocityZ = velocity.vector.z


#/matrice300_5807/angular_velocity_fused
angularVelocityX = 0.0
angularVelocityY = 0.0
angularVelocityZ = 0.0
def angularVelocityCB(angularVelocity):
    global angularVelocityX, angularVelocityY, angularVelocityZ
    angularVelocityX = angularVelocity.vector.x
    angularVelocityY = angularVelocity.vector.y
    angularVelocityZ = angularVelocity.vector.z

#matrice300_5807/acceleration_ground_fused
linearAccelerationX = 0.0
linearAccelerationY = 0.0
linearAccelerationZ = 0.0
def linearAccelerationCB(linearAcceleration):
    global linearAccelerationX, linearAccelerationY, linearAccelerationZ
    linearAccelerationX = linearAcceleration.vector.x
    linearAccelerationY = linearAcceleration.vector.y
    linearAccelerationZ = linearAcceleration.vector.z
    
windDirection = 0.0
windSpeed = 0.0
def windDataCB(windData):
    global windDirection, windSpeed
    
    windDirection = windData.direction
    windSpeed = windData.speed
    
    #windDirection = windData.windDirection
    #windSpeed = windData.windSpeed
    
    
def imucallback(messagedat):
    global angularx,angulary,linearaccellxy
    angularx=messagedat.angular_velocity.x
    angulary=messagedat.angular_velocity.y
    linearaccellxy=math.sqrt((messagedat.linear_acceleration.x)**2 + (messagedat.linear_acceleration.y)**2)

lat = 0.0
lon = 0.0
alt = 0.0
def telemetryCB(telemetry):
    global lat,lon, alt,headingangle,velocitylinear,altitude,rostime,rostime_nano,fligth_time_seconds, batteryPercentage, gpssig, satnumber, home_lat, home_lon, drone_serial
    lat=telemetry.latitude
    lon=telemetry.longitude
    altitude=telemetry.altitude
    headingangle=telemetry.heading
    velocitylinear=telemetry.velocity
    
    rostime=telemetry.rostime_secs
    rostime_nano=telemetry.rostime_nsecs
    fligth_time_seconds=telemetry.flightTimeSecs
    batteryPercentage=telemetry.batteryPercentage
    gpssig=telemetry.gpsSignal
    satnumber=telemetry.satelliteNumber
    home_lat=telemetry.homeLatitude
    home_lon=telemetry.homeLongitude
    drone_serial=telemetry.serialVersionUID


#def telemetrycallback_b(teldat2):
#    global lat2,lon2,headingangle2,velocitylinear2,altitude2
#    lat2=teldat2.latitude
#    lon2=teldat2.longitude
#    headingangle2=teldat2.heading
#    velocitylinear2=teldat2.velocity
#    altitude2=teldat2.altitude

def rawgpscallback(raw):
    global rawLat, rawLon
    rawLat = raw.latitude
    rawLon = raw.longitude

#def packetlosscb(data):
#        global packlosspub
#        print('ta start ine', data.start)
#        db.create_tables([Data])
#        print('print to for',data.fin-data.start)
#        for i in range(data.fin-data.start,data.fin):
#            print('to fin ine',data.fin)
#            print('to seq ine',seq)
#            print('to i ine',i)
#            for test in Data.select().where(Data.seq == i):
#                print('to test ine :',test)
#                newpacket2 = Packet2()
#                newpacket2.seq = test.seq
#                newpacket2.serialVersionUID = test.serialVersionUID.encode('ascii', 'replace')
#
#                print('to xameno packet ine:',newpacket2.seq)
#                newpacket2.heading = test.head
#                newpacket2.velocity = test.vel
#                newpacket2.longitude = test.lon
#                newpacket2.latitude = test.lat
#                newpacket2.altitude = test.altitude
#                newpacket2.date_time = test.date_time.encode('ascii', 'replace')
#                newpacket2.rostime_secs = 0
#                newpacket2.flightTimeSecs = int(test.fl_time)
#        
#                newpacket2.batteryThreshold = test.batteryThresh
#                newpacket2.batteryPercentage = test.battery
#                newpacket2.cpuTemp = test.cpuTemp
#                newpacket2.gpuTemp = test.gpuTemp
#
#                newpacket2.gpsSignal = test.gpssgnal
#                newpacket2.satelliteNumber = test.gpssat
#                newpacket2.homeLatitude = test.homLat
#                newpacket2.homeLongitude = test.homLon
#                   
#                       
#                # Drone Orientation
#                newpacket2.d_quat_x = test.d_quat_x
#                newpacket2.d_quat_y = test.d_quat_y
#                newpacket2.d_quat_z = test.d_quat_z
#                newpacket2.d_quat_w = test.d_quat_w
#                # Gimbal Orientation
#                newpacket2.g_yaw = test.g_yaw
#                newpacket2.g_pitch = test.g_pitch
#                newpacket2.g_roll = test.g_roll
#                packlosspub.publish(newpacket2)

d_quat_x = 0.0
d_quat_y = 0.0
d_quat_z = 0.0
d_quat_w = 0.0

    
d_roll = 0.0
d_pitch = 0.0
d_yaw = 0.0
rad2deg = 57.2958
def DroneOrientationCB(msg):
    global d_quat_x, d_quat_y, d_quat_z, d_quat_w
    global d_roll, d_pitch, d_yaw
    d_quat_x = msg.quaternion.x
    d_quat_y = msg.quaternion.y
    d_quat_z = msg.quaternion.z
    d_quat_w = msg.quaternion.w

    quat = [d_quat_x, d_quat_y, d_quat_z, d_quat_w]
    (d_roll, d_pitch, d_yaw) = euler_from_quaternion(quat)
    d_roll *= rad2deg
    d_pitch *= rad2deg * -1.0
    d_yaw *= rad2deg    


def Gimbal_cb(gimbal):
    global g_yaw, g_pitch, g_roll
    g_yaw = gimbal.vector.z
    g_pitch = gimbal.vector.y
    g_roll = gimbal.vector.x


def GetJetsonCPUTemp():
    global cpuTemp
    f = open("/sys/class/thermal/thermal_zone0/temp", "r")
    cpuTemp = int(f.read().strip())
    return cpuTemp

def GetJetsonGPUTemp():
    f = open("/sys/class/thermal/thermal_zone1/temp", "r")
    gpuTemp = int(f.read().strip())
    return gpuTemp

# tegrastats
# RAM 2130/7774MB (lfb 1004x4MB) SWAP 0/3887MB (cached 0MB) 
# CPU [11%@1190,2%@1190,1%@1190,1%@1190,1%@1190,2%@1190] EMC_FREQ 0% GR3D_FREQ 0% 
# AO@31C GPU@31.5C PMIC@100C AUX@31.5C CPU@33C thermal@31.95C

# sudo tegrastats
# RAM 2387/7774MB (lfb 961x4MB) SWAP 0/3887MB (cached 0MB) 
# CPU [14%@1190,9%@1190,2%@1190,4%@1190,0%@1190,1%@1344] EMC_FREQ 1%@1600 GR3D_FREQ 0%@114 
# VIC_FREQ 0%@115 APE 150 MTS fg 0% bg 3% 
# AO@31C GPU@31.5C PMIC@100C AUX@31.5C CPU@33C thermal@31.95C 
# VDD_IN 3528/3569 VDD_CPU_GPU_CV 410/426 VDD_SOC 1107/1115
def GetTerminalHardware():     
    #p = subprocess.Popen('tegrastats | head -n1', stdout=subprocess.PIPE, shell = True)
    try:
        p = subprocess.Popen(('/usr/bin/tegrastats'), stdout=subprocess.PIPE)
        tegrastats_stdout = subprocess.check_output(('head', '-n1'), stdin=p.stdout)
    except Exception as e:
        print('non-fatal tegrastats exception:', e)
        return None
     
    #tegrastats_stdout = p.stdout.readline()
    tegrastats_stdout = tegrastats_stdout.split()
    tegrastats_stdout = [item.decode('utf-8') for item in tegrastats_stdout]

    p.kill()    
    #print(tegrastats_stdout)

    termHwPacket = TerminalHardware()
    termHwPacket.seq = seq
    termHwPacket.uid = GetBoardID()  
    
    ram_str = tegrastats_stdout[1]
    #print('ram_str:', ram_str)
    ram_str = ram_str.replace('MB', '').split('/')
    #print('ram_str:', ram_str)
    
    
    termHwPacket.ram_use = int(ram_str[0])
    termHwPacket.ram_max = int(ram_str[1])
    
    swap_str = tegrastats_stdout[5]
    #print("swap_str", swap_str)
    swap_str = swap_str.replace('MB', '').split('/')
    #print("swap_str", swap_str)
    termHwPacket.swap_use = int(swap_str[0])
    termHwPacket.swap_max = int(swap_str[1])
    
    cached_swap_str = tegrastats_stdout[7]
    #print(cached_swap_str)
    cached_swap_str = cached_swap_str.replace('MB)', '')
    
    termHwPacket.swap_cache = int(cached_swap_str)
    
    emc_str = tegrastats_stdout[11]
    #print(emc_str)
    termHwPacket.emc_usage = int(emc_str.replace('%', ''))
    
    cpu_str = tegrastats_stdout[9] 
    #print(cpu_str)
    cpu_str = cpu_str.replace('[','').replace(']','')    
    cpu_str = cpu_str.split(',')
    
    cpu_core_usage = []
    cpu_core_freq = []
    for core in cpu_str:
        core_str = core.split('%@')
        cpu_core_usage.append(int(core_str[0]))
        cpu_core_freq.append(int(core_str[1]))
        
    termHwPacket.cpu_core_count = len(cpu_core_usage)
        
    termHwPacket.cpu_core_usage = cpu_core_usage
    termHwPacket.cpu_core_freq = cpu_core_freq
    termHwPacket.cpuTemp = GetJetsonCPUTemp()
    
    f = open("/sys/devices/generic_pwm_tachometer/hwmon/hwmon1/rpm", "r")
    termHwPacket.cpuFanRPM = int(f.readline())
    f.close()
    
    gr3d_str = tegrastats_stdout[13]
    termHwPacket.gr3d_usage = int(gr3d_str.replace('%',''))
    
    #    gpu.0/devfreq/17000000.gv11b/cur_freq
    f = open("/sys/devices/gpu.0/devfreq/17000000.gv11b/cur_freq", "r")
    #print(f.readline())
    termHwPacket.gr3d_freq = int(f.readline())
    termHwPacket.gpuTemp = GetJetsonGPUTemp()
    f.close()
    
    return termHwPacket




#def Create3dMesh():
#    meshConstructorDir = '/home/jetson/Documents/mesh_construction/aidersplatform_algorithms_mesh_construction.py'
#    
#    # Move Some Files
#    p = subprocess.Popen(('python', meshConstructorDir), stdout=subprocess.PIPE)
#    p.wait()
#    p.kill()
#    
#    print('Mesh Construction has finished')
#    print('Posting 3d Mesh to platform')    
#    # Move some more   
#    # Post 3d mesh to platform
    
    
def DroneIdAckCB(ack):
    global droneIdAck    
    print('Ack received', ack)
    
    if ack.data == '1':
        droneIdAck = True
                  
            
def checkPortStatus(ip, port):
    p = subprocess.Popen(("nc", "-zvw5", ip, port), stderr = subprocess.PIPE)
    status = p.communicate()[1]
    
    print(str(status))
    
    
    # print (rc)
    # print (status)
    if 'succeeded' in str(status):
        return True
    else:
        return False


def findPlatformIp():
    print('Finding platform Ip...')
    platformIp = None

    while not platformIp:
        linkStats = rospy.wait_for_message('/master_discovery/linkstats', LinkStatesStamped)
        rosdevices = linkStats.links
        
        for device in rosdevices:
            print(device.destination, local_ip)
            if device.destination != local_ip:
                if checkPortStatus(device.destination, '8000') and checkPortStatus(device.destination, '1935'):
                    platformIp = device.destination
                    print("Platform Ip:", device)
                    return platformIp

        rospy.sleep(1)


def handshakePlatform():
    global droneIdPub, droneIdAckSub, droneIdAck
    
    droneIdAckSub = rospy.Subscriber(dronename+'/handshake',String, DroneIdAckCB)
    droneIdPub = rospy.Publisher('/droneIds',String, queue_size=2)  
    
    
    hello_msg = "{\"Timestamp\":\""+str(rospy.get_rostime())+"\",\"DroneName\":\""+dronename.replace('/','')+"\",\"Connected\":\"True\",\"Model\":\"MATRICE_300_RTK\",\"cameraVideoStreamSource\":\"unknown\",\"cameraName\":\"Zenmuse_H20T\"}"
    timeout = 1000000
    print(hello_msg)
    for i in range(0, timeout):
        droneIdPub.publish(hello_msg)
        print('Attempt:\t' + str(i))
        rospy.sleep(5)
        
        if droneIdAck:
            print('Platform register Successful')
            droneIdAck = False
            return True
 
    if not droneIdAck:
        print('Could not register on platform.\n') 
        return False


def setupTopics():
    global imuSub, rawGpsSub, gpsPub, droneOrientationSub, gimbalSub, terminalHwPub, droneHwPub, droneMvPub, telemetrySub
    global packlosspub, packetSub, packetPub2
    
    print('Setting up rostopics...')
    #imuSub=rospy.Subscriber(dronename+'/imu', Imu, imucallback)
    
    #rawGpsSub=rospy.Subscriber(dronename+'/raw_gps_position',NavSatFix,rawgpscallback)
    #gpsPub=rospy.Publisher(dronename+'/GPSmeters',Float64MultiArray, queue_size=1)
    
    #packlosspub=rospy.Publisher(dronename+'/PacketLoss',Packet2, queue_size=1)
#    packetSub=rospy.Subscriber(dronename+'/Packet',Packet, packetlosscb)
    #packetPub2=rospy.Publisher(dronename+'/Packet2',Packet2, queue_size=1)
    
    droneOrientationSub = rospy.Subscriber(dronename+'/attitude',QuaternionStamped, DroneOrientationCB)
    gimbalSub=rospy.Subscriber(dronename+'/gimbal_angle',Vector3Stamped, Gimbal_cb)
    
    telemetrySub = rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)

    rospy.Subscriber(dronename+'/battery_state', BatteryState, batteryCB)
    rospy.Subscriber(dronename+'/ESC_data', EscData, escMotorCB) 
    
    terminalHwPub = rospy.Publisher(dronename+'/TerminalHardware', TerminalHardware, queue_size=1)
    droneHwPub = rospy.Publisher(dronename+'/DroneHardware', DroneHardware, queue_size=1)
    
    droneMvPub = rospy.Publisher(dronename+'/DroneMovement', DroneMovement, queue_size=10)
    
    rospy.Subscriber(dronename+'/velocity', Vector3Stamped, velocityCB)
    rospy.Subscriber(dronename+'/angular_velocity_fused', Vector3Stamped, angularVelocityCB)
    rospy.Subscriber(dronename+'/acceleration_ground_fused', Vector3Stamped, linearAccelerationCB) 
    
    #rospy.Subscriber(dronename+'/wind_data', WindData, windDataCB)
    rospy.Subscriber(dronename+'/WeatherStation', trisonica_msg, windDataCB)    
    
    rospy.Subscriber(dronename+'/FlightLogicState', String, flightLogicStateCB)

    flightlog_directory_pub = rospy.Publisher(dronename + '/FlightLogDirectory', String, queue_size=1, latch=True)
    logdir_pckt = String()
    logdir_pckt.data = flightlog_path
    print('Publishing FlightLogDirectory:', flightlog_path)
    flightlog_directory_pub.publish(logdir_pckt)
     
    print('Setting up rostopics FINISHED')       


def publishTerminalHardware():
    hwPckt = GetTerminalHardware()
    if hwPckt is not None:
        terminalHwPub.publish(hwPckt)


def publishDroneHardware():
    global droneHwPub
    
    dhwPacket = DroneHardware()
    
    dhwPacket.seq = seq
    dhwPacket.uid = boardId
    
    dhwPacket.batteryThreshold = 10
    dhwPacket.batteryPercentage = int(batteryPercentage)
    dhwPacket.batteryTemperature = 0
    
    try:
        batteryTempSrv = rospy.ServiceProxy(dronename+'/get_single_battery_dynamic_info', GetSingleBatteryDynamicInfo)
        response = batteryTempSrv(1)
        
        if response:
            dhwPacket.batteryTemperature = response.smartBatteryDynamicInfo.batteryTemperature
        else:
            dhwPacket.batteryTemperature = 3735928559            
        
    except Exception as e:
        #print('Battery Temperature Service Error', e)
        dhwPacket.batteryTemperature = 3735928559   
    
    dhwPacket.batteryVoltage = int(batteryVoltage)
    dhwPacket.batteryCurrent = int(batteryCurrent)
    dhwPacket.batteryCapacity = int(batteryCapacity)

    dhwPacket.escCount = escCount
    dhwPacket.escSpeed = escSpeed
    dhwPacket.escVoltage = escVoltage
    dhwPacket.escTemperature = escTemperature
    
    droneHwPub.publish(dhwPacket)    
  

mvSeq = 0
def publishDroneMovement():
    global droneMvPub
    global mvSeq
    global flightLogicState
    
    dmvPacket = DroneMovement()
    
    dmvPacket.seq = mvSeq
    mvSeq +=1
    
    dmvPacket.uid = boardId
    dmvPacket.timestamp = int(time.time())
    
    dmvPacket.flight_logic_state = flightLogicState
    dmvPacket.wind_speed = windSpeed
    dmvPacket.wind_angle = windDirection

    dmvPacket.battery_voltage = batteryVoltage
    dmvPacket.battery_current = batteryCurrent
    
    dmvPacket.position_x = lon
    dmvPacket.position_y = lat
    dmvPacket.position_z = altitude
    dmvPacket.altitude = 200
    
    dmvPacket.orientation_x = d_quat_x
    dmvPacket.orientation_y = d_quat_y
    dmvPacket.orientation_z = d_quat_z
    dmvPacket.orientation_w = d_quat_w
    
    dmvPacket.velocity_x = velocityX
    dmvPacket.velocity_y = velocityY
    dmvPacket.velocity_z = velocityZ
    
    dmvPacket.angular_x = angularVelocityX
    dmvPacket.angular_y = angularVelocityY
    dmvPacket.angular_z = angularVelocityZ
    
    dmvPacket.linear_acceleration_x = linearAccelerationX
    dmvPacket.linear_acceleration_y = linearAccelerationY
    dmvPacket.linear_acceleration_z = linearAccelerationZ
    
    dmvPacket.payload = 0.471
    
    droneMvPub.publish(dmvPacket)
    

# Create a list to store functions
function_queue = []

# Function to execute the functions in separate threads
def publish_metrics():
    threads = []

    for func in function_queue:
        thread = threading.Thread(target=func)
        threads.append(thread)
        thread.start()

        #print('Started function:', str(func))



if __name__=='__main__':   
    print('Initializing ekffusionnode...')
    rospy.init_node(dronename.replace('/', ''))
    
    #rate=rospy.Rate(0.5)
    #rate=rospy.Rate(10)        # 10hz
    rate=rospy.Rate(1)
    
    precision = 8
    np.set_printoptions(precision = precision)
        
    setupTopics()
    
    rate.sleep()

    function_queue.append(publishTerminalHardware)
    function_queue.append(publishDroneHardware)
    function_queue.append(publishDroneMovement)

    while not rospy.is_shutdown():
        now = datetime.datetime.now()
        timestmp = now.strftime("%Y%m%d-%H:%M:%S.%s")
        rate.sleep()
        
        publish_metrics()
        
        if seq % 100 == 0:
            #print('ekfussion:', boardId, timestmp)
            pass
            
        seq = seq+1
    
#    except rospy.ROSInterruptException: 
#        pass
