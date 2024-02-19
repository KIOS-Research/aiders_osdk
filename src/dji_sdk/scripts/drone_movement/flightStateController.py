 #!/usr/bin/env python

# Author: Christos Georgiades
# Email: cgeorg15@ucy.ac.cy
# Date :07/07/2022


import sys

import rospy
import threading
import time 

from std_msgs.msg import String, Bool, Int32, Float32
#from dji_sdk.msg import ScanArea, MissionWaypoint, 
from dji_sdk.msg import JoystickParams
from kios.msg import Telemetry
#from kios.msg import MissionDji, GpsInput, MissionCommandDJI

from dji_sdk.srv import SetJoystickMode, JoystickAction, ObtainControlAuthority
#from dji_sdk.msg import ScanArea, MissionWaypoint, 
from dji_sdk.msg import JoystickParams, MissionWaypointTask, MissionList
from kios.msg import Telemetry, MissionDji

import flightCommons as flc
#import flightController as flController
import flightLogic as flLogic

from task_queue import TaskQueue
from drone_mission import DroneMission

import fault_detection

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/logging_system')
import logging_commons as log

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/collaborative_localization/')
import crps_commons as crps

#import controlAuthorityHandler as controlAuthority

cmd2Action = ['Start', 'Stop', 'Pause', 'Resume', 'Take-Off', 'Landing-Monitored', 'Landing-LZ']
cmd2State = ['In_Mission', 'Flying', 'Paused_Mission', 'In_Mission', 'Taking-Off', 'Landing-Forced', 'Landing-LZ']

missionQueue = TaskQueue()
missionQueue_completed = TaskQueue()


# flightLogicState = 'Idle-Hover'
# def publishFlightLogicState():
#     global flightLogicStatePub, flightLogicState
    
#     flsPacket = String()
#     flsPacket.data = str(flightLogicState)
    
#     flightLogicStatePub.publish(flsPacket)


def publish_task_list():
    global mission_list_pub

    print('publish_task_list() - START')

    task_list = missionQueue.get_task_list()
    task_count = len(task_list)

    task_string_list = []

    print('task_count:', str(task_count))
    print('task_list:', str(task_list))

    for task in task_list:
        print('publish_task_list() - task:', str(task))

        thread_target = task.thread_target

        if thread_target:
            task_string = str(thread_target) + '-' + str(task.get_waypoint_index()) + '/' + str(task.get_waypoint_index_max())

            print('publish_task_list() - thread_target:', str(thread_target))
            task_string_list.append(str(thread_target))
        else:
            waypoints = str(task.get_waypoint_index()) + '/' + str(task.get_waypoint_index_max())
            print('publish_task_list() - Waypoints:', waypoints)
            task_string_list.append(waypoints)

    mission_list = MissionList()
    mission_list.task_count = task_count
    mission_list.task_list = task_string_list

    mission_list_pub.publish(mission_list)
    print('publish_task_list() - DONE')


fault_response_index = 0
cmd2FaultResponse = ['RTH', 'Continue Mission', 'Halt', 'Land-in-Place']
def fault_response_CB(fault_response_packet):
    global fault_response_index, fault_response_state_pub
    fr_index = fault_response_packet.data

    if fr_index >= 0 and fr_index < len(cmd2FaultResponse):
        fault_response_index = fr_index
        fault_response_state_packet = String()
        fault_response_state_packet.data = cmd2FaultResponse[fault_response_index]
        fault_response_state_pub.publish(fault_response_state_packet)


minimumAltitude = 0.0
def minimumAltitudeCB(minAlt):
    #global minimumAltitude
    #minimumAltitude = minAlt.data
    flLogic.set_minimum_altitude(minAlt.data)


latest_mission = None
def MissionCommandCB(missionMsg):
    global latest_mission

    if latest_mission == missionMsg:
        print('Received duplicate task:', missionMsg)
        return

    latest_mission = missionMsg

    cmd = missionMsg.missionCommand.missionCommand

    print('\nReceived Command from Platform:', cmd2Action[cmd], '\n')
    print('missionMsg:', str(missionMsg), '\n')
    
    handle_command_msg(missionMsg)
    publish_task_list()


def handle_command_msg(missionMsg):
    cmd_fulfilled = False
    cmd = missionMsg.missionCommand.missionCommand

    if cmd >= 1 and cmd <= 3:
        handle_TaskQueue_commands(cmd)
    else:
        drone_mission = create_mission(missionMsg)
        if drone_mission:
            missionQueue.put(drone_mission)


def handle_TaskQueue_commands(cmd):
    cmd_fulfilled = False

    if cmd == 1:  # stop
        stopMission()
    elif cmd == 2:  # pause
        pauseMission()
    elif cmd == 3:  # resume
        resumeMission()
    elif cmd == -1:
        pass
        # skip task

    #set_drone_state(cmd)
    set_drone_state(missionQueue)


def create_mission(missionMsg):
    cmd = missionMsg.missionCommand.missionCommand
    drone_mission = None

    if cmd == 0:    # start
        drone_mission = DroneMission(missionMsg, flLogic.traverseRoute)
    elif cmd == 4:  # take-off
        drone_mission = DroneMission(missionMsg, flLogic.vtol.monitored_takeoff())    
    elif cmd == 5:  # force_landing
        drone_mission = DroneMission(missionMsg, flLogic.vtol.monitored_landing())          
    elif cmd == 6:  # lz_landing
        drone_mission = DroneMission(missionMsg, flLogic.lz.lz_landing)
    elif cmd == 8:
        pass
        #return to home
    elif cmd == 7:
        print('Collecting Water')

    return drone_mission
    

def stopMission():
    #missionQueue.set_queue_active(False)
    if missionQueue.size() > 0:
        current_mission = missionQueue.get()
        current_mission.set_mission_active(False)

        missionQueue.dump_queue()

def pauseMission():
    missionQueue.set_queue_active(False)

    if missionQueue.size() > 0:
        current_mission = missionQueue.get()
        current_mission.set_mission_active(False)

def resumeMission():
    missionQueue.set_queue_active(True)

    if missionQueue.size() > 0:
        current_mission = missionQueue.get()
        current_mission.set_mission_active(True)


def check_task_completion(missionQueue):
    task_finished = False

    #print('get_queue_active:',missionQueue.get_queue_active())

    if missionQueue.get_queue_active() and missionQueue.size() > 0:
        current_mission = missionQueue.get()
        task_finished = current_mission.get_task_finished()


            
        if task_finished:
            print('\nMoving finished task from: missionQueue to: missionQueue_completed')
            print('current_mission:', current_mission)
            print('task_finished:', task_finished, '\n')
            
            missionQueue_completed.put(missionQueue.pop())
            publish_task_list()
        else:
            resumeMission()

    return task_finished


print_no_missions_log = True
def activate_next_task(missionQueue):
    global print_no_missions_log
    next_task_active = False

    if missionQueue.get_queue_active() and missionQueue.size() > 0:
        print_no_missions_log = True
        
        current_mission = missionQueue.get()
        if not current_mission.get_mission_active():
            current_mission.set_mission_active(True)
        
            log.create_publish_log('Activating Next Mission', 'Flight State Controller - Mission Worker')

        return True
    elif missionQueue.size() <= 0:
        if print_no_missions_log:
            log.create_publish_log('No missions in missionQueue. Awaiting...', 'Flight State Controller - Mission Worker')
            print_no_missions_log = False
    
    return False


altitudeLandedLimit  = 2.0
droneState = "On_Ground"
def set_drone_state(missionQueue):
    global droneState, droneStatePub

    droneState = 'Flying'
    if missionQueue.size() > 0:
        if missionQueue.get_queue_active():
            current_mission = missionQueue.get()
            cmd = current_mission.cmd
            
            droneState = cmd2State[cmd]
        else:
            cmd = 2
            droneState = cmd2State[cmd]
    else:
        droneState = 'Flying'

        altitude = flc.altitude
        if droneState == 'Flying' or droneState == 'Landed' or droneState == 'On_Ground':
            if altitude > altitudeLandedLimit:
                droneState = 'Flying'
            else:
                droneState = 'On_Ground'

    droneStatePub.publish(droneState)


def check_faults():
    global received_crps_ack, received_crps_ack_mutex

    #error_dict = fault_detection.get_fault_dict()
    error_dict = log.get_error_log_list()
    #if error_dict:
        #if len(error_dict) > 0:
        #    print('Faults detected.', len(error_dict))
            
    return error_dict


# def request_crps_from_squadron():
#     global received_crps_ack, received_crps_ack_mutex
#     global crpsRequestPub

#     #log.create_publish_log('Published CRPS Request to network', 'FlightStateController - No GPS')
#     received_crps_ack_mutex.acquire()
#     crps_ack_count = received_crps_ack
#     received_crps_ack_mutex.release()

#     if crps_ack_count < 1:
#         tele = flc.get_telemetry()
#         crpsRequestPub.publish(tele)
#         log.create_publish_log('Published CRPS Request to network', 'CRPS Request')
#         #crpsRequestPub
#         #Drop last known location
#         #/crps/Request
#     else:
#         print('Already received ack from drone in squadron. Will not publish more requests...')


# def await_crps():
#     global crps_request_in_progress
#     global received_crps_ack, received_crps_ack_mutex
#     await_rate = rospy.Rate(0.1)

#     #print('Awaiting crps from drones in squadron...')
#     #print('Will continue requesting crps_from_squadron')

#     received_crps_ack_mutex.acquire()
#     crps_request_in_progress = True
#     received_crps_ack = 0
#     received_crps_ack_mutex.release()

#     continue_crps = False

#     while not rospy.is_shutdown() and check_faults() and crps_request_in_progress:       
#         request_crps_from_squadron()

#         await_rate.sleep()

#         faults = check_faults()
#         telemetry_source = flc.get_telemetry_source()

#         print('telemetry_source:', telemetry_source)

#         if 'crps' in telemetry_source.lower():
#             #print('Received crps from squadron')
#             log.create_publish_log('Received CRPS from squadron', 'CRPS Request')
#             continue_crps = True

#             received_crps_ack_mutex.acquire()
#             crps_request_in_progress = False
#             received_crps_ack = received_crps_ack + 1
#             received_crps_ack_mutex.release()

#         if not faults:
#             log.create_publish_log('GPS fault Cleared itself. Will continue normally', 'CRPS Request')
#             continue_crps = False

#             received_crps_ack_mutex.acquire()
#             crps_request_in_progress = False
#             received_crps_ack = 0
#             received_crps_ack_mutex.release()

#     print('await_crps', 'continue_crps:', continue_crps)

#     return continue_crps
        

# def squadron_crps_request_CB(tele_roque):
#     #check if drone is crps capable
#     if not check_faults():

#         pauseMission()

#         #go to drone
#         if not flc.get_crps_state():
#             #print('Got CRPS request from drone in squadron')

#             log.create_publish_log('Received CRPS request from drone in squadron', 'CRPS Request-Received')

#             print('\t', round(tele_roque.latitude,4), '\n\t', round(tele_roque.longitude,4), \
#                 '\n\t', round(tele_roque.altitude,4), '\n\t', round(tele_roque.heading,4))

#             flc.set_crps_state(True)
#         else:
#             print('CRPS already enabled')

#         crpsRequestAckPub.publish(flc.get_telemetry())


# received_crps_ack = 0
# received_crps_ack_mutex = threading.Lock()
# def squadron_crps_request_Ack_CB(crps_ack):
#     global received_crps_ack, received_crps_ack_mutex
#     # uint32 rostime_secs
#     # uint32 rostime_nsecs
#     # uint32 flightTimeSecs
#     # uint32 batteryThreashold
#     # uint32 batteryPercentage
#     # uint32 gpsSignal
#     # uint32 satelliteNumber
#     # float32 altitude
#     # float32 heading
#     # float64 velocity
#     # float64 longitude
#     # float64 latitude
#     # float64 homeLatitude
#     # float64 homeLongitude
#     # string droneState
#     # float32 gimbalAngle
#     # string serialVersionUID
#     ack_drone_name = crps_ack.serialVersionUID
#     ack_drone_longitude = crps_ack.longitude
#     ack_drone_latitude = crps_ack.latitude
    
#     log.create_publish_log('Received CRPS request from drone('+ack_drone_name+') in squadron.', 'CRPS Request-Received')

#     received_crps_ack_mutex.acquire()
#     received_crps_ack = received_crps_ack + 1
#     received_crps_ack_mutex.release()


fault_resolution_in_progress = False
def fault_detection_response(faults_detected):
    global received_crps_ack, received_crps_ack_mutex
    global fault_resolution_in_progress

    gps_signal_fault = False
    #faults_detected = dict(faults_detected)         # dict() forces a copy, avoiding runtime size change
    if not faults_detected:
        faults_detected = {}

    faults_detected = list(faults_detected)
     
    if len(faults_detected) > 0:
        for fault_message in faults_detected:
            
            #fault = fault_detection.fault_dictionary[fault_message]
            #print(fault_message)

            #print('fault_message:', fault_message)
            #print('flightStateController -', 'Detected Fault:', fault)

            if fault_message.error_id == 100:
                if not crps.get_crps_request_state() and not fault_resolution_in_progress:
                    log.create_publish_log('Found GPS Fault - Starting CRPS procedure', 'Fault='+str(fault_message.message))
                    pauseMission()

                    crps.set_crps_request_state(True)
                    fault_resolution_in_progress = True
                # else:
                #     print('crps.get_crps_request_state() - Active... Checking if got data')

                #     tele = flc.get_telemetry()
                #     tele_source = flc.get_telemetry_source()

                #     print('tele_source:', tele_source)
                #     if not fault_resolution_in_progress:
                #         if 'crps' in tele_source:
                #             if 'crps' in tele.serialVersionUID:
                #                 print('Actively receiving CRPS')
                #                 print('Will notify FlightStateController')
                                
                #                 gps_signal_fault_response()


                    


                # if continue_crps:
                #     gps_signal_fault_response()
                # else:
                #     print("CRPS Issue Fixed")
                    #pass
    else:
        if fault_resolution_in_progress:
            log.create_publish_log('GPS Fault Resolved - Stopping CRPS procedure', 'No Fault')
            #pauseMission()

            crps.set_crps_request_state(False)
            fault_resolution_in_progress = False


    #ask for crps
    #await for crps
    #determine optimal course of action
    #store task
    #perform fault detection response
    #if fault is ok, continue task


fault_response_thread = None
fault_response_thread_running = False
def set_fault_response_thread_state_CB(set_state_enable):
    global fault_response_thread_state_pub, fault_response_thread, fault_response_thread_running

    set_state_enable = set_state_enable.data

    print('set_fault_response_thread_state_CB - set_state_enable:', set_state_enable)

    if set_state_enable:
        if not fault_response_thread_running:
            log.create_publish_log('gps_signal_fault_response starting', 'fault_response_procedure')
            fault_response_thread_running = True

            fault_response_thread = threading.Thread(target=gps_signal_fault_response, args=())
            fault_response_thread.start()

            fault_response_thread_state_pub.publish(fault_response_thread_running)
    elif not set_state_enable:
        fault_response_thread_running = False
        fault_response_thread.join()

        fault_response_thread_state_pub.publish(fault_response_thread_running)

        log.create_publish_log('gps_signal_fault_response stopped', 'fault_response_procedure')


def gps_signal_fault_response():
    global fault_response_index
    global missionQueue
    global fault_resolution_in_progress

    log.create_publish_log('gps_signal_fault_response starting', 'Got CRPS Data')

    pauseMission()
    response_string = 'RTH'
    cmd2FaultResponse = ['RTH', 'Continue Mission', 'Halt', 'Land-in-Place']

    print('gps_signal_fault_response() - cmd2FaultResponse:', cmd2FaultResponse[fault_response_index])

    if cmd2FaultResponse[fault_response_index] == 'RTH':
        log.create_publish_log('Starting RTH', 'gps_signal_response')
        lz_landing_mission = DroneMission(missionMsg=None, thread_target=flLogic.vtol.monitored_landing)

        rth_mission = DroneMission(thread_target=flLogic.traverseRoute)
        home_wp = flc.get_home_position()
        home_wp.altitude = 40.0

        print('home_wp:', str(home_wp))

        rth_mission.push_waypoint(home_wp)
        
        flLogic.set_velocity_range((-2.0, 3.0))
        missionQueue.dump_queue()
        
        #missionQueue.put_first(lz_landing_mission)
        missionQueue.put_first(rth_mission)
        #missionQueue.put_first(lz_landing_mission)
        
        #publish_task_list()
        resumeMission()
        log.create_publish_log('Got CRPS, going home', 'gps_signal_fault')
    # elif cmd2FaultResponse[fault_response_index] == 'Continue Mission':
    #     log.create_publish_log('Got CRPS, continuing mission', 'gps_signal_response')
    #     resumeMission()
    # elif cmd2FaultResponse[fault_response_index] == 'Halt':
    #     log.create_publish_log('Got CRPS, continuing mission', 'gps_signal_response')
    #     missionQueue.dump_queue()
    # elif cmd2FaultResponse[fault_response_index] == 'Land-in-Place':
    #     lz_landing_mission = DroneMission(missionMsg=None, thread_target=flLogic.vtol.monitored_landing())
    #     missionQueue.put_first(lz_landing_mission)
    #     resumeMission()

    r = rospy.Rate(1)
    #while missionQueue.size() > 0 and True:
    while True:
        #print('missionQueue.size():', missionQueue.size())
        #print('missionQueue.get():', missionQueue.get())
        r.sleep()

    flLogic.reset_velocity_range()
    log.create_publish_log('gps_signal_fault_response finished', 'Got CRPS Data')
    fault_resolution_in_progress = False
    fault_response_thread_state_pub.publish(fault_resolution_in_progress)
    publish_task_list()
    

# Worker responsible for activating tasks and checking for faults
def MissionWorker():
    global missionQueue, missionQueue_completed
    global fault_resolution_in_progress

    mission_rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        faults_detected = check_faults()
        
        if not faults_detected:
            if fault_resolution_in_progress:
                fault_detection_response(faults_detected)

            task_finished = check_task_completion(missionQueue)

            if not rospy.is_shutdown():
                activate_next_task(missionQueue)                       
                set_drone_state(missionQueue)
        else:
            fault_detection_response(faults_detected)

            #if not fault_resolution_in_progress:

        mission_rate.sleep()


def init():
    global logicRate, statePubRate
    global droneStatePub, JoystickActionPub, flightLogicStatePub
    global crpsRequestPub, crpsRequestAckPub
    global missionWorker_thread
    global fault_response_state_pub
    global fault_response_thread_state_pub
    global mission_list_pub

    rospy.init_node(nodename)  

    logicRate = rospy.Rate(25)
    statePubRate = rospy.Rate(1)
    
    # Subscribe to Mission from platform
    rospy.Subscriber(dronename+'/Mission', MissionDji, MissionCommandCB)

    # Mission/Task Status
    rospy.Publisher(dronename+'/flight_controller/fault_response/thread/Get_State_Enable', Bool, queue_size=1, latch=True)
    
    #rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)
    #rospy.Subscriber(dronename+'/WaypointPath', MissionWaypointTask, initializeMission)
    rospy.Subscriber(dronename+'/setMinimumAltitude', Float32, minimumAltitudeCB)
    
    # Publish to flightController
    droneStatePub = rospy.Publisher(dronename+'/DroneState', String, queue_size=10)
    JoystickActionPub = rospy.Publisher(dronename+'/Joystick/PublishJoystickAction', JoystickParams, queue_size=1)
   
    flightLogicStatePub = rospy.Publisher(dronename+'/FlightLogicState', String, queue_size=1)

    # CRPS Squadron Communication
    # rospy.Subscriber('/crps/Request', Telemetry, squadron_crps_request_CB)
    # rospy.Subscriber('/crps/Request_Ack', Telemetry, squadron_crps_request_Ack_CB)

    # crpsRequestPub = rospy.Publisher('/crps/Request', Telemetry, queue_size=1)
    # crpsRequestAckPub = rospy.Publisher('/crps/Request_Ack', Telemetry, queue_size=1)

    # Fault Response
    rospy.Subscriber(dronename+'/flight_controller/fault_response/Set_Fault_Response', Int32, fault_response_CB)
    fault_response_state_pub = rospy.Publisher(dronename+'/flight_controller/fault_response/Current_Fault_Response', String, queue_size=1, latch=True)

    fault_response_state_packet = String()
    fault_response_state_packet.data = cmd2FaultResponse[fault_response_index]
    fault_response_state_pub.publish(fault_response_state_packet)

    rospy.Subscriber(dronename+'/flight_controller/fault_response/thread/Set_State_Enable', Bool, set_fault_response_thread_state_CB)
    fault_response_thread_state_pub = rospy.Publisher(dronename+'/flight_controller/fault_response/thread/Get_State_Enable', Bool, queue_size=1, latch=True)
    fault_response_thread_state_pub.publish(False)

    mission_list_pub = rospy.Publisher(dronename+'/flight_controller/Mission/Mission_List', MissionList, queue_size=1, latch=True)
    mission_list_pub.publish(MissionList())

    flc.init()
    flLogic.init()

    fault_detection.init()
    log.init()
    
    crps.init()

    missionWorker_thread = threading.Thread(target=MissionWorker)
    missionWorker_thread.start()

    print(nodename, '- init() - DONE')


def listener(dji_name = "/matrice300"):
    global dronename, nodename
    
    dronename = dji_name
    nodename = dronename.replace('/', '') + '_flightStateController'
    print(nodename)

    init()
    
    rospy.spin()
    

if __name__=='__main__':
    print('Initializing Flight State Controller...')
    f = open("/var/lib/dbus/machine-id", "r")
    boardId = f.read().strip()[0:4]
    
    dronename = '/matrice300' + '_' + boardId
    listener(dronename)  
