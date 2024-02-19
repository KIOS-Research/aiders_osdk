##!/usr/bin/env python
#
## Author: Christos
## Email: cgeorg15@ucy.ac.cy
## Date :07/07/2022
#
## license removed for brevity
#
#import rospy
#from std_msgs.msg import String, Bool
#from dji_sdk.msg import ScanArea, MissionWaypoint, MissionWaypointTask, WaypointV2MissionStatePush
#from kios.msg import Telemetry, MissionDji, GpsInput, MissionCommandDJI
#
#from dji_sdk.srv import InitWaypointV2Setting, StartWaypointV2Mission, SetupCameraStream, SetupCameraH264, SubscribeWaypointV2State, StopWaypointV2Mission, PauseWaypointV2Mission, ResumeWaypointV2Mission
#
#responsePub=None
#
#droneState='Landed'
#
#missionStateSub = None
#droneStatePub = None
#startMissionService = None
#
#dronename = None
#
#
##def MissionStateCB(missionState): 
##    global droneState    
##    #print missionState
##    
##    if int(missionState.state) is 0 and int(missionState.curWaypointIndex) is 0:
##        if droneState == 'In_Mission':
##            missionStateSub.unregister()
##            finishMission()
##            
##            
##def setupMissionStateCallback():
##    global missionStateSub   
##    
##    pollMissionStateService = rospy.ServiceProxy(dronename+'/waypointV2_subscribeMissionState', SubscribeWaypointV2State)  
##    response = pollMissionStateService(1)
##    print(response)
##    
##    missionStateSub = rospy.Subscriber(dronename+'/waypointV2_mission_state', WaypointV2MissionStatePush, MissionStateCB)
#
#
#def MissionCommandCB(missionMsg):
#    cmd = missionMsg.missionCommand.missionCommand
#    print('========')
#    print('cmd:', cmd)
#    print(missionMsg)
#    print('========')
#     
#    if cmd == 0:
#        print('start')
#        pushMission2Drone(missionMsg)
#        droneStatePub.publish('In_Mission')
#    elif cmd == 1:    # stop
#        print('stop')
#        #fc.stopMission()
#        droneStatePub.publish('Flying')
#    elif cmd == 2:    # pause
#        print('pause')
#        #fc.pauseMission()
#        droneStatePub.publish('Paused_Mission')
#    elif cmd == 3:    # resume
#        print('resume')
#        #fc.resumeMission()
#        droneStatePub.publish('In_Mission')
#
#
#def pushMission2Drone(mission):
#    global responsePub
#    
#    response = MissionWaypointTask()
#    
#    response.velocity_range = 10.0
#    response.idle_velocity = 5.0
#    
#    response.action_on_finish = MissionWaypointTask.FINISH_NO_ACTION
#    response.mission_exec_times = 1
#    
#    response.yaw_mode = MissionWaypointTask.YAW_MODE_AUTO
#    response.trace_mode = MissionWaypointTask.TRACE_POINT
#
#    response.action_on_rc_lost = MissionWaypointTask.ACTION_AUTO
#    response.gimbal_pitch_mode = MissionWaypointTask.GIMBAL_PITCH_FREE
#    
#    response.mission_waypoint = []       
#        
#    for wp in mission.gpsInput:
#        waypoint = MissionWaypoint()  
#        
#        waypoint.latitude = wp.latitude
#        waypoint.longitude = wp.longitude
#        waypoint.altitude = wp.altitude
#        
#        waypoint.damping_distance = 0
#        waypoint.target_yaw = 0
#        waypoint.target_gimbal_pitch = -900
#                  
#        waypoint.turn_mode = 1
#        
#        waypoint.has_action = 0
#        waypoint.action_number = 2
#        waypoint.action_time_limit = 100
#        
#        wait_time = 60
#        waypoint.waypoint_action.action_repeat = 1 #total running times 1 (upper 4 bits) => 16 + total number of actions (lower 4 bits) => 2 ---> total = 16 + 2 = 18
#        waypoint.waypoint_action.command_list = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#        waypoint.waypoint_action.command_parameter = [0,wait_time,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#        
#        #print "lat: ", waypoint.latitude , " , long: ", waypoint.longitude, " , alt: ", waypoint.altitude , " , wait: ", wait_time
#        response.mission_waypoint.append(waypoint)
#        #print("\n")
#        
#    #fc.initializeMission(response.mission_waypoint)
#    #return
#     
#    print('Publishing Mission to drone - DJI')
#    responsePub.publish(response) 
#    
#
#def finishMission():
#    global droneStatePub
#    
#    missionMsg = 'Flying'
#    
#    print('Mission Finished')
#    
#    #droneStatePub = rospy.Publisher("/matrice300/DroneState", String, queue_size=2)
#    droneStatePub.publish(missionMsg)
#    
#    cameraStatePub = rospy.Publisher(dronename+'/BuildMapRequest', Bool, queue_size=2)
#    cameraStatePub.publish(False)
#
#      
#def DroneStateCB(state):
#    global droneState
#    droneState = str(state.data)
#   
#
#def listener(dji_name = "/matrice300"):
#    global responsePub, droneStatePub, startMissionService
#    global dronename
#    
#    dronename = dji_name  
#    nodename = dronename.replace('/', '') + '_missionHandler'
#    print(nodename)
#    
#    rospy.init_node(nodename, anonymous=False)    
#    #fc.listener(dji_name = dronename)
#    
#    # Subscribe to Mission from platform
#    rospy.Subscriber(dji_name+'/Mission', MissionDji, MissionCommandCB)
#    #responsePub = rospy.Publisher(dji_name+'/ScanAreaResponse', MissionWaypointTask, queue_size=2)
#    responsePub = rospy.Publisher(dji_name+'/WaypointPath', MissionWaypointTask, queue_size=1)
#
#    # Dronestate
#    rospy.Subscriber(dji_name+'/DroneState', String, DroneStateCB)
#    droneStatePub = rospy.Publisher(dji_name+'/DroneState', String, queue_size=2)
#    
#    # DJI Mission Service
#    startMissionService = rospy.ServiceProxy(dji_name+'/waypointV2_startMission', StartWaypointV2Mission)
#    
#    #rospy.spin()
#    rate = rospy.Rate(50)
#    while not rospy.is_shutdown():
#        rate.sleep()
#        
#       
#if __name__=='__main__':
#    print('Initializing Mission Handler...')
#    f = open("/var/lib/dbus/machine-id", "r")
#    boardId = f.read().strip()[0:4]
#    
#    dronename = '/matrice300' + '_' + boardId
#    listener(dronename)
