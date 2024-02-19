import threading

from dji_sdk.msg import MissionWaypoint, MissionWaypointTask


class DroneMission():
    def __init__(self, missionMsg=None, thread_target=None):
        self.missionMsg = missionMsg
        
        if missionMsg:
            response = parse_Mission(missionMsg)
            self.mission_waypoint = response.mission_waypoint
            self.waypoint_index_max = len(self.mission_waypoint) - 1

            self.cmd = self.missionMsg.missionCommand.missionCommand
        else:
            self.mission_waypoint = []
            self.waypoint_index_max = 0

            self.cmd = -1
        
        self.waypoint_index = 0
        self.waypoint_finished = False

        self.mission_thread = None
        self.mission_active = False
        self.thread_target = thread_target

        
       
    def get_waypoint_index(self):
        return self.waypoint_index

    def get_waypoint_index_max(self):
        return self.waypoint_index_max

    def update_waypoint_index(self, mission_waypoint_index):
        #print('old mission_waypoint_index:', self.waypoint_index)
        
        if mission_waypoint_index < 0:
            self.waypoint_index = 0
        elif mission_waypoint_index > self.waypoint_index_max:
            self.waypoint_index = self.waypoint_index_max
            self.waypoint_finished = True
        else:
            self.waypoint_index = mission_waypoint_index

        #print('new mission_waypoint_index:', mission_waypoint_index)

    def set_waypoint_index(self, mission_waypoint_index):
        self.update_waypoint_index(mission_waypoint_index)

    def increment_waypoint_index(self):
        self.update_waypoint_index(self.waypoint_index + 1)

    def get_next_waypoint(self):
        return self.mission_waypoint[self.waypoint_index]

    def get_mission_active(self):
        return self.mission_active

    def get_waypoint_finished(self):
        return self.waypoint_finished

    def get_task_finished(self):
        return self.waypoint_finished

    def set_task_finished(self, task_finished_value):
        self.waypoint_finished = task_finished_value
    
    def push_waypoint(self, wp):
        self.mission_waypoint.append(wp)
        self.waypoint_index_max = len(self.mission_waypoint) - 1
    

    def set_mission_active(self, mission_active, thread_target=None, thread_args=None):
        if mission_active == False:
            self.mission_active = mission_active
            if self.mission_thread:
                self.mission_thread.join()
        elif mission_active == True and not self.mission_active:
            if thread_target:
                self.thread_target = thread_target
            
            if self.thread_target:
                print('self.mission_thread.start() -', 'thread_target:', str(self.thread_target))

                self.mission_active = mission_active
                
                self.mission_thread = threading.Thread(target=self.thread_target, args=(self, thread_args,))
                self.mission_thread.start()
            
            else:
                print('No thread_target specified')


def parse_Mission(mission):
    global responsePub

    response = MissionWaypointTask()    
    response.velocity_range = 10.0
    response.idle_velocity = 5.0
    
    response.action_on_finish = MissionWaypointTask.FINISH_NO_ACTION
    response.mission_exec_times = 1
    
    response.yaw_mode = MissionWaypointTask.YAW_MODE_AUTO
    response.trace_mode = MissionWaypointTask.TRACE_POINT

    response.action_on_rc_lost = MissionWaypointTask.ACTION_AUTO
    response.gimbal_pitch_mode = MissionWaypointTask.GIMBAL_PITCH_FREE
    
    response.mission_waypoint = []       
        
    for wp in mission.gpsInput:
        waypoint = MissionWaypoint()  
        
        waypoint.latitude = wp.latitude
        waypoint.longitude = wp.longitude
        waypoint.altitude = wp.altitude
        
        waypoint.damping_distance = 0
        waypoint.target_yaw = 0
        waypoint.target_gimbal_pitch = -900
                  
        waypoint.turn_mode = 1
        
        waypoint.has_action = 0
        waypoint.action_number = 2
        waypoint.action_time_limit = 100
        
        wait_time = 60
        waypoint.waypoint_action.action_repeat = 1 #total running times 1 (upper 4 bits) => 16 + total number of actions (lower 4 bits) => 2 ---> total = 16 + 2 = 18
        waypoint.waypoint_action.command_list = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        waypoint.waypoint_action.command_parameter = [0,wait_time,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        
        #print "lat: ", waypoint.latitude , " , long: ", waypoint.longitude, " , alt: ", waypoint.altitude , " , wait: ", wait_time
        response.mission_waypoint.append(waypoint)

    #print('Publishing Mission to drone - flightLogic')
    #initializeMission(response.mission_waypoint)
    return response
        