import rospy

from dji_sdk.msg import JoystickParams
from dji_sdk.srv import FlightTaskControl

from kios.msg import Telemetry


#from autonomous_landing import stereoHandler
#from autonomous_landing.landing_pad_detection import qr_detection

#constant for tasks
# uint8 TASK_GOHOME = 1
# uint8 TASK_POSITION_AND_YAW_CONTROL   = 2
# uint8 TASK_GOHOME_AND_CONFIRM_LANDING = 3
# uint8 TASK_TAKEOFF = 4
# uint8 TASK_VELOCITY_AND_YAWRATE_CONTROL = 5
# uint8 TASK_LAND          = 6
# uint8 START_MOTOR        = 7
# uint8 STOP_MOTOR         = 8
# uint8 TASK_EXIT_GO_HOME  = 12
# uint8 TASK_EXIT_LANDING  = 14
# uint8 TASK_FORCE_LANDING_AVOID_GROUND = 30 #/*!< confirm landing */
# uint8 TASK_FORCE_LANDING              = 31 #/*!< force landing */

# #request
# uint8 task    # see constants above for possible tasks
# JoystickParams joystickCommand  #Provide Position and Velocity control
# uint32 velocityControlTimeMs    #Velocity control time
# float32 posThresholdInM  #(Meter)
# float32 yawThresholdInDeg  #(Degree)
# ---
# #response
# bool result
altitude = 0.0
altitude_prev = 0.0
altitude_change = 0.0
def telemetryCB(tele):
    global altitude, altitude_prev, longitude, latitude, yaw
    altitude_prev = altitude
    altitude = tele.altitude
    altitude_change = altitude - altitude_prev

    longitude = tele.longitude
    latitude = tele.latitude
    yaw = tele.heading


def call_vtolSrv(request):
    print("takeoff_request:", request)
    response = vtolSrv(request.task, request.joystickCommand, request.velocityControlTimeMs,
                       request.posThresholdInM, request.yawThresholdInDeg)
    print("vtolSrv_response:", response)

    return response


def monitored_takeoff():
    global vtolSrv
    
    takeoff_request = FlightTaskControl()
    takeoff_request.task = 4
    takeoff_request.joystickCommand = JoystickParams()

    takeoff_request.velocityControlTimeMs = 100
    takeoff_request.posThresholdInM = 1.0
    takeoff_request.yawThresholdInDeg = 1.0
    
    takeoff_response = False
    takeoff_tries = 5
    takeoff_attempt_rate = rospy.Rate(0.5)
    while not takeoff_response and takeoff_tries > 0: #and not altitude_change > 0:            
        try:
            print("Calling Take Off")
            return call_vtolSrv(takeoff_request)
        except Exception as e:
            print("monitored_takeoff - Caught Exception:", e)
    
        takeoff_tries = takeoff_tries - 1
        takeoff_attempt_rate.sleep()


def monitored_landing():
    global vtolSrv
    
    landing_request = FlightTaskControl()
    landing_request.task = 31
    landing_request.joystickCommand = JoystickParams()

    landing_request.velocityControlTimeMs = 100
    landing_request.posThresholdInM = 1.0
    landing_request.yawThresholdInDeg = 1.0

    landing_response = False
    landing_tries = 5
    landing_attempt_rate = rospy.Rate(0.5)
    while not landing_response and landing_tries > 0: #and not altitude_change < 0:            
        try:
            print("Calling Landing")
            return call_vtolSrv(landing_request)
        except Exception as e:
            print("monitored_landing - Caught Exception:", e)
    
        landing_tries = landing_tries - 1
        landing_attempt_rate.sleep()


def init():
    global vtolSrv

    rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)

    rospy.wait_for_service(dronename+'/flight_task_control')
    vtolSrv=rospy.ServiceProxy(dronename+'/flight_task_control', FlightTaskControl)

    print(nodename, '- init() - DONE')


def listener(dji_name = "/matrice300", as_module=False):
    global dronename, nodename
    
    dronename = dji_name

    nodename = dronename.replace('/', '') + '_vtol'
    print('VTOL Handler', nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


try:
    print('Initializing VTOL logic')
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
