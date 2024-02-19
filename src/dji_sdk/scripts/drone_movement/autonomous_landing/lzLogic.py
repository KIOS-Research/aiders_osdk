import time

import rospy

from std_msgs.msg import Header, String
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Point


from dji_sdk.msg import Resolution, AruQrDetection
from dji_sdk.msg import JoystickParams

from kios.msg import Telemetry

import autonomous_landing.vtolLogic as vtol
from simple_pid import PID

controllerFrequency = 50
sampleTime = 1 / controllerFrequency
pidDistX = PID(Kp=0.004, Ki=0.0, Kd=0.0, sample_time=sampleTime, output_limits=(-1.5, 1.5))
pidDistX.setpoint = 0    # value we are trying to achieve
pidDistX(0)              # value we read

pidDistY = PID(Kp=0.004, Ki=0.0, Kd=0.0, sample_time=sampleTime, output_limits=(-1.5, 1.5))
pidDistY.setpoint = 0    # value we are trying to achieve
pidDistY(0)              # value we read

pidAlt = PID(Kp=1.0, Ki=0.0, Kd=0.2, sample_time=sampleTime, output_limits=(-1.0, 1.0))
pidAlt.setpoint = 0    # value we are trying to achieve
pidAlt(0)              # value we read




def telemetryCB(tele):
    global altitude, longitude, latitude, yaw
    altitude = tele.altitude
    longitude = tele.longitude
    latitude = tele.latitude
    yaw = tele.heading

def publishAction(x, y, z, yaw):
    actionParams = JoystickParams()
    actionParams.x = x
    actionParams.y = y
    actionParams.z = z
    actionParams.yaw = yaw
    
    #print(x, y, z, yaw)    
    #response = joystickActionSrv(actionParams)
    
    JoystickActionPub.publish(actionParams)


# Header header
# geometry_msgs/Point img_center
# geometry_msgs/Point landing_zone_center
# float32 landing_zone_error
img_center = None
lz_center = None
lz_error = 0.0

lz_error_threshold = 0.0
lz_error_threshold_percentage = 0.1
def MarkerDetectionCB(marker_detection):
    global img_center, lz_center, lz_error_threshold
    global lz_error

    img_center = (marker_detection.img_center.x, marker_detection.img_center.z)
    lz_center = (marker_detection.landing_zone_center.x, marker_detection.landing_zone_center.z)
    lz_error = marker_detection.landing_zone_error

    if lz_error_threshold == 0.0:
        lz_error_threshold = img_center[1] * lz_error_threshold_percentage
        print('lz_error_threshold set to:', lz_error_threshold, '[m]')


lz_landing_timeout = 360
lz_landing_timeout_print_every = 200
def lz_landing():
    print('starting lz_landing')

    start_time = time.time()
    cur_time = time.time()
    elapsed_time = cur_time - start_time
    c = 0
    #awaitLzCenterRate = rospy.Rate(controllerFrequency)
    centerLzRate = rospy.Rate(controllerFrequency)
    while (not img_center or not lz_center) and elapsed_time < lz_landing_timeout and not rospy.is_shutdown():
        cur_time = time.time()
        elapsed_time = cur_time - start_time
        elapsed_time_formatted = "{:.6f}".format(float(elapsed_time))
        if c >= lz_landing_timeout_print_every:
            print('Awaiting Detection...', 'elapsed_time:', elapsed_time_formatted)
            c = 0
        c = c + 1
        centerLzRate.sleep()

    c = 0

    landingAlt = 10.0
    landingAlt_error_thresh = 0.1
    if img_center and lz_center:
        pidDistX.setpoint = img_center[0]
        pidDistY.setpoint = img_center[1]
        pidAlt.setpoint = landingAlt


        while (lz_error > lz_error_threshold or abs(altitude - landingAlt) > landingAlt_error_thresh) and not rospy.is_shutdown():
            distX = pidDistX(lz_center[0])
            distY = pidDistY(lz_center[1]) 
            distAlt = pidAlt(altitude)   
            #distAlt = 0

            if c >= lz_landing_timeout_print_every:
                print('Correcting over landing zone:', round(distX, 3), round(distY, 3), round(distAlt, 3))
                print('\tpidDistX:', lz_center[0], '/', pidDistX.setpoint)
                print('\tpidDistY:', lz_center[1], '/', pidDistY.setpoint)
                print('\tpidAlt:', round(altitude, 2), '/', pidAlt.setpoint)
                c = 0
                
            c = c + 1

            publishAction(distY, -distX, distAlt, yaw)
            centerLzRate.sleep()
            #print('lz_landing running')
        
        print('lz_error:', round(lz_error, 2), '[pixels]', 'lz_error_threshold:', lz_error_threshold, '[pixels]')
        print('altitude:', round(altitude, 2), '[m]')
        print('calling monitored_landing')
        vtol.monitored_landing()
        return True
    else:
        print('Failed to make detection. Exiting...')
        return False


def init():
    global vtolSrv
    global JoystickActionPub

    rospy.Subscriber(dronename+'/aruqr/detections', AruQrDetection, MarkerDetectionCB)
    rospy.Subscriber(dronename+'/Telemetry', Telemetry, telemetryCB)

    JoystickActionPub = rospy.Publisher(dronename+'/PublishJoystickAction', JoystickParams, queue_size=1)

    vtol.init()

    print(nodename, '- init() - DONE')
    

def listener(dji_name = "/matrice300", as_module=False):
    global dronename, nodename
    
    dronename = dji_name
    nodename = dronename.replace('/', '') + '_lz'
    print('VTOL Handler', nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


try:
    print('Initializing LZ logic')
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