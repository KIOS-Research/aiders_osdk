import rospy
import threading

import os, sys

from dji_sdk.msg import telemetry2 
from kios.msg import Telemetry

from std_msgs.msg import String, Int32, Float32, Bool

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/logging_system')
import logging_commons as log
#/matrice300/Telemetry
#/matrice300/fused
#/matrice300/crps

dict_source_priority_ = {}


class TelemetrySource:
    latest_tele = None
    latest_tele_mutex = threading.Lock()

    # for crps use
    latest_gps_tele = None
    latest_gps_tele_mutex = threading.Lock()

    confidence_value = 0.5
    confidence_value_mutex = threading.Lock()
    confidence_window = 10
    confidence_timeframe_min = 3

    def __init__(self, topic, priority, message_type=Telemetry, validate_satellites=False, validate_timestamp=False, expected_frequency=0):
        self.topic = topic
        self.priority = priority
        self.message_type = message_type
        self.validate_satellites = validate_satellites
        self.validate_timestamp = validate_timestamp
        self.expected_frequency = expected_frequency

        if self.expected_frequency > 0:
            self.confidence_window = self.expected_frequency * self.confidence_timeframe_min
        elif self.expected_frequency == 0:
            self.expected_frequency = 10

        self.state_enabled = True
        self.valid = False
        self.setup_subscriber()


    
    def setup_subscriber(self):
        rospy.Subscriber(self.topic, self.message_type, self.telemetryCB)
        rospy.Subscriber(self.topic+'/Set_State_Enable', Bool, self.set_state_enabledCB)
        print('setup_subscriber', self.topic+'/Set_State_Enable')

        if '/crps/' in self.topic:
            rospy.Subscriber(dronename+'/telemetry2', telemetry2, self.telemetryGpsCB)

        if self.validate_timestamp:
            self.pub_rate_validator = threading.Thread(target=self.validate_topic_publishing_rate, args=())
            self.pub_rate_validator.start()

        print('setup_subscriber -', self.topic)


    def telemetryCB(self, tele):
        #print(self.topic, 'got tele')
        if self.state_enabled:
            self.validate_packet(tele)

    
    def telemetryGpsCB(self, tele):
        #print(self.topic, 'got tele')
        if self.state_enabled:
            self.latest_gps_tele_mutex.acquire()
            self.latest_gps_tele = tele
            self.latest_gps_tele_mutex.release()

    
    def set_state_enabledCB(self, target_state):
        print('\n' + self.topic, '- Capture Packets:', target_state.data, '\n')
        self.state_enabled = target_state.data


    def validate_topic_publishing_rate(self):
        expected_frequency = self.expected_frequency * 0.95
        pub_timeframe = 1 / expected_frequency
        conf_increment = 0.1

        pubRate = rospy.Rate(expected_frequency)

        while not rospy.is_shutdown():
            pubRate.sleep()

            latest_tele_valid = False

            if self.latest_tele: 
                if self.validate_packet_timestamp(self.latest_tele, timeframe_sec=pub_timeframe*60.0):
                    latest_tele_valid = True
                    #self.confidence_value = self.confidence_value + conf_increment / self.expected_frequency
                    #print(self.topic, 'self.latest_tele received in time')
            
            if not latest_tele_valid:
                new_confidence = self.change_source_confidence(latest_tele_valid)          


    def validate_packet(self, tele):
        if self.validate_timestamp:
            valid_time = self.validate_packet_timestamp(tele)
        else:
            valid_time = True

        if self.validate_satellites:
            valid_sat = self.validate_packet_satellite(tele)
        else:
            valid_sat = True

        valid_packet = valid_time and valid_sat
        
        new_confidence_value = self.change_source_confidence(valid_packet)

        #print('valid_packet:', valid_packet, 'valid_time:', valid_time, 'valid_sat:', valid_sat)
        #print('confidence_value:', new_confidence_value, 'valid:', self.valid)

        #if valid_packet:
        #    self.set_latest_tele(tele)
        if valid_packet or new_confidence_value > 0.5:
            self.set_latest_tele(tele)

     
    def change_source_confidence(self, valid_packet):
        conf_increment = 0.1
        val = conf_increment if valid_packet else -conf_increment
        #print('val:', val)
        self.confidence_value_mutex.acquire()
        old_confidence_value = self.confidence_value
        self.confidence_value = self.confidence_value + val / self.expected_frequency
        
        if self.confidence_value > 1.0:
            self.confidence_value = 1.0
        elif self.confidence_value < 0.0:
            self.confidence_value = 0.0

        if self.confidence_value > 0.5:
            self.valid = True
        else:
            self.valid = False

        new_confidence_value = self.confidence_value
        self.confidence_value_mutex.release()

        if new_confidence_value > 0.5 and old_confidence_value < 0.5:
            message = 'Aircraft regained ' + self.topic + ' signal.'
            context = 'confidence_value > ' + str(0.5)
            log.create_publish_log(message, context, 100, 0)
        elif new_confidence_value < 0.5 and old_confidence_value > 0.5:
            message = 'Aircraft lost ' + self.topic + ' signal.'
            context = 'confidence_value < ' + str(0.5)
            log.create_publish_log(message, context, 100, 1)

        #print(self.topic, 'Confidence:', self.confidence_value, 'Packet Valid:', valid_packet)
        return new_confidence_value


    def validate_packet_timestamp(self, tele, timeframe_sec = 120.0):
        telemetryTimeoutMin = timeframe_sec / 60.0
        valid_time = False
        now = rospy.Time.now()

        minutes_diff = (now.secs - tele.rostime_secs) / 60.0
        if minutes_diff > telemetryTimeoutMin:
            valid_time = False
        else:
            valid_time = True

        #print(self.topic, 'Packet age:\t', minutes_diff)

        return valid_time

    def validate_packet_satellite(self, tele):
        valid_sat = False
        if not self.validate_satellites:
            return True
        else:
            gpsSignal = tele.gpsSignal
            satelliteNumber = tele.satelliteNumber
 
            #print(self.topic, 'gpsSignal:', gpsSignal, 'satelliteNumber:', satelliteNumber)

            if satelliteNumber > 8:
                valid_sat = True
            else:
                valid_sat = False

        return valid_sat


    def set_latest_tele(self, tele):
        self.valid = True
        
        tele.serialVersionUID = tele.serialVersionUID + self.topic
        vel = tele.velocity

        if 'e' in str(vel).lower():
            tele.velocity = 0.0

        if '/crps/' in self.topic:
            self.latest_gps_tele_mutex.acquire()
            latest_gps_tele = self.latest_gps_tele
            self.latest_gps_tele_mutex.release()

            tele.altitude = latest_gps_tele.altitude
            tele.heading = latest_gps_tele.heading

        self.latest_tele_mutex.acquire()
        self.latest_tele = tele
        self.latest_tele_mutex.release()

    def get_latest_tele(self):
        self.latest_tele_mutex.acquire()
        tele_packet = self.latest_tele
        self.latest_tele_mutex.release()

        return tele_packet


def setup_ps_topic(topic, priority, message_type=Telemetry, validate_satellites=False, validate_timestamp=False, expected_frequency=0):
    global dict_source_priority_

    if priority in dict_source_priority_:
        print('There already exists a telemetry source with priority', priority)
    else:
        tele_source = TelemetrySource(topic, priority, message_type, validate_satellites, validate_timestamp, expected_frequency)
        dict_source_priority_[priority] = tele_source


def find_highest_priority_valid_tele():
    global dict_source_priority_
    
    if len(dict_source_priority_):
        for i in range(len(dict_source_priority_)):
            if dict_source_priority_[i].valid:
                return dict_source_priority_[i]

        return dict_source_priority_[0]

    return None


def get_valid_telemetry():
    tele_packet = None

    tele_source = find_highest_priority_valid_tele()
    if tele_source and tele_source.valid:
        tele_packet = tele_source.get_latest_tele()

    return tele_packet, tele_source

  
def psp_telemetry_worker(frequency):
    global gpsTelemetry, dronename
    
    publish_rate = rospy.Rate(frequency)
    tele_publisher_worker = rospy.Publisher(dronename+'/PSP/Telemetry', Telemetry, queue_size=1, latch=True)
    source_publisher_worker = rospy.Publisher(dronename+'/PSP/Source', String, queue_size=1, latch=True)
    priority_publisher_worker = rospy.Publisher(dronename+'/PSP/Priority', Int32, queue_size=1, latch=True)
    confidence_publisher_worker = rospy.Publisher(dronename+'/PSP/Confidence', Float32, queue_size=1, latch=True)
    
    print('psp_telemetry_worker starting up:', frequency, 'hz')

    while not rospy.is_shutdown():
        tele_packet, tele_source = get_valid_telemetry()
        if tele_packet:
            tele_publisher_worker.publish(tele_packet)

        source_publisher_worker.publish(tele_source.topic)
        priority_publisher_worker.publish(tele_source.priority)
        confidence_publisher_worker.publish(tele_source.confidence_value)

        publish_rate.sleep()


def init():
    log.init()

    setup_ps_topic(dronename+'/telemetry2', priority=0, message_type=telemetry2, validate_satellites=True, validate_timestamp=True, expected_frequency=50)
    setup_ps_topic('/crps/Telemetry', priority=1)

    workerPSP = threading.Thread(target=psp_telemetry_worker, args=(1, ))
    workerPSP.start()

    print(nodename, 'Initialization DONE')


def listener(dji_name = "matrice300", as_module = False):
    global boardId, translator, home_latitude, home_longitude
    global dronename, positioningPub
    global nodename

    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename + '_psp'
    print('TelemetryTranslator boardId:', boardId)

    if not as_module:
        rospy.init_node(nodename)
        init()

    print('PSP READY')

    if not as_module:
        rospy.spin()


try:
    print('Initializing postitioning_system_priority')
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