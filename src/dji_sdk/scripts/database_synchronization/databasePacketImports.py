#!/usr/bin/env
# -*- coding: utf-8 -*-
# author: Christos Georgiades
# contact: chr.georgiades@gmail.com
# date: 27-12-2022

# .Msg Imports #
from std_msgs.msg import String, Bool, UInt32, Int8, Float64
from sensor_msgs.msg import BatteryState, Joy

from geometry_msgs.msg import Vector3Stamped, QuaternionStamped

#import os
#ros_version = os.system('rosversion -d')
import subprocess
ros_version = subprocess.check_output('rosversion -d', shell=True).decode('utf-8').strip()
if ros_version == 'melodic':
    from multimaster_msgs_fkie.msg import LinkStatesStamped
elif ros_version == 'noetic':
    from fkie_multimaster_msgs.msg import LinkStatesStamped
    
#from 

#from dji_sdk.msg import BatteryState
from dji_sdk.msg import Quaternion, ControlAuthorityRequest
from dji_sdk.msg import EscData, WindData
from dji_sdk.msg import FlightAnomaly
from dji_sdk.msg import ComponentList

from dji_sdk.msg import ErrorLog, ErrorLogList

from kios.msg import MissionDji, trisonica_msg
from kios.msg import Telemetry, TerminalHardware, DroneHardware, Positioning, DroneMovement, DatabasePacketRequest
from kios.msg import BuildMap
from kios.msg import CrpsRequest, CrpsRequestAckList


