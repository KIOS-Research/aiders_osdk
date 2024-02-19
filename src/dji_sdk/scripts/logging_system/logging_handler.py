#!/usr/bin/env python

# Author: Christos Georgiades
# Email: chr.georgiades@gmail.com
# Date : 01/09/2023
#
# Logging System for Matrice300

import sys

import rospy

from std_msgs.msg import Int32

from dji_sdk.msg import ErrorLog, ErrorLogList

severity_to_string = ['INFO', 'WARN', 'ERROR', 'DEBUG']
severity_to_publisher = []

log_id = 0

#uint32 rostime_secs
#string source
#uint32 severity		# 0 - INFO, 1 - WARN, 2 - ERROR, 3 - DEBUG
#string message
#string context
#uint32 log_id
#
# [severity: <severity>][source: <source>][ros_time: <rostime_secs>][log_id: <log_id>]
# Message: <message>
# Context: <context>
def error_logCB(error_log):
    global log_id, log_id_pub
    
    log_id = log_id + 1
    log_id_pub.publish(log_id)

    severity_string = '[' + severity_to_string[error_log.severity] + ']'
    source_string = error_log.source
    time_string = '[' + str(error_log.rostime_secs) + ']'
    id_string = '[' + str(error_log.log_id) + ']'
    
    message_context_string = 'Message:\t' + error_log.message + '\nContext:\t' + error_log.context

    log_string = '\n' + severity_string + ' ' + time_string + ' ' + source_string + ' ' + id_string + '\n' + message_context_string + '\n'

    if error_log.severity < 4:
        print(log_string)
    else:
        print(log_string, file=sys.stderr)

    severity_to_publisher[error_log.severity].publish(error_log)
    update_error_log_list(error_log)



error_dict = {}
def update_error_log_list(error_log):
    global error_log_listPub

    error_id = error_log.error_id

    #print('error_id:', error_id, 'error_log.severity:', error_log.severity)
        
    if error_log.severity > 0:        
        error_dict[error_id] = error_log
    else:
        if error_id in error_dict:
            del error_dict[error_id]

    error_log_list_packet = ErrorLogList()
    error_log_list = list(error_dict.values())

    error_log_list_packet.error_log_list = error_log_list
    error_log_listPub.publish(error_log_list_packet)


def init():
    global log_id, log_id_pub
    global error_log_listPub

    if rospy.is_shutdown():
        print(nodename,'- init()','- rospy is not ready')
    else:
        rospy.Subscriber(dronename+'/Log', ErrorLog, error_logCB)
        
        severity_to_publisher.append(rospy.Publisher(dronename+'/Log/Info', ErrorLog, queue_size=10))
        severity_to_publisher.append(rospy.Publisher(dronename+'/Log/Warning', ErrorLog, queue_size=10))
        severity_to_publisher.append(rospy.Publisher(dronename+'/Log/Error', ErrorLog, queue_size=10))
        severity_to_publisher.append(rospy.Publisher(dronename+'/Log/Debug', ErrorLog, queue_size=10))

        error_log_listPub = rospy.Publisher(dronename+'/Log/List', ErrorLogList, queue_size=1, latch=True)

        log_id_pub = rospy.Publisher(dronename+'/Log/Log_ID', Int32, queue_size=1)
        log_id_pub.publish(log_id)

        print(nodename,'- init()','- DONE')


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, nodename

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_logging_handler'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Fault Detection...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)