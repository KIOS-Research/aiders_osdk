import sys

import rospy
import inspect

from std_msgs.msg import Int32

from dji_sdk.msg import ErrorLog, ErrorLogList

severity_to_string = ['INFO', 'WARN', 'ERROR', 'DEBUG']
severity_publisher = []

log_id = 0
def log_idCB(log_id_packet):
    global log_id
    log_id = log_id_packet.data


error_log_list = None
def error_log_listCB(error_log_list_packet):
    global error_log_list
    error_log_list = error_log_list_packet.error_log_list

def get_error_log_list():
    global error_log_list
    return error_log_list


def get_caller_source():
    # Use the inspect stack to get information about the calling function
    stack = inspect.stack()   
    # The calling function's information is in the second frame (index 3)
    calling_frame = stack[3]    
    # Extract the calling function's name
    calling_function_name = calling_frame[3]

    source = dronename + '-' + sys.argv[0].split('/')[-1] + '-' + calling_function_name
    
    return source


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
def create_log(message, context=None, error_id=0, severity=0):
    error_log = ErrorLog()
    
    error_log.rostime_secs = rospy.Time.now().secs
    error_log.source = get_caller_source()
    error_log.error_id = error_id
    error_log.severity = severity
    error_log.message = message
    error_log.context = context
    error_log.log_id = log_id

    return error_log


def create_publish_log(message, context=None, error_id=0, severity = 0):
    global log_publisher

    error_log = create_log(message, context, error_id, severity)

    log_publisher.publish(error_log)

    return error_log


def publish_error_log(error_log):
    global log_publisher

    log_publisher.publish(error_log)


def publish_error_log_list(error_log_list):
    global log_publisher

    for error_log in error_log_list:
        publish_error_log(error_log)


def log_to_string(error_log):
    severity_string = '[' + severity_to_string[error_log.severity] + ']'
    source_string = error_log.source
    time_string = '[' + str(error_log.rostime_secs) + ']'
    id_string = '[' + str(error_log.log_id) + ']'
    
    message_context_string = 'Message:\t' + error_log.message + '\nContext:\t' + error_log.context

    log_string = '\n' + severity_string + ' ' + time_string + ' ' + source_string + ' ' + id_string + '\n' + message_context_string + '\n'

    return log_string


def init():
    global log_publisher

    if rospy.is_shutdown():
        print(nodename,'- init()','- rospy is not ready')
    else:
        rospy.Subscriber(dronename+'/Log/Log_ID', Int32, log_idCB)
        rospy.Subscriber(dronename+'/Log/List', ErrorLogList, error_log_listCB)

        log_publisher = rospy.Publisher(dronename+'/Log', ErrorLog, queue_size=10)

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

dronename = 'matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)