# Author: Christos Georgiades
# Date: 2023-10-26
# Collaboration Handler for CRPS

import os, sys

import rospy

from std_msgs.msg import Bool
from kios.msg import CrpsRequest, CrpsRequestAckList

import threading

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/drone_movement/')
import flightCommons as flc

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/logging_system')
import logging_commons as log

sys.path.append('/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/raft_consensus/')
import raft_commons as squadron

fault_response_thread_state = False
def fault_response_thread_state_CB(set_state_enable):
    global fault_response_thread_state
    fault_response_thread_state = set_state_enable.data


# Incoming requests from other UAVs in squadron
def squadron_crps_request_CB(crps_request): 
    request_active = crps_request.request_active
    roque_uid = crps_request.serialVersionUID
    tele_roque = crps_request.telemetry

    log.create_publish_log('Received CRPS request from '+roque_uid+'. Request Active: '+str(request_active), 'CRPS-Request '+roque_uid+'-'+boardId)

    if request_active:
        flc.set_crps_state(True)

        publish_squadron_crps_request_ack(roque_uid, True)

        log.create_publish_log('Received CRPS request from ' + roque_uid, 'CRPS Request-Received')
    else:
        flc.set_crps_state(False)

        publish_squadron_crps_request_ack(roque_uid, False)

        log.create_publish_log('Received Annul for CRPS request from ' + roque_uid, 'CRPS Request-Received')


# Ack response for incoming requests from other UAVs in squadron
ack_topic = '/crps/initiate_collaboration_request/acknowledgments/Ack'
def publish_squadron_crps_request_ack(drone_uid, set_state_enable):
    global ack_topic

    crps_ack = CrpsRequest()
    crps_ack.serialVersionUID = boardId
    crps_ack.request_active = set_state_enable
    crps_ack.telemetry = flc.get_telemetry()

    print('Publishing ack to', 'matrice300_'+drone_uid+ack_topic)
    temp_pub = rospy.Publisher('matrice300_'+drone_uid+ack_topic, CrpsRequest, queue_size=1)
    temp_pub.publish(crps_ack)

    log.create_publish_log('Published CRPS Ack to '+drone_uid+'. Request Active: '+str(set_state_enable), 'CRPS-Request '+drone_uid+'-'+boardId)


# Incoming Acks from other UAVs in squadron
crps_request_ack_dict = {}
def collaboration_ack_CB(crps_ack): 
    drone_source = crps_ack.serialVersionUID
    drone_availability = crps_ack.request_active

    log.create_publish_log('Received CRPS Ack from '+drone_source+'. Request Active: '+str(drone_availability), 'CRPS-Request '+boardId+'-'+drone_source)

    if drone_availability:
        crps_request_ack_dict[drone_source] = crps_ack
    else:
        if drone_source in crps_request_ack_dict:
            del crps_request_ack_dict[drone_source]

    print('collaboration_ack_CB - Updated crps_request_ack_dict')
    publish_collaboration_ack_list()    


# Publishes list of UAVs that already Acked a CRPS request
def publish_collaboration_ack_list(): 
    global crps_request_ack_list_Pub, crps_request_ack_dict

    print('publish_collaboration_ack_list - START')    
    crps_request_ack_list = crps_request_ack_dict.values()

    crpsAckList_Packet = CrpsRequestAckList()
    crpsAckList_Packet.crps_request_ack_count = len(crps_request_ack_list)
    crpsAckList_Packet.crps_request_ack_list = crps_request_ack_list

    print('crps_request_ack_list size:', len(crps_request_ack_list))
    print('crps_request_ack_list:', str(crps_request_ack_list))

    crps_request_ack_list_Pub.publish(crpsAckList_Packet)
    print('publish_collaboration_ack_list - DONE')


def await_crps():
    global received_crps_ack, received_crps_ack_mutex
    await_rate = rospy.Rate(0.1)

    #print('Awaiting crps from drones in squadron...')
    #print('Will continue requesting crps_from_squadron')

    received_crps_ack_mutex.acquire()
    crps_request_in_progress = True
    received_crps_ack = 0
    received_crps_ack_mutex.release()

    continue_crps = False

    while not rospy.is_shutdown() and crps_request_in_progress:       
        await_rate.sleep()
        telemetry_source = flc.get_telemetry_source()

        print('telemetry_source:', telemetry_source)

        if 'crps' in telemetry_source.lower():
            #print('Received crps from squadron')
            log.create_publish_log('Received CRPS from squadron', 'CRPS Request')
            continue_crps = True

            received_crps_ack_mutex.acquire()
            crps_request_in_progress = False
            received_crps_ack = received_crps_ack + 1
            received_crps_ack_mutex.release()

        # if not faults:
        #     log.create_publish_log('GPS fault Cleared itself. Will continue normally', 'CRPS Request')
        #     continue_crps = False

        #     received_crps_ack_mutex.acquire()
        #     crps_request_in_progress = False
        #     received_crps_ack = 0
        #     received_crps_ack_mutex.release()

    print('await_crps', 'continue_crps:', continue_crps)

    return continue_crps
        

# def respond_crps_request_CB(tele_roque):
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
# def receive_crps_request_Ack_CB(crps_ack):
#     global received_crps_ack, received_crps_ack_mutex
 
#     ack_drone_name = crps_ack.serialVersionUID
#     ack_drone_longitude = crps_ack.longitude
#     ack_drone_latitude = crps_ack.latitude
    
#     log.create_publish_log('Received CRPS request from drone('+ack_drone_name+') in squadron.', 'CRPS Request-Received')

#     received_crps_ack_mutex.acquire()
#     received_crps_ack = received_crps_ack + 1
#     received_crps_ack_mutex.release()


# publishes crps requests in drones in squadron
def publish_crps_request(drone_list, crps_request_ack_list):
    global request_procedure_state_Pub
    global boardId

    print('Publishing CRPS Request to drones in squadron...', 'drone_list:', drone_list, 'crps_request_ack_list:', crps_request_ack_list, '\n')

    tele = flc.get_telemetry()
    drone_list = squadron.get_squadron_drones_ids()
    drone_published_list = []

    request_topic = '/crps/handle_collaboration_request/Set_State_Enable'

    print('drone_list size:', len(drone_list), str(drone_list))
    if len(drone_list) <= 1:
        print('Will keep trying until more drones are available...')

    for drone in drone_list:
        if drone == boardId:
            print('Drone:', drone, 'Will not request myself...')
        #todo: add request_pending list
        elif drone not in crps_request_ack_list:
            crps_request = CrpsRequest()
            crps_request.serialVersionUID = boardId
            crps_request.request_active = True
            crps_request.telemetry = tele

            drone_published_list.append(drone)

            temp_pub = rospy.Publisher('matrice300_'+drone+request_topic, CrpsRequest, queue_size=1)
            temp_pub.publish(crps_request)

            log.create_publish_log('Publishing CRPS Request to drone '+drone, 'CRPS-Request '+boardId+'-'+drone)
        else:
             print('Already received ack from drone in squadron. Will not publish more requests...')

    
    if len(drone_published_list) > 0:
        print('\npublish_crps_request:\n', str(tele), '\n\n')
        log.create_publish_log('Published CRPS Request to drones ' + str(drone_published_list), 'publish_crps_request')


def publish_crps_annul():
    global crps_request_ack_dict

    request_topic = '/crps/handle_collaboration_request/Set_State_Enable'
    tele = flc.get_telemetry()

    for drone in crps_request_ack_dict:
        crps_request = CrpsRequest()
        crps_request.serialVersionUID = boardId
        crps_request.request_active = True
        crps_request.telemetry = tele

        temp_pub = rospy.Publisher('matrice300_'+drone+request_topic, CrpsRequest, queue_size=1)
        temp_pub.publish(crps_request)

    crps_request_ack_dict = {}


def crps_request_procedure(crps_request_packet):
    global crps_request_ack_dict
    global crps_request_procedure_running
    global fault_response_thread_state
    global fault_response_thread_state_pub

    log.create_publish_log('crps_request_procedure thread running', 'crps_request_procedure')
    
    r = rospy.Rate(0.2) 
    while crps_request_procedure_running and not rospy.is_shutdown():
        squadron_drones = squadron.get_squadron_drones_ids()

        #print('hello')
        #await_crps = await_crps()
        publish_crps_request(squadron_drones, crps_request_ack_dict)
        r.sleep()

        if len(crps_request_ack_dict) > 0 and len(crps_request_ack_dict) == len(squadron_drones) - 1:
            log.create_publish_log('Received Ack from all the drones in squadron', 'crps_request_procedure')
            break


    log.create_publish_log('Awaiting CRPS data from squadron', 'crps_request_procedure')
    c=0
    print_every = 20
    while crps_request_procedure_running and not rospy.is_shutdown():
        if c > print_every:
            print('Awaiting CRPS data...')
            c = 0
        # receive crps from drones
        # complete response/task

        tele = flc.get_telemetry()
        tele_source = flc.get_telemetry_source()
        

        if 'crps' in tele_source:
            if 'crps' in tele.serialVersionUID:
                log.create_publish_log('Received CRPS data from squadron. Starting fault_response_thread', 'crps_request_procedure')
                fault_response_thread_state_pub.publish(True)
                break

        c += 1
        r.sleep()

    log.create_publish_log('Will await until fault_response_thread finishes', 'crps_request_procedure')
    while fault_response_thread_state:
        r.sleep()

    
    # if not faults:
    #     log.create_publish_log('GPS fault Cleared itself. Will continue normally', 'CRPS Request')
    #     continue_crps = False

    #     received_crps_ack_mutex.acquire()
    #     crps_request_in_progress = False
    #     received_crps_ack = 0
    #     received_crps_ack_mutex.release()

    #if crps_request_ack_list:
    #    if len(crps_request_ack_list) > 0:
    #        for drone in crps_request_ack_list:

    log.create_publish_log('crps_request_procedure thread finished', 'crps_request_procedure')

    publish_crps_annul()

    crps_request_procedure_running = False
    request_procedure_state_Pub.publish(crps_request_procedure_running)
    consert_clengaged_Pub.publish(crps_request_procedure_running)



crps_request_packet = None
crps_request_procedure_running = False
crps_request_procedure_thread = None
def start_crps_request_procedure():
    global crps_request_procedure_running, crps_request_procedure_thread

    if not crps_request_procedure_running:
        log.create_publish_log('Starting crps_request_procedure thread', 'crps_request_procedure')

        crps_request_procedure_running = True
        crps_request_procedure_thread = threading.Thread(target=crps_request_procedure, args=(crps_request_packet,))
        crps_request_procedure_thread.start()

        request_procedure_state_Pub.publish(crps_request_procedure_running)
        consert_clengaged_Pub.publish(crps_request_procedure_running)


def stop_crps_request_procedure():
    global crps_request_procedure_running, crps_request_procedure_thread
    
    if crps_request_procedure_running:
        publish_crps_annul()

        crps_request_procedure_running = False
        crps_request_procedure_thread.join()

        request_procedure_state_Pub.publish(crps_request_procedure_running)
        consert_clengaged_Pub.publish(crps_request_procedure_running)

        log.create_publish_log('crps_request_procedure thread stopped', 'crps_request_procedure')


# Internal request for crps
def set_crps_request_state_CB(crps_request):
    global crps_request_packet

    crps_request_packet = crps_request
    crps_request_state_enable = crps_request_packet.request_active

    if crps_request_state_enable:
        start_crps_request_procedure()
    else:
        stop_crps_request_procedure()


def init():
    global request_procedure_state_Pub, consert_clengaged_Pub
    global crps_request_ack_list_Pub
    global fault_response_thread_state_pub
    
    # Initiate Collaboration Request. Internal calls from the same drone
    rospy.Subscriber(dronename+'/crps/initiate_collaboration_request/Set_State_Enable', CrpsRequest, set_crps_request_state_CB)
    request_procedure_state_Pub = rospy.Publisher(dronename+'/crps/initiate_collaboration_request/Get_State_Enable', Bool, queue_size=1, latch=True)
    request_procedure_state_Pub.publish(False)

    consert_clengaged_Pub = rospy.Publisher(dronename+'/kios/clengaged', Bool, queue_size=1, latch=True)
    consert_clengaged_Pub.publish(False)

    # Initiate Collaboration assistance. Calls from roque drone
    rospy.Subscriber(dronename+'/crps/handle_collaboration_request/Set_State_Enable', CrpsRequest, squadron_crps_request_CB)
    aid_procedure_state_Pub = rospy.Publisher(dronename+'/crps/handle_collaboration_request/Get_State_Enable', Bool, queue_size=1, latch=True)
    aid_procedure_state_Pub.publish(False)

    # Acknowledgments from squadron
    rospy.Subscriber(dronename+'/crps/initiate_collaboration_request/acknowledgments/Ack', CrpsRequest, collaboration_ack_CB)
    crps_request_ack_list_Pub = rospy.Publisher(dronename+'/crps/initiate_collaboration_request/acknowledgments/Ack_List', CrpsRequestAckList, queue_size=1, latch=True)
    crps_request_ack_list_Pub.publish(CrpsRequestAckList())

    # Flight State Controller - Fault Response thread control
    rospy.Subscriber(dronename+'/flight_controller/fault_response/thread/Get_State_Enable', Bool, fault_response_thread_state_CB)
    fault_response_thread_state_pub = rospy.Publisher(dronename+'/flight_controller/fault_response/thread/Set_State_Enable', Bool, queue_size=1, latch=True)

    log.init()
    squadron.init()
    flc.init()


def listener(dji_name = "matrice300", as_module=False):
    global dronename, boardId, nodename

    dronename = dji_name
    boardId = dji_name.split('_', 1)[1]
    nodename = dronename.replace('/', '') + '_component_handler'

    if not as_module:
        rospy.init_node(nodename, anonymous=False)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Component Handler...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = '/matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)