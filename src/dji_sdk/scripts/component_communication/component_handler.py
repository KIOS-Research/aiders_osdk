import rospy

from dji_sdk.msg import ComponentList, ComponentState


def cameraListCB(cameraList):
    global payloadIndex, cameraVersion, cameraPosition

    update_flag = False
    cameraCount = len(cameraList.payloadIndex)

    if cameraCount != len(payloadIndex):
        payloadIndex = cameraList.payloadIndex
        cameraVersion = cameraList.cameraVersion
        cameraPosition = cameraList.cameraPosition

        update_flag = True

    if update_flag:   
        update_component_list()


def componentStateCB(componentMsg):
    global componentNames, componentPorts
    
    print('Connection Handler - Got Component:', componentMsg)

    update_flag = False

    if componentMsg.componentActive:
        if componentMsg.componentName not in componentNames:
            componentNames.append(componentMsg.componentName)
            componentPorts.append(componentMsg.componentPort)

            update_flag = True
    else:
        if componentMsg.componentName in componentNames:
            i = componentNames.index(componentMsg.componentName)
            
            del componentNames[i]
            del componentPorts[i]

            update_flag = True

    if update_flag:
        update_component_list()


seq = 0
def update_component_list():
    global seq, boardId
    global payloadIndex, cameraVersion, cameraPosition
    global componentNames, componentPorts

    packet = ComponentList()
    packet.seq = seq
    packet.uid = boardId
    packet.timestamp = int(rospy.get_time())
    
    packet.cameraCount = len(payloadIndex)
    packet.payloadIndex = payloadIndex
    packet.cameraVersion = cameraVersion
    packet.cameraPosition = cameraPosition
    
    packet.componentCount = len(componentNames)
    packet.componentName = componentNames
    packet.componentPort = componentPorts

    componentListPub.publish(packet)


def init():
    global dronename
    global payloadIndex, cameraVersion, cameraPosition
    global componentNames, componentPorts
    global componentListPub
  
    # Info regarding attachable cameras. Coming from C++ backend
    payloadIndex = []
    cameraVersion = []
    cameraPosition = []

    # Info regarding attachable 3rd-party components. Coming from python3 layer
    componentNames = []
    componentPorts = []
 
    rospy.Subscriber(dronename+'/CameraList', ComponentList, cameraListCB)
    rospy.Subscriber(dronename+'/ComponentState', ComponentState, componentStateCB)
    
    componentListPub = rospy.Publisher(dronename+'/ComponentList', ComponentList, queue_size=1, latch=True)

    update_component_list()


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