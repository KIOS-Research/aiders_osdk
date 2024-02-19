import rospy



class Component:
    def __init__(self, name, port=None, baudrate=None):
        self.name = name
        self.port = port
        self.baudrate = baudrate


        self.handshake_success = False
        self.connection_success = False

    
    def 


def handshake():
    pass

def init():
    pass

def listener(dji_name = "/matrice300", as_module=False):
    global dronename, nodename
    global logicRate
    
    dronename = dji_name  
    nodename = dronename.replace('/', '') + '_flightLogic'
    print(nodename)
    
    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()

        # while not rospy.is_shutdown():
        #     publishFlightLogicState()
        #     statePubRate.sleep()     


try:
    print('Initializing Flight Logic')
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

