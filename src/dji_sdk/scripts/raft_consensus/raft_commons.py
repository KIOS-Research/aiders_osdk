import rospy
from std_msgs.msg import String

current_client_pool = []
def client_pool_CB(client_pool_id_list):
    global current_client_pool
    
    client_pool_string = client_pool_id_list.data
    client_pool_split = client_pool_string.split("'")
    client_pool_list = []
    for i in range(1, len(client_pool_split), 2):
        #print('i:',i,':', client_pool_split[i])
        client_pool_list.append(client_pool_split[i])

    current_client_pool = client_pool_list
    #print('current_client_pool:', current_client_pool)


def get_squadron_drones_ids():
    global current_client_pool

    return current_client_pool


def init():
    global log_publisher
        
    rospy.Subscriber('/raft/client_pool', String, client_pool_CB)


def listener(dji_name = "/matrice300", as_module = False):
    global dronename, nodename

    dronename = dji_name
    nodename = dronename.replace('/', '') + '_raft_commons'
    #print(nodename)

    if not as_module:
        rospy.init_node(nodename)
        init()
    else:
        print(nodename,'- Imported as a module. Call init when ready to initialize rostopics...')

    if not as_module:
        rospy.spin()


print('Initializing Raft Commons...')
f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]

dronename = 'matrice300' + '_' + boardId

if __name__ == '__main__':
    as_module = False
else:
    as_module = True

listener(dji_name=dronename, as_module=as_module)