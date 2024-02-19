import serial
import rospy
from std_msgs.msg import String
import time
from datetime import datetime
import re
import os

drone_name = ""
drone_name_flag = False
loremetry_data = ""
received_data = ""
id_flag = False
balora_id = ""

port_name = '/dev/ttyUSB0'

ser = serial.Serial(
    port= port_name,  # replace with the name of your serial port
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


def GetBoardID():
    f = open("/var/lib/dbus/machine-id", "r")
    boardID = f.read().strip()[0:4]
    #print(boardID)
    return boardID

# =============================================================================
# def get_drone_name():
#     global drone_name
#     global drone_name_flag
# 
#     while True:
#         try:
#             while drone_name_flag == False:
#                 drone_name = ser.readline().decode('utf-8')
#                 print(drone_name)
#                 if drone_name.endswith('\n'):
#                     ser.write("True")
#                     drone_name_flag = True
#                     break
#         except UnicodeDecodeError as decode_error:
#             continue
#         except UnicodeEncodeError as encode_error:
#             continue
# =============================================================================


def loremetry_callback(data):
    global loremetry_data
    global balora_id
    global drone_name

    now = datetime.now()
    loremetry_data = data.data
    tmstmp = now.strftime('%Y/%m/%d %H:%M:%S.%f')[:-3]
    return loremetry_data

    

def main():
    global drone_name
    global drone_name_flag
    global received_data
    global balora_id
    global loremetry_data

    pattern = r'>>.*?<<.*?<<'

    
    now = datetime.now()
    tmstmp = now.strftime('%Y_%m_%d_%H:%M:%S.%f')[:-3]

    directory = "/home/jetson/Documents/aiders_osdk/src/dji_sdk/scripts/balora/"

    os.makedirs(directory, exist_ok=True)

    file_name = f'Data_{tmstmp}.txt'
    
    file_path = os.path.join(directory, file_name)

    res_file = open(file_path, 'w')

    drone_name = "/matrice300_" + GetBoardID()
    loremetry_sub = drone_name + "/loremetry"

    rospy.init_node('balora2', anonymous=True)
    rospy.Subscriber(loremetry_sub, String, loremetry_callback)


    while id_flag == False:
        ser.write("ID".encode())
        received_id = ser.readline().decode('utf-8').strip()
        print(received_id)
        if received_id[:2] == "ID":
            balora_id = received_id.split(",")[-1]
            print(balora_id)
            id_flag == True
        break
    while not rospy.is_shutdown():
        received_data = ser.readline().decode('utf-8').strip()
        now = datetime.now()
        tmstmp = now.strftime('%Y_%m_%d_%H:%M:%S.%f')[:-3]


        if str(received_data) == "ACK":
            sub_data = GetBoardID() + ',' + loremetry_data
	        # print(sub_data)
            ser.write(sub_data.encode() + b'\n' )
        # elif not "#" in received_data:
        #     data = re.sub(pattern, '', received_data)
        #     print(data)
        #     res_file.write(tmstmp + ',' + data + '\n')
        #     telemetry_lora_pub = rospy.Publisher(drone_name + '/TelemetryLora', String, queue_size=10)
        #     telemetry_lora_pub.publish(data)
        #     time.sleep(0.1)
        elif len(received_data) > 0:
            data = re.sub(pattern, '', received_data)
            print(data)
            res_file.write(tmstmp + ',' + data + '\n')
            telemetry_lora_pub = rospy.Publisher(drone_name + '/TelemetryLora', String, queue_size=10)
            telemetry_lora_pub.publish(data)
            time.sleep(0.1)

    rospy.spin()


    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
