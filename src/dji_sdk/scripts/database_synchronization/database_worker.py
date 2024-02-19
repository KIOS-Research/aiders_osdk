
import os, sys

import rospy
import threading

import datetime

import databaseUtils as dbUtils
import databaseLogging as dbLog
import databaseSynchronization as dbSync
from databasePacketImports import *

from std_msgs.msg import UInt32

from packet_queue import PacketQueue

from rospy_message_converter import message_converter

import builtins
import traceback

f = open("/var/lib/dbus/machine-id", "r")
boardId = f.read().strip()[0:4]   
dronename = '/matrice300' + '_' + boardId
f.close()

class DatabaseWorker:
    def __init__(self, topic, message_instance, expected_frequency=0, timestamp_entries=False, publish_stats=True):
        self.topic = topic
        self.message_instance = message_instance
        self.message_type = self.message_instance._type
        self.expected_frequency = expected_frequency
        self.timestamp_entries = timestamp_entries
        self.publish_stats = publish_stats

        if self.expected_frequency == 0:
            self.packet_buffer = PacketQueue(20)
        else:
            self.packet_buffer = PacketQueue(self.expected_frequency)
        self.packet_buffer_mutex = threading.Lock()

        
        self.setup_subscriber()


    def setup_subscriber(self):
        column_names_tuple = self.get_packet_column_names(self.message_instance())

        dbLog.create_table(self.topic, self.message_type, column_names_tuple)
        rospy.Subscriber(self.topic, self.message_instance, self.capture_packet_CB)

        if self.publish_stats:
            self.stat_pub = rospy.Publisher(dronename+'/Database/increment_stored_packets', UInt32, queue_size=5)
        
    def capture_packet_CB(self, packet):
        serialized_packet = self.serialize_packet(packet)
        self.store_data_buffer(serialized_packet)


    def get_packet_column_names(self, packet):  
        packet_dictionary = message_converter.convert_ros_message_to_dictionary(packet)


        # Place timestamp column first
        if self.timestamp_entries:
            time_dict = {'timestamp':''}
            packet_dictionary = {**time_dict, **packet_dictionary}

        column_names = tuple(packet_dictionary.keys())
        return column_names


    def serialize_packet(self, packet):
        packet_dictionary = message_converter.convert_ros_message_to_dictionary(packet)
        serialized_packet = []

        if self.timestamp_entries:
            serialized_packet.append(str(datetime.datetime.now().strftime("%Y%m%d-%H:%M:%S.%s")))

        for key, value in packet_dictionary.items():
            if isinstance(value, (dict, list)):
                # If the value is a dictionary or list, convert it to a string
                serialized_packet.append(str(value))
            else:
                # If it's not a dictionary or list, keep it unchanged
                serialized_packet.append(value)


        #print(tuple(serialized_packet))

        return tuple(serialized_packet)


    def store_data_buffer(self, serialized_packet):
        with self.packet_buffer_mutex:
            self.packet_buffer.put(serialized_packet)

        #self.print('self.packet_buffer.size():', self.packet_buffer.size())

        if self.packet_buffer.size() >= self.packet_buffer.max_size() / 2:
            #self.print('Please empty buffer to sql database')
            self.store_data_db()

    
    def store_data_db(self):
        self._store_data_db_thread = threading.Thread(target=self._store_data_db)
        self._store_data_db_thread.start()
    
    def _store_data_db(self):
        #print('_store_data_db_thread - Started')
        with self.packet_buffer_mutex:
            data_list = self.packet_buffer.get_packet_list()
            self.packet_buffer.dump_queue()

        if self.publish_stats:
            packet_count = len(data_list)
            self.stat_pub.publish(packet_count)
        
        dbUtils.executeQueryWriteMany(self.topic, data_list)

        #print('\n\n')
        #for data in data_list:
        #    print(data)
        
        #print('_store_data_db_thread - Done')


    def print(self, *objs, **kwargs):
        my_prefix = self.topic + ':'
        builtins.print(my_prefix, *objs, **kwargs)
