import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

#import copy

import os, sys
import math
from geopy import distance

import numpy as np
import sqlite3
import copy

###
point_size = 1.0

###


class MissionDirectory:
    def __init__(self, directory):
        self.directory = directory
        
        self.db_files = find_files(self.directory, ".db")
        self.csv_files = find_files(self.directory, ".csv")
        self.subdirectories_list = find_subdirectories(self.directory)


    def list_db_files(self):
        print(".db files in '" + self.directory + "':")
    
        for db_file in self.db_files:
            print("\t" + db_file)
        
        print()
        
    
    def list_csv_files(self):
        print(".csv files in '" + self.directory + "':")
    
        for csv_file in self.csv_files:
            print("\t" + csv_file)
        
        print()
        
        
    def list_subdirectories(self):
        print("subdirectories in '" + self.directory + "':")
    
        for subdirectories in self.subdirectories_list:
            print("\t" + subdirectories)
        
        print()
        
        
    def __str__(self):
        directory_name = self.directory + "\n"
        db_files_count = "\tdb_files: " + str(len(self.db_files))
        csv_files_count = "\tcsv_files: " + str(len(self.csv_files))
        subdirectories_count = "\tsubdirectories_count: " + str(len(self.subdirectories_list))
        
        final_string = directory_name + db_files_count + csv_files_count + subdirectories_count
        
        return final_string
        

def find_files(directory, extension, debug=False):
    dbFiles = []
    
    if debug:
        print(extension + " files in '" + directory + "':")
    
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(extension):
                if debug:
                    print("\t" + os.path.join(root, file))
                dbFiles.append(os.path.join(root, file))
    
    if debug:
        print()
    
    return dbFiles


def read_csv(file_name):
    global telemetry_c4ea, telemetry_c4ea_crps, telemetry_c4ea_fused
    global telemetry_c501, telemetry_c501_crps, telemetry_c501_fused

    #telemetry_c501_fused = []
    #telemetry_c4ea_fused = []
    print(file_name)

    with open(file_name) as fp:
        for line in fp:
            if 'c501' in line:
                if '_fused' in line:
                       telemetry_c501_fused.append(line)
                elif '-crps' in line:     
                     if 'c501' in file_name:
                         telemetry_c501_crps.append(line)
                else:
                    telemetry_c501.append(line)
            elif 'c4ea' in line:
                if '_fused' in line:
                    telemetry_c4ea_fused.append(line)
                elif '-crps' in line:
                    if 'c4ea' in file_name:
                        telemetry_c4ea_crps.append(line)         
                else:
                    telemetry_c4ea.append(line)


def read_csv_dump(file_name):
    csv_content = []
    
    with open(file_name, 'r') as fp:
        for line in fp:
            csv_content.append(line.split(','))
            
    return csv_content


def read_db(file_name):
    print('read_db file_name:', file_name)
    connection = sqlite3.connect(file_name)

    # Create cursor
    cursor = connection.cursor()
    
    dbDrone_Id = file_name.split('matrice300_')[1].split('-')[0]
    dbDrone_Id_crps = dbDrone_Id + '_crps'
    dbDrone_Id_fused = dbDrone_Id + '_fused'
    
    # Execute query and fetch sorted data
    #query = "SELECT * FROM Telemetry WHERE serialVersionUID = '" + dbDrone_Id + "'"
    #print('query:', query)
    
    query = "SELECT * FROM DroneMovement"
    print('query:', query)
    
    try:
        cursor.execute(query)
        telemetryData = cursor.fetchall()
    except Exception as e:
        print('Failed to gather telemetry data from DroneMovement')
        print(e)
        telemetryData = []
    
    query = "SELECT * FROM Telemetry WHERE serialVersionUID = '" + dbDrone_Id_crps + "'"
    print('query:', query)
    
    try:
        cursor.execute(query)
        crpsData = cursor.fetchall()
    except Exception as e:
        print('Failed to gather crps data from Telemetry')
        print(e)
        crpsData = []
    
    query = "SELECT * FROM Telemetry WHERE serialVersionUID = '" + dbDrone_Id_fused + "'"
    print('query:', query)
    
    try:
        cursor.execute(query)
        fusedData = cursor.fetchall()
    except Exception as e:
        print('Failed to gather fused data from Telemetry')
        print(e)
        fusedData = []
    
    
    #print("Telemetry Data:\n", data)
    
    # Close cursor and connection
    cursor.close()
    connection.close()
    
    print('drone_id:', dbDrone_Id)
    print('telemetryData size:', len(telemetryData))
    print('crpsData size:', len(crpsData))
    print('fusedData size:', len(fusedData))
    print()
    
    return dbDrone_Id, telemetryData, crpsData, fusedData


def read_wind_data(file_name):
    print('read_db file_name:', file_name)
    connection = sqlite3.connect(file_name)

    # Create cursor
    cursor = connection.cursor()

    query = "SELECT wind_speed,wind_angle FROM DroneMovement"
    print('query:', query)
    
    try:
        cursor.execute(query)
        windData = cursor.fetchall()
    except Exception as e:
        print('Failed to gather telemetry data from DroneMovement')
        print(e)
        windData = []

    return windData




def find_subdirectories(directory):
    subdirectories_list = []
    
    for item in os.listdir(directory):
        item_path = os.path.join(directory, item) + "/"
        if os.path.isdir(item_path):
            #print("item_path:", item_path)
            subdirectories_list.append(item_path)
            
    return subdirectories_list


def find_valid_mission_directories(directory):
    search_directory = directory
    #log_directory = "./flightlogs/"  # Current directory
    extension = ".db"  # File extension
    
    mission_directories_list = []
    
    mission_directories = find_subdirectories(search_directory)
    
    for directory in mission_directories:
        #print(directory)
        mission_directories_list.append(MissionDirectory(directory))
        
    mission_directories_list.sort(key=lambda x: x.directory, reverse=False)        
    
    return mission_directories_list


def print_cmd_choices():
    print("===========================")
    print("1. List Mission Directories")
    print("2. List Mission Directory contents")
    print("3. Create plots for Mission Directory")
    print("9. Exit")


def list_mission_directories(mission_directories_list):
    for i in range(len(mission_directories_list)):
        print(str(i) + ") " + str(mission_directories_list[i]))
   
def list_mission_directory_contents(mission_directory):
    mission_directory.list_db_files()
    mission_directory.list_csv_files()
    mission_directory.list_subdirectories()


def create_animation(frame_path):
    width_pixels = 640
    height_pixels = 640
    dpi = 80

    width_inches = width_pixels / dpi
    height_inches = height_pixels / dpi
    

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 2
    output_file = directory + 'animation.mp4'
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width_pixels, height_pixels))

    print('Released video_writer')
    video_writer.release()