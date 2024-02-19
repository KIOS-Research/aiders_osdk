import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

#import copy

import os, sys
import math
from geopy import distance

import numpy as np
import sqlite3
import copy
import time

import file_utils
import plot_utils

###
point_size = 1.0
slice_config_name = 'slice_config.txt'

###
def create_slice_config(drone_list, directory):
    with open(directory + slice_config_name, 'w') as f:      
        for drone in drone_list:
            f.write(drone.name + '\n')
            f.write(str(drone.telemetry_slice_start) + ' ' + str(drone.telemetry_slice_end) + '\n')
            f.write(str(drone.crps_slice_start)  + ' ' +  str(drone.crps_slice_end) + '\n')
            f.write(str(drone.fused_slice_start)  + ' ' + str(drone.fused_slice_end) + '\n')

    print('slice_config.txt created')


def load_slice_config(drone_list, slice_config_path):
    try:
        with open(slice_config_path, 'r') as f:
            data_drone_name = f.readline().strip()
            while data_drone_name:
                data_drone_tele_slice = f.readline().strip()
                data_drone_crps_slice = f.readline().strip()
                data_drone_fused_slice = f.readline().strip()
                
                print('====')
                print(data_drone_name)
                print(data_drone_tele_slice)
                print(data_drone_crps_slice)
                print(data_drone_fused_slice)
                print('====')


                #print(data_drone_tele_slice)
                data_drone_tele_list = data_drone_tele_slice.split(' ')
                data_drone_tele_slice_start = int(data_drone_tele_list[0])
                data_drone_tele_slice_end = int(data_drone_tele_list[1])

                #print(data_drone_crps_slice)
                data_drone_crps_list = data_drone_crps_slice.split(' ')
                data_drone_crps_slice_start = int(data_drone_crps_list[0])
                data_drone_crps_slice_end = int(data_drone_crps_list[1])

                #print(data_drone_fused_slice)
                data_drone_fused_list = data_drone_fused_slice.split(' ')
                data_drone_fused_slice_start = int(data_drone_fused_list[0])
                data_drone_fused_slice_end = int(data_drone_fused_list[1])

                drone_found = False
                for drone in drone_list:
                    if data_drone_name == drone.name:
                        drone.telemetry_slice_start = data_drone_tele_slice_start
                        drone.telemetry_slice_end = data_drone_tele_slice_end
                
                        drone.crps_slice_start = data_drone_crps_slice_start
                        drone.crps_slice_end = data_drone_crps_slice_end

                        drone.fused_slice_start = data_drone_fused_slice_start
                        drone.fused_slice_end = data_drone_fused_slice_end
                        
                        drone_found = True
                        break

                if not drone_found:
                    print("Couldn't match slice_config setup to drone")
                    print("data_drone_name:", data_drone_name)
                    #print("Couldn't match slice_config setup to drone")

                data_drone_name = f.readline().strip()

        return True
    except Exception as e:
        print('An exception came up during load_slice_config()')
        print('Exception:', e)
        return False


def read_slice_config_prompt(drone_list, directory):
    if os.path.exists(directory + slice_config_name):
        print('\n' + directory + slice_config_name, "exists!")

        print('Do you wanna use this slice config file?')
        #print('This will save you time next time you try to recreate the plots.')
        print('1. Yes\n2. No')

        load_slice_config_success = False

        cmd = -1 
        while cmd != 1 and cmd != 2:
            cmd = -1
            while cmd == -1:
                try:
                    # Prompt the user for an integer
                    user_input = input("> ")
                    
                    # Try to convert the input to an integer
                    integer_input = int(user_input)
                    cmd = integer_input        
                except ValueError:
                    # If a ValueError occurs, the input was not a valid integer
                    cmd = -1
                    print("Invalid input. Please enter an integer.")
            
            if cmd == 1:
                load_slice_config_success = load_slice_config(drone_list, directory + slice_config_name)

        return load_slice_config_success
    else:
        return False



def create_slice_config_prompt(drone_list, directory):
    print('Do you wanna create a slice config file?')
    print('This will save you time next time you try to recreate the plots.')
    print('1. Yes\n2. No')

    cmd = -1 
    while cmd != 1 and cmd != 2:
        cmd = -1

        
        while cmd == -1:
            try:
                # Prompt the user for an integer
                user_input = input("> ")
                
                # Try to convert the input to an integer
                integer_input = int(user_input)
                cmd = integer_input        
            except ValueError:
                # If a ValueError occurs, the input was not a valid integer
                cmd = -1
                print("Invalid input. Please enter an integer.")
        
        if cmd == 1:
            create_slice_config(drone_list, directory)


def print_cmd_choices():
    print("===========================")
    print("1. List Mission Directories")
    print("2. List Mission Directory contents")
    print("3. Create plots for Mission Directory")
    print("4. Change Directory")
    print("8. Recreate previous")
    print("9. Exit")


cur_dir_file = 'current_directory.cfg'
def create_current_directory_config(directory):
    with open(cur_dir_file, 'w') as file:
        file.write(directory)


def read_current_directory_config():
    with open(cur_dir_file, 'r') as file:
        data = file.read()
        cur_dir = data.strip()
        return cur_dir
    
    if not data:
        print("Reached the end of the file.")
    else:
        print("Did not reach the end of the file.")
    



def create_mission_plots(mission_directory, use_slice_config=False):
    directory = mission_directory.directory         # Current directory
    create_current_directory_config(directory)
    
    extension = ".db"                               # File extension
    db_list = file_utils.find_files(directory, extension)  
    
    extension = ".csv"
    csv_list = file_utils.find_files(directory, extension)

    print("directory:", directory)
    print(".db files:", )
    for db in db_list:
        print('\t', db)

    print("\n.csv files:", )
    for csv in csv_list:
        print('\t', csv)

    time.sleep(1)
     
    drone_list = []
    for db in db_list:
        dbDrone_Id, tele, crps, fused = file_utils.read_db(db)
        dbDrone_Id = dbDrone_Id.strip()
        
        matching_monitoring_internal_file = None
        for csv_file in csv_list:
            #print("csv_file:", csv_file)
            if dbDrone_Id in csv_file and "monitoring_internal" in csv_file:
                matching_monitoring_internal_file = csv_file
        
        print('dbDrone_Id:', dbDrone_Id)
        print('matching_monitoring_internal_file:', matching_monitoring_internal_file)
        
        monitoring_internal_data = []
        if matching_monitoring_internal_file:
            monitoring_internal_data = file_utils.read_csv_dump(matching_monitoring_internal_file)
        
        drone = plot_utils.DroneTelemetry(dbDrone_Id, tele, crps, fused, monitoring_internal_data, db)
        drone_list.append(drone)
    
    #
    # check if slice txt exists

    imported_slice_config = False
    if use_slice_config:
        imported_slice_config = load_slice_config(drone_list, directory + slice_config_name)
    else:
        imported_slice_config = read_slice_config_prompt(drone_list, directory)
    
    print('calling create_mission_analysis_plot\n', '\tuse_slice_config:', use_slice_config, '\timported_slice_config:', imported_slice_config)
    plot_utils.create_mission_analysis_plot(drone_list, directory, slice_config_exists=imported_slice_config)
    
    if not imported_slice_config:
        # create slice txt
        create_slice_config_prompt(drone_list, directory)
    


    plot_utils.create_estimation_plot(drone_list, directory)
    #plot_utils.create_error_plot(drone_list, directory)
    #plot_utils.create_animate_plot(drone_list, directory)
    #create_error_dist_plot(drone_list)
    #create_plot(drone_list)
    print("Done.")
    #try:
    #    plt.show()
    #except:
    #    pass


def handle_cmd_input(cmd, mission_directories_list):
    if cmd == 1:
        file_utils.list_mission_directories(mission_directories_list)
    if cmd == 2:
        directory_input = -1
        while directory_input == -1:
            try:
                # Prompt the user for an integer
                user_input = input("> Enter Directory Number:")
                
                # Try to convert the input to an integer
                directory_input = int(user_input)     
            except ValueError:
                directory_input = -1
                print("Invalid input. Please enter an integer.")

        file_utils.list_mission_directory_contents(mission_directories_list[directory_input])
    if cmd == 3:
        directory_input = -1
        while directory_input == -1:
            try:
                # Prompt the user for an integer
                user_input = input("> Enter Directory Number:")
                
                # Try to convert the input to an integer
                directory_input = int(user_input)                
            except ValueError:
                directory_input = -1
                print("Invalid input. Please enter an integer.")
            
            
            create_mission_plots(mission_directories_list[directory_input])
    if cmd == 4:
        directory_input = -1
        while directory_input == -1:
            try:
                # Prompt the user for an integer
                user_input = input("> Enter Directory Path or Directory index:")
                
                if user_input.isnumeric():
                    directory_input = int(user_input)
                else:
                    directory_input = str(user_input)
            except ValueError:
                directory_input = -1
                print("Invalid input. Please enter a valid path.")
            
            if type(directory_input) == str:
                print('\n', directory_input)
                start_interactive_prompt(directory_input)
            else:
                print('\n', mission_directories_list[directory_input].directory)
                start_interactive_prompt(mission_directories_list[directory_input].directory)
    if cmd == 8:
        latest_directory = read_current_directory_config()
        
        if os.path.isdir(latest_directory):
            print("Directory exists!")
            mission_directory = file_utils.MissionDirectory(latest_directory)
            create_mission_plots(mission_directory, use_slice_config=True)

        else:
            print("Directory does not exist.")

            
        

def interactive_prompt(directory):   
    mission_directories_list = file_utils.find_valid_mission_directories(directory)

    print('\nCurrent Directory:', directory, '\n')

    cmd = -1 
    while cmd != 9:
        cmd = -1
        print_cmd_choices()
        
        while cmd == -1:
            try:
                # Prompt the user for an integer
                user_input = input("> ")
                
                # Try to convert the input to an integer
                integer_input = int(user_input)
                cmd = integer_input        
            except ValueError:
                # If a ValueError occurs, the input was not a valid integer
                cmd = -1
                print("Invalid input. Please enter an integer.")
        
        handle_cmd_input(cmd, mission_directories_list)

    if cmd == 9:
        exit()


def start_interactive_prompt(directory):
    interactive_prompt(directory)


if __name__ == "__main__":  
    start_interactive_prompt('../flightlogs/')

    exit()
