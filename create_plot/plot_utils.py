import time
import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

#import copy

import os, sys
import math
from geopy import distance

import numpy as np
import sqlite3
import copy

import cv2

import file_utils as fu

###
point_size = 1.0

###

class DroneTelemetry:
    def __init__(self, name, telemetry, crps, fused, monitoring_internal, db_file):
        self.name = name
        self.telemetry = telemetry
        self.crps = crps
        self.fused = fused
        self.db_file = db_file

        self.telemetry_slice_start = 0
        self.telemetry_slice_end = 0

        self.crps_slice_start = 0
        self.crps_slice_end = 0

        self.fused_slice_start = 0
        self.fused_slice_end = 0
               
        if len(self.crps) * len(self.fused) == 0:
            self.roque = True
        else:
            self.roque = False
        
        #f"{str(datetime.datetime.now())},{bearing},{bearingY},{self.distXY[-1][0]},{self.distXY[-1][1]},{self.folrpsgps[-1][0]},{self.folrpsgps[-1][1]},{self.rpsgpsXY[-1][0]},{self.rpsgpsXY[-1][1]},{self.zoom}
        self.monitoring_internal = monitoring_internal
        
    def set_roque(self, is_roque):
        self.roque  = is_roque
                
    def is_roque(self):
        return self.roque
       
    def get_monitoring_internal_data(self, monitoring_internal_entry):
        timestamp = monitoring_internal_entry[0]
        
        bearing = float(monitoring_internal_entry[1])
        bearingY = float(monitoring_internal_entry[2])
        
        distXY0 = float(monitoring_internal_entry[3])
        distXY1 = float(monitoring_internal_entry[4])
        
        folrpsgps0 = float(monitoring_internal_entry[5])
        folrpsgps1 = float(monitoring_internal_entry[6])
        
        rpsgpsXY0 = float(monitoring_internal_entry[7])
        rpsgpsXY1 = float(monitoring_internal_entry[8])
        
        zoom = float(monitoring_internal_entry[9])
        
        return timestamp, bearing, bearingY, distXY0, distXY1, folrpsgps0, folrpsgps1, rpsgpsXY0, rpsgpsXY1, zoom
       
    def calculate_monitoring_internal_original(self):        
        self.lat_mi_og = []
        self.lon_mi_og = []
        
        print('Calculate monitoring internal original')
        
        #print(self.monitoring_internal)
          
        if len(self.monitoring_internal) > 0:   
            for mi_entry in self.monitoring_internal:
                if len(mi_entry) == 10:
                    timestamp, bearing, bearingY, distXY0, distXY1, folrpsgps0, folrpsgps1, rpsgpsXY0, rpsgpsXY1, zoom = self.get_monitoring_internal_data(mi_entry)
                
                    #bearing = self.folrpsgps[-1][2]# +360 bearing from telemetry ( test +360 if not working properly) 
                    bearingY_calc = -(math.atan(distXY0 / distXY1)*(180 / math.pi)) + bearing
                    
                    # calculate distance using pythagorean theorem
                    a = math.sqrt(distXY0**2 + distXY1**2) / 1000.0
                    # print(a,self.distXY[-1][0], self.distXY[-1][1])
                    # convert that distance to an distance object
                    d = distance.distance(kilometers=a)
                    #print('self.folrpsgps[-1]=', self.folrpsgps[-1])
                    
                    # convert that distance object into a geopoint 
                    uvar = d.destination(point=(folrpsgps0, folrpsgps1), bearing=bearingY_calc)
                    #self.rpsgpsXY.append((uvar.latitude,uvar.longitude,bearingY))
                    
                    self.lat_mi_og.append(uvar.latitude)
                    self.lon_mi_og.append(uvar.longitude)
                    
                    if False:
                        print("bearingY:", bearingY_calc, bearingY, bearingY_calc / bearingY)
                        print("a:", a)
                        print("d:", d)
                        print("uvar:", uvar)
                
                
        return self.lat_mi_og, self.lon_mi_og
    
    def calculate_monitoring_internal_augmented(self, error_factor = 1.25):
        self.lat_mi_au = []
        self.lon_mi_au = []
        
        print('Calculate monitoring internal augmented')
        
        #error_factor = 1.25
        error_factor_power = 1.0
        
        if len(self.monitoring_internal) > 0:   
            for mi_entry in self.monitoring_internal:
                if len(mi_entry) == 10:
                    timestamp, bearing, bearingY, distXY0, distXY1, folrpsgps0, folrpsgps1, rpsgpsXY0, rpsgpsXY1, zoom = self.get_monitoring_internal_data(mi_entry)
                
                    #bearing = self.folrpsgps[-1][2]# +360 bearing from telemetry ( test +360 if not working properly) 
                    bearingY_calc = -(math.atan(distXY0 / distXY1)*(180 / math.pi)) + bearing
                    
                    # calculate distance using pythagorean theorem
                    a = (math.sqrt(distXY0**2 + distXY1**2) / 1000.0) * error_factor #* error_factor_power**2
                    # print(a,self.distXY[-1][0], self.distXY[-1][1])
                    # convert that distance to an distance object
                    d = distance.distance(kilometers=a)
                    #print('self.folrpsgps[-1]=', self.folrpsgps[-1])
                    
                    # convert that distance object into a geopoint 
                    uvar = d.destination(point=(folrpsgps0, folrpsgps1), bearing=bearingY_calc)
                    #self.rpsgpsXY.append((uvar.latitude,uvar.longitude,bearingY))
                    
                    self.lat_mi_au.append(uvar.latitude)
                    self.lon_mi_au.append(uvar.longitude)
                    
                    if False:
                        print("bearingY:", bearingY_calc, bearingY, bearingY_calc / bearingY)
                        print("a:", a)
                        print("d:", d)
                        print("uvar:", uvar)
        
        return self.lat_mi_au, self.lon_mi_au
            

def geo_diff_valid(lat, lon, lat_prev, lon_prev):
    error_lim = 0.00001
    lat_diff = lat - lat_prev
    lon_diff = lon - lon_prev

    #print('lat_diff:', f'{lat_diff:.10f}')
    #print('lon_diff:', f'{lon_diff:.10f}')

    if abs(lat_diff) > error_lim or abs(lon_diff) > error_lim:
         return False

    return True


def retrieve_geo(point_list, datatype_name=None, enforce_alt_limit=True):
    lat = []
    lon = []
    
    lat_v = 0.0
    lon_v = 0.0
    alt_v = 0.0
    
    alt_v_lim = 10.0
    
    row_size = 0   
    if len(point_list) > 0:
        row_size = len(point_list[0])
        
    print('\nrow_size:', row_size)
    
    for point in point_list:       
        if row_size <= 17:
            #Telemetry
            lat_v = (float(point[11]))
            lon_v = (float(point[10]))
            alt_v = alt_v_lim + 1
        else:
            #DroneMovement
            lat_v = (float(point[9]))
            lon_v = (float(point[8]))
            alt_v = (float(point[10]))
            
        if lat_v * lon_v != 0.0 and (alt_v > alt_v_lim or not enforce_alt_limit):           
            lat.append(lat_v)
            lon.append(lon_v)
    if datatype_name:
        print('datatype_name:', datatype_name)
    
    print('lat size:', len(lat))
    print('lon size:', len(lon))
    
    return lat, lon


def retrieve_timestamp(point):
    timestamp = 0.0
    
    row_size = len(point)        
    #print('\nrow_size:', row_size)
    
    if row_size <= 17:
        #Telemetry
        timestamp = (float(point[0]))
    else:
        #DroneMovement
        timestamp = (float(point[2]))

    return timestamp


def smooth_path(lat_list, lon_list):
    win_size = 10
    
    lat_mov_avg = []
    lon_mov_avg = []
    
    list_size = min(len(lat_list), len(lon_list))
    
    for i in range(list_size - win_size + 1):
        window_lat = lat_list[i:i+win_size]
        window_lon = lon_list[i:i+win_size]
        
        average = sum(window_lat) / win_size
        lat_mov_avg.append(average)
        
        average = sum(window_lon) / win_size
        lon_mov_avg.append(average)
        
    return lat_mov_avg, lon_mov_avg


def calculate_error(lat_est, lon_est, lat_gt, lon_gt):
    if len(lat_est) * len(lon_est) and len(lat_gt) * len(lon_gt):
        point_count = min(lat_est, lat_gt)
        dist_sum = 0.0
        
        
    # =============================================================================
    #     error_x = [abs(lat_gt[i] - lat_est[i]) for i in range(len(point_count))]
    #     error_y = [abs(lon_gt[i] - lon_est[i]) for i in range(len(point_count))]
    #     
    #     #print('error_x:', error_x)
    #     #print('error_y:', error_y)
    #     
    #     average_error_x = sum(error_x) / len(error_x)
    #     average_error_y = sum(error_y) / len(error_y)
    #     
    # 
    #     #std_deviation = (sum((e - mean_error) ** 2 for e in error) / len(error)) ** 0.5
    #     degrees_to_meters = 111000
    # 
    #     # Convert the error to meters
    #     error_meters_x = degrees_to_meters * average_error_x
    #     error_meters_y = degrees_to_meters * average_error_y
    # =============================================================================
        
        for i in range(len(lat_est)):
            dist_km = decimal_degrees_to_km(lat_est[i], lon_est[i], lat_gt[i], lon_gt[i])
        
            dist_sum = dist_km + dist_sum
            
        average_error_x = dist_sum / len(lat_est)
        average_error_y = dist_sum / len(lat_est)
        
        error_meters_x = average_error_x * 1000
        error_meters_y = average_error_y * 1000
            
        print('average_error_x:', average_error_x, "[km]")
        print('average_error_y:', average_error_y, "[km]")
        print()
        print('error_meters_x:', error_meters_x, "[m]")
        print('error_meters_y:', error_meters_y, "[m]")
    
        return average_error_x, average_error_y, error_meters_x, error_meters_y


def decimal_degrees_to_km(lat1, lon1, lat2, lon2):
    # Earth radius in kilometers
    earth_radius = 6371

    # Convert decimal degrees to radians
    lat1_rad = math.radians(lat1)
    lon1_rad = math.radians(lon1)
    lat2_rad = math.radians(lat2)
    lon2_rad = math.radians(lon2)

    # Haversine formula
    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad
    a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    distance = earth_radius * c

    return distance
  

def average_paths(lat_list, lon_list):
    path_count = min(len(lat_list), len(lon_list))
    lat_points = sys.maxsize
    lon_points = sys.maxsize
    
    print('\npath_count:', path_count)
    print('lat_list_len:', len(lat_list))
    print('lon_list_len:', len(lon_list))    
    
    for i in range(path_count):
        lat_points = min(lat_points, len(lat_list[i]))
        lon_points = min(lon_points, len(lon_list[i]))
       
    point_count = min(lat_points, lon_points)    
       
    print('point_count:', point_count)
    print('lat_points:', lat_points)
    print('lon_points:', lon_points)
    
    lat_average_path = []
    lon_average_path = [] 
    for j in range(point_count):
        lat = 0.0
        lon = 0.0
        
        for i in range(path_count):
            lat = lat + lat_list[i][j]
            lon = lon + lon_list[i][j]
            
        lat_avg_point = lat / path_count
        lon_avg_point = lon / path_count
        
        lat_average_path.append(lat_avg_point)
        lon_average_path.append(lon_avg_point)

    return lat_average_path, lon_average_path


def normalize_latlon_population(lat1, lon1, lat2, lon2):
    print('normalize_latlon_population - Original List Size:')
    print('lat size:', len(lat1), len(lat2))
    print('lon size:', len(lon1), len(lon2))
    
    size1 = len(lat1)
    size2 = len(lat2)
    
    if size1 != size2:
        if size1 > size2:
            larger_list_index = 1
            step = 1 - size2 / size1
        else:
            larger_list_index = 2
            step = 1 - size1 / size2
            
        if larger_list_index == 1:
            lat1 = normalize_list(lat1, step)
            lon1 = normalize_list(lon1, step)
        else:
            lat2 = normalize_list(lat2, step)
            lon2 = normalize_list(lon2, step)
        
    print('Final List Size:')
    print('lat size:', len(lat1), len(lat2))
    print('lon size:', len(lon1), len(lon2))


def normalize_lists_population(list1, list2):
    l1 = copy.deepcopy(list1)
    l2 = copy.deepcopy(list2)
    
    print('normalize_lists_population - Original List Size:')
    print('list1 size:', len(l1))
    print('list2 size:', len(l2))
    
    size1 = len(l1)
    size2 = len(l2)
    
    if size1 != size2:
        if size1 > size2:
            larger_list_index = 1
            step = 1 - size2 / size1
        else:
            larger_list_index = 2
            step = 1 - size1 / size2
            
        if larger_list_index == 1:
            l1 = normalize_list(l1, step)
        else:
            l2 = normalize_list(l2, step)
        
    print('Final List Size:')
    print('list1 size:', len(l1))
    print('list2 size:', len(l2))
    
    return l1, l2


def normalize_list(list1, step):
    c = 0.0
    deleted_indexes = []
    print ('step:', step)
    
    for i in range(len(list1)):
        if c >= 1:
            deleted_indexes.append(i)
            c = c - 1
            
        c = c + step
        
    if c > 0:
        if i not in deleted_indexes:
            deleted_indexes.append(i)
            deleted_indexes.reverse()
        else:
            deleted_indexes.reverse()
            deleted_indexes.append(0) 
    else:       
        deleted_indexes.reverse()

    for index in deleted_indexes:
        del list1[index]

    return list1
   

def slice_path(lat, lon, slice_start, slice_end):
    if slice_start == 0 and slice_end == 0:
        print('slice_path - slice_start, slice_end are both set to 0. Will not slice path')
        return lat, lon

    print('Slice Path - Original List Size:')
    print('lat size:', len(lat))
    print('lon size:', len(lon))
    
    if slice_end == 0:
        lat = lat[slice_start:]
        lon = lon[slice_start:]
    else:
        lat = lat[slice_start:-slice_end]
        lon = lon[slice_start:-slice_end]
    
    print('Slice Path - Final List Size:')
    print('lat size:', len(lat))
    print('lon size:', len(lon))
    
    return lat, lon


def slice_list_percentage(initial_list, size_percentage):
    path_size = len(initial_list)
    path_slice_end = int(path_size * (1.0 - size_percentage))

    print('path_slice_end:', path_slice_end)
    
    final_list, _ = slice_path(initial_list, initial_list, 0, path_slice_end)

    return final_list


def slice_path_percentage(lat, lon, size_percentage):
    path_size = len(lat)
    path_slice_end = int(path_size * (1.0 - size_percentage))

    print('path_slice_end:', path_slice_end)
    
    lat, lon = slice_path(lat, lon, 0, path_slice_end)

    return lat, lon


def create_mission_analysis_plot(drone_list, directory, slice_config_exists = False):
    list_size = len(drone_list)
    
    fig, ax = plt.subplots(list_size, 5)
    fig.set_facecolor("w")
    
    plt.title("Collaborative Localization - Drone path analysis")

    #plt.legend(loc="lower right")
    point_size = 1.0
    
    roque_lat = []
    roque_lon = []

    lat_fused_list = []
    lon_fused_list = []
    
    lat_aug_list, lon_aug_list = [], []
    
    r = 0
    for drone in drone_list:
        print('drone.telemetry size:', len(drone.telemetry))
        lat, lon = retrieve_geo(drone.telemetry, drone.name)
        lat_crps, lon_crps = retrieve_geo(drone.crps, drone.name + '_crps')
        lat_fused, lon_fused = retrieve_geo(drone.fused, drone.name + '_fused')

        if not slice_config_exists:
            print('Slicing Drone Telemetry')
            lat, lon, telemetry_slice_start, telemetry_slice_end = interactive_slicing(lat, lon)
            drone.telemetry_slice_start = telemetry_slice_start
            drone.telemetry_slice_end = telemetry_slice_end
        else:
            lat, lon = slice_path(lat, lon, drone.telemetry_slice_start, drone.telemetry_slice_end)

        lat_mov_avg, lon_mov_avg = [], []
        lat_mi_og, lon_mi_og = [], []
        lat_mi_au, lon_mi_au = [], []
             
        is_roque_drone = False
        if (len(lat_crps) * len(lon_crps)) * (len(lat_fused) * len(lon_fused)) == 0:
            is_roque_drone = True
            roque_lat = lat
            roque_lon = lon
            drone.set_roque(True)
        else:
            if not slice_config_exists:
                lat_crps, lon_crps, crps_slice_start, crps_slice_end  = interactive_slicing(lat_crps, lon_crps, scatterplot=True)
                drone.crps_slice_start = crps_slice_start
                drone.crps_slice_end = crps_slice_end
            else:
                lat_crps, lon_crps = slice_path(lat_crps, lon_crps, drone.crps_slice_start, drone.crps_slice_end)

            if not slice_config_exists:
                lat_fused, lon_fused, fused_slice_start, fused_slice_end  = interactive_slicing(lat_fused, lon_fused, scatterplot=True)
                drone.fused_slice_start = fused_slice_start
                drone.fused_slice_end = fused_slice_end
            else:
                lat_fused, lon_fused = slice_path(lat_fused, lon_fused, drone.fused_slice_start, drone.fused_slice_end)

            normalize_latlon_population(lat_crps, lon_crps, lat_fused, lon_fused)
            
            lat_mi_au, lon_mi_au = drone.calculate_monitoring_internal_augmented()
        

            
            lat_mov_avg, lon_mov_avg = smooth_path(lat_fused, lon_fused)
            
            lat_fused_list.append(lat_mov_avg)
            lon_fused_list.append(lon_mov_avg)
            
            lat_aug_list.append(lat_mi_au)
            lon_aug_list.append(lon_mi_au)
            
        
        if list_size > 1:
            ax[r, 0].plot(lat, lon, label=drone.name)
            ax[r, 0].set_title(drone.name)
            if r == list_size - 1:
                ax[r, 0].set_xlabel("Longitude", labelpad=8)
            ax[r, 0].set_ylabel("Latitude", labelpad=8)
            #ax[r, 0].legend(loc="lower right")
            
            #ax[r, 1].plot(lat_crps, lon_crps, label=drone.name+"_crps")
            ax[r, 1].scatter(x=lat_crps, y=lon_crps, s=point_size, label=drone.name+"_crps")
            ax[r, 1].set_title(drone.name+"_crps")
            #ax[r, 1].legend(loc="lower right")
            
            #ax[r, 2].plot(lat_fused, lon_fused, label=drone.name+"_fused")
            ax[r, 2].scatter(x=lat_fused, y=lon_fused, s=point_size, label=drone.name+"_fused")
            ax[r, 2].set_title(drone.name+"_fused")
            #ax[r, 2].legend(loc="lower right")
            
            #ax[r, 3].plot(lat_mov_avg, lon_mov_avg, label=drone.name+"_fused_smoothed")
            ax[r, 3].scatter(x=lat_mov_avg, y=lon_mov_avg, s=point_size, label=drone.name+"_fused_smoothed")
            ax[r, 3].set_title(drone.name+"_fused_smoothed")
            #ax[r, 3].legend(loc="lower right")
            
            
            #ax[r, 4].plot(lat_mi_og, lon_mi_og, label=drone.name+"_mon_int_original")
            #ax[r, 4].scatter(x=lat_mi_og, y=lon_mi_og, s=point_size, label=drone.name+"_mon_int_original")
            #ax[r, 4].set_title(drone.name+"_mon_int_original")
            
            #ax[r, 5].plot(lat_mi_au, lon_mi_au, label=drone.name+"_mon_int_augmented")
            ax[r, 4].scatter(x=lat_mi_au, y=lon_mi_au, s=point_size, label=drone.name+"_mon_int_augmented")
            ax[r, 4].set_title(drone.name+"_mon_int_augmented")
        else:
            ax[0].plot(lat, lon, label=drone.name)      
            ax[1].plot(lat_crps, lon_crps, label=drone.name+"_crps")      
            ax[2].plot(lat_fused, lon_fused, label=drone.name+"_fused")
            ax[3].plot(lat_mov_avg, lon_mov_avg, label=drone.name+"_fused_smoothed")
            
            ax[0].set_xlabel("Longitude", labelpad=8)
            ax[0].set_ylabel("Latitude", labelpad=8)
            
        r = r + 1

    fig.savefig(directory + 'plot.pdf', format='pdf')
    #fig_overlap.savefig('plot_overlap.pdf', format='pdf')
    plt.show(block=False)
    print('Done.')   


def annotate_plot(lat, lon, ax, print_every = 100):
    #print_every = 100
    for i in range(len(lat)):
        if i % print_every == 0:
            ax.annotate(str(i), (lat[i], lon[i]))


def interactive_slicing(initial_lat, initial_lon, scatterplot = False):
    def plot_path(lat, lon, fig, ax):
        ax.cla()
        if not scatterplot:
            ax.plot(lat, lon, 'r-')
        else:
            point_size = 1.0
            ax.scatter(x=lat, y=lon, s=point_size, color='red')

        annotate_plot(lat, lon, ax, print_every = 200)
        fig.canvas.draw() #afto
        fig.canvas.flush_events() #afto
        plt.show(block=False)

    def looks_ok_prompt():
        print('====\nIs plot OK?')
        print('1. Yes')
        print('2. No')
        cmd = -1
        while cmd != 1 and cmd != 2:
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

        looks_ok = True
        if cmd == 2:
            looks_ok = False

        return looks_ok

    def enter_slices_prompt():
        print('Enter slices to cut plot:')
        cmd = -1
        while cmd == -1:
            try:
                # Prompt the user for an integer
                user_input = input("Slice Start: > ")
                
                # Try to convert the input to an integer
                integer_input = int(user_input)
                cmd = integer_input        
            except ValueError:
                # If a ValueError occurs, the input was not a valid integer
                cmd = -1
                print("Invalid input. Please enter an integer.")
        slice_start = cmd
        cmd = -1
        while cmd == -1:
            try:
                # Prompt the user for an integer
                user_input = input("Slice End: > ")
                
                # Try to convert the input to an integer
                integer_input = int(user_input)
                cmd = integer_input        
            except ValueError:
                # If a ValueError occurs, the input was not a valid integer
                cmd = -1
                print("Invalid input. Please enter an integer.")
        slice_end = cmd

        return slice_start, slice_end

    
    fig, ax = plt.subplots()
    slice_start = 0
    slice_end = 0

    print('Showing whole path first:')
    plot_path(initial_lat, initial_lon, fig, ax)


    cmd = -1
    looks_ok = False
    lat, lon = [], []
    while not looks_ok:
        looks_ok = looks_ok_prompt()

        if not looks_ok:
          slice_start, slice_end = enter_slices_prompt()
          lat, lon = slice_path(initial_lat, initial_lon, slice_start, slice_end)
          plot_path(lat, lon, fig, ax)

    plt.close()

    return lat, lon, slice_start, slice_end


def create_estimation_plot(drone_list, directory):   
    fig_estimation, ax_estimation = plt.subplots()
    plt.title("Collaborative Localization - Drone path | crps estimation")
    ax_estimation.set_xlabel("Longitude", labelpad=10)
    ax_estimation.set_ylabel("Latitude", labelpad=10)
        
    roque_lat = []
    roque_lon = []

    lat_tele_list = []
    lon_tele_list = []
    
    lat_fused_list = []
    lon_fused_list = []
    
    for drone in drone_list:
        if drone.is_roque():
            roque_lat, roque_lon = retrieve_geo(drone.telemetry, drone.name)
            roque_lat, roque_lon = slice_path(roque_lat, roque_lon, drone.telemetry_slice_start, drone.telemetry_slice_end)
            #roque_lat, roque_lon = interactive_slicing(roque_lat, roque_lon)

            ax_estimation.plot(roque_lat, roque_lon, label="Roque("+drone.name+") Path")
        else:
            lat, lon = retrieve_geo(drone.telemetry, drone.name)
            lat, lon = slice_path(lat, lon, drone.telemetry_slice_start, drone.telemetry_slice_end)

            lat_tele_list.append(lat)
            lon_tele_list.append(lon)

            #lat, lon = interactive_slicing(lat, lon)
            
            ax_estimation.plot(lat, lon, label="Follower("+drone.name+") Path")
            
            lat_fused, lon_fused = retrieve_geo(drone.fused, drone.name + '_fused')
            lat_fused, lon_fused = slice_path(lat_fused, lon_fused, drone.fused_slice_start, drone.fused_slice_end)
            
            lat_fused_list.append(lat_fused)
            lon_fused_list.append(lon_fused)

    lat_fused_v, lon_fused_v = [], []
    if len(lat_fused_list) > 0 and len(lon_fused_list) > 0:
        lat_fused_v, lon_fused_v = average_paths(lat_fused_list, lon_fused_list)
        #lat_fused_v, lon_fused_v = interactive_slicing(lat_fused_v, lon_fused_v)
        ax_estimation.scatter(x=lat_fused_v, y=lon_fused_v, s=point_size, color='red', label="Fused Estimation Line")
    
        print_every = 100
        for i in range(len(lat_fused_v)):
            if i % print_every == 0:
                ax_estimation.annotate(str(i), (lat_fused_v[i], lon_fused_v[i]))
        
    ax_estimation.legend(loc="lower right")
    fig_estimation.savefig(directory + 'plot_estimation.pdf', format='pdf')
    plt.show(block=False)
    print('Done.')

    fig_estimation_size = fig_estimation.get_size_inches()*fig_estimation.dpi

    create_animate_plot(drone_list, directory, fig_estimation_size, roque_lat, roque_lon, lat_tele_list, lon_tele_list, lat_fused_v, lon_fused_v, display_wind_data=True)


def create_animate_plot(drone_list, directory, fig_size, roque_lat, roque_lon, lat_tele_list, lon_tele_list, lat_fused_v, lon_fused_v, total_frames=100, display_wind_data=False):
    def plot_path(lat, lon, fig, ax, label='label', color='blue', scatterplot=False, annotate=False):
        if not scatterplot:
            ax.plot(lat, lon, label=label)
        else:
            point_size = 1.0
            ax.scatter(x=lat, y=lon, s=point_size, color='red', label=label)

        if annotate:
            annotate_plot(lat, lon, ax, print_every = 200)

    def display_fig(fig, frame_i):
        fig.canvas.draw() #afto
        fig.canvas.flush_events() #afto
        #plt.show(block=False)
        
        frame_path = directory + 'animation/' + 'plot_estimation_' + str(frame_i) + '.png'
        fig.savefig(frame_path, format='png')
        frame = cv2.imread(frame_path)

        # Write the frame to the video file
        video_writer.write(frame)

    print('\nCreating plot animation\n')
    if not os.path.exists(directory + 'animation/'):
        os.makedirs(directory + 'animation/')
        print(f"Directory '{directory + 'animation/'}' created successfully.")
    else:
        pass

    

    windSpeed = []
    windAngle = []

    left_drone_assigned = False
    roque_drone_instance = None
    camera_drone_list = []
    for drone in drone_list:
        if drone.is_roque():
            roque_drone_instance = drone
            windData = fu.read_wind_data(roque_drone_instance.db_file)
            # print('windData', windData)
            # if len(windData) > 0:
            #     windSpeed = windData[0]
            #     windAngle = windData[1]
            #     print('windSpeed', windSpeed)
            #     print('windAngle', windAngle)
        else:
            if not left_drone_assigned:
                drone.name ='Left'
                left_drone_assigned=True
            else:
                drone.name ='Right'
            camera_drone_list.append(drone)

    print('roque_tele_size(' + roque_drone_instance.name + '):', len(roque_lat))
    for i in range(len(lat_tele_list)):
        print('lat_tele_size(' + camera_drone_list[i].name + '):', len(lat_tele_list[i]))
    print('lat_fused_v_size:', len(lat_fused_v))


    lat_points = []
    lon_points = []
    print('roque_tele_size(' + roque_drone_instance.name + '):', len(roque_lat))
    lat_points.extend(roque_lat)
    lon_points.extend(roque_lon)
    for i in range(len(lat_tele_list)):
        print('lat_tele_size(' + camera_drone_list[i].name + '):', len(lat_tele_list[i]))
        lat_points.extend(lat_tele_list[i])
        lon_points.extend(lon_tele_list[i])
    print('lat_fused_v_size:', len(lat_fused_v))
    lat_points.extend(lat_fused_v)
    lon_points.extend(lon_fused_v)
    
    min_lat, max_lat = min(lat_points), max(lat_points)
    min_lon, max_lon = min(lon_points), max(lon_points)

    print('lat_points:', lat_points [0])
    print('lat_points_size:', len(lat_points))
    
    fig_animate, ax_animate = plt.subplots()
    
    width_pixels = 640
    height_pixels = 640
    dpi = 100

    width_inches = width_pixels / dpi
    height_inches = height_pixels / dpi
    

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    fps = 2
    output_file = directory + 'animation.mp4'
    video_writer = cv2.VideoWriter(output_file, fourcc, fps, (width_pixels, height_pixels))
    

    #title_label = "Collaborative Localization - Drone path | crps estimation"
    title_label = "Weather Station enabled drone - Data Collection"
    path_show_percentage = 0.0
    path_show_percentage_increase = 1.0 / total_frames
    for frame_i in range(total_frames):
        ax_animate.cla()
        plt.title(title_label, pad=14)
        #plt.autoscale(True)
        #fig_animate.set
        #ax_animate.axes.set_xlim(min_lat, max_lat)
        #ax_animate.axes.set_ylim(min_lon, max_lon)
        ax_animate.axes.set_xlim(min_lon, max_lon)
        ax_animate.axes.set_ylim(min_lat, max_lat)

        fig_animate.set_figwidth(width_inches)
        fig_animate.set_figheight(height_inches)


        ax_animate.axes.set_aspect('equal')
        #ax_animate.axes.set_aspect('equal')
        #fig_animate.canvas.setFixedSize(fig_size[0], fig_size[0])
        ax_animate.set_xlabel("Latitude", labelpad=10)
        ax_animate.set_ylabel("Longitude", labelpad=10)

        print('fig_size[0]:', fig_size[0])
        print('fig_size[1]:', fig_size[1])


        #fig_animate.set_size_inches(fig_size[0], fig_size[1])

        roque_lat_f, roque_lon_f = slice_path_percentage(roque_lat, roque_lon, path_show_percentage)

        #roque_path_label = "Roque Path"
        roque_path_label = "Data collection drone - Path"

        #plot_path(roque_lat_f, roque_lon_f, fig_animate, ax_animate, label="Roque("+drone.name+") Path", scatterplot=False, annotate=False)
        plot_path(roque_lon_f, roque_lat_f, fig_animate, ax_animate, label=roque_path_label, scatterplot=False, annotate=False)

        for drone_i in range(len(camera_drone_list)):
            lat_tele, lon_tele = slice_path_percentage(lat_tele_list[drone_i], lon_tele_list[drone_i], path_show_percentage) 
            #plot_path(lat_tele, lon_tele, fig_animate, ax_animate, label="Follower("+camera_drone_list[drone_i].name+") Path", scatterplot=False, annotate=False)
            plot_path(lon_tele, lat_tele, fig_animate, ax_animate, label=camera_drone_list[drone_i].name + " Follower Path", scatterplot=False, annotate=False)

        lat_fused_vf, lon_fused_vf = slice_path_percentage(lat_fused_v, lon_fused_v, path_show_percentage)
        #plot_path(lat_fused_vf, lon_fused_vf, fig_animate, ax_animate, label="Estimation Line", scatterplot=True, annotate=True)
        if len(lon_fused_vf) > 0 and len(lat_fused_vf) > 0:
            plot_path(lon_fused_vf, lat_fused_vf, fig_animate, ax_animate, label="Fused Estimation Line", scatterplot=True, annotate=True)

        if display_wind_data:
            windData_cur, _ =  slice_path_percentage(windData, windData, path_show_percentage)

            if len(windData_cur) > 0:
                windSpeed_cur_string = str(round(windData_cur[-1][0], 2))
                windAngle_cur_string = str(round(windData_cur[-1][1], 2))
            else:
                windSpeed_cur_string = '0.0'
                windAngle_cur_string = '0.0'

            windSpeed_label = 'Wind Speed: ' + windSpeed_cur_string
            windAngle_label = 'Wind Angle: ' + windAngle_cur_string

            plot_path([], [], fig_animate, ax_animate, label=windSpeed_label, scatterplot=True, annotate=False)
            plot_path([], [], fig_animate, ax_animate, label=windAngle_label, scatterplot=True, annotate=False)

        path_show_percentage = path_show_percentage + path_show_percentage_increase
        
        ax_animate.legend(loc="upper left")
        display_fig(fig_animate, frame_i)
        #time.sleep(1)
    

    print('Released video_writer')
    video_writer.release()


def calculate_error_timestamping(path, ground_truth_path, timestamp_error_threshhold = 5.0): 
    estimation_path = path
    roque_path = ground_truth_path
    
    print("path size:", len(path))
    print("ground_truth_path size:", len(ground_truth_path))
    
    roque_path_point = []
    estimation_path_point = []
    path_point_distance = []
    
    point_distance_sum = 0.0
    point_distance_avg = 0.0
    
    for point in estimation_path:    
        timestamp = retrieve_timestamp(point)
        timestamp_found = False
        
        coords_1 = (0, 0)
        coords_2 = (0, 0)
        
        print("point:", point)
        print("timestamp:", timestamp)
        lat, lon = retrieve_geo([point], enforce_alt_limit=True)
        print("lat:", lat)
        print("lon:", lon)
        
        if len(lat) * len(lon) > 0:
            coords_1 = (lat[0], lon[0])
            
            min_tele_point = None
            min_point_distance = 1000.0
            for tele_point in roque_path:
                tele_timestamp = retrieve_timestamp(tele_point)
                
                
                if abs(timestamp - tele_timestamp) < timestamp_error_threshhold:
                    timestamp_found = True
                    
                    print("tele_point:", tele_point)
                    print("tele_timestamp:", tele_timestamp)
                    
                    lat, lon = retrieve_geo([tele_point], enforce_alt_limit=False)
                    if len(lat) * len(lon) > 0:
                        coords_2 = (lat[0], lon[0])
                
                        print("coords_1:", coords_1)
                        print("coords_2:", coords_2)
                        print()
                
                        point_distance = distance.distance(coords_1, coords_2).m
                        #print(point_distance)
                        
                        if point_distance < min_point_distance:
                            min_point_distance = point_distance
                            min_tele_point = tele_point
                        
            if timestamp_found:
                coords_2 = (float(min_tele_point[9]), float(min_tele_point[8]))
                
                roque_path_point.append(coords_1)
                estimation_path_point.append(coords_2)
                path_point_distance.append(min_point_distance)
            point_distance_sum = point_distance_sum + min_point_distance

            #print ("Geodesic Distance:", min_point_distance, "[km]")
            
    point_distance_avg = point_distance_sum / len(path_point_distance)
        
    return roque_path_point, estimation_path_point, path_point_distance, point_distance_avg


def place_error_fig(ax, roque_path_point, estimation_path_point, path_point_distance, label="Ground Truth - Estimation Distance", place_line_every=20, annotate_line_every=10, color='go-'):
    markersize=1
    place_every = place_line_every
    
    print("roque_path_point size:", len(roque_path_point))
    print("estimation_path_point size:", len(estimation_path_point))
    print("path_point_distance size:", len(path_point_distance))
    
    roque_path_point_lat = []
    roque_path_point_lon = []
    
    estimation_path_point_lat = []
    estimation_path_point_lon = []

# =============================================================================
#     for sublist in roque_path_point:
#         roque_path_point_lat.append(sublist[0])
#         roque_path_point_lon.append(sublist[1]) 
#         
#     for sublist in estimation_path_point:
#         estimation_path_point_lat.append(sublist[0])
#         estimation_path_point_lon.append(sublist[1]) 
# =============================================================================
    
    for i in range(0, len(roque_path_point), 1):
        if i % place_every == 0:
            error_lat, error_lon = [], []
            
            error_lat.append(roque_path_point[i][0])
            error_lon.append(roque_path_point[i][1])
            
            error_lat.append(estimation_path_point[i][0])
            error_lon.append(estimation_path_point[i][1])
            
            if i == 0:
                plt.plot(error_lat, error_lon, color, markersize=markersize, label=label)
            else:
                plt.plot(error_lat, error_lon, color, markersize=markersize)
            
            if i % (place_every * annotate_line_every) == 0:
                dist_formatted = "{:.1f}".format(path_point_distance[i])
                label = str(i) + " - " + str(dist_formatted) + "m"
                ax.annotate(label, (error_lat[0], error_lon[0]))
    

def create_error_plot(drone_list, directory):
    fig_error, ax_error = plt.subplots()
    plt.title("Collaborative Localization - Drone/Estimation path - Error Plot")
    ax_error.set_xlabel("Longitude", labelpad=10)
    ax_error.set_ylabel("Latitude", labelpad=10)
        
    roque_lat = []
    roque_lon = []
    
    lat_fused_list = []
    lon_fused_list = []
    
    roque_drone = None
    camera_drones = []
    for drone in drone_list:
        if drone.is_roque():
            roque_drone = drone

            roque_lat, roque_lon = retrieve_geo(drone.telemetry, drone.name)
            
            sliced_roque_path = roque_drone.telemetry[drone.telemetry_slice_start:-drone.telemetry_slice_end]
            roque_lat, roque_lon = slice_path(roque_lat, roque_lon, drone.telemetry_slice_start, drone.telemetry_slice_end)
            
            ax_error.plot(roque_lat, roque_lon, label="Roque("+drone.name+") Path")    
        else:
            lat, lon = retrieve_geo(drone.telemetry, drone.name)
            lat, lon = slice_path(lat, lon, drone.telemetry_slice_start, drone.telemetry_slice_end)

            ax_error.plot(lat, lon, label="Follower("+drone.name+") Path")
            
            lat_fused, lon_fused = retrieve_geo(drone.fused, drone.name + '_fused')
            lat_fused, lon_fused = slice_path(lat_fused, lon_fused, drone.fused_slice_start, drone.fused_slice_end)
            
            lat_fused_list.append(lat_fused)
            lon_fused_list.append(lon_fused)
            
            camera_drones.append(drone)
          
            
    #fused_path_slice_start = 100
    #fused_path_slice_end = 100
    roque_path_point, estimation_path_point, path_point_distance = [], [], []
    point_distance_avg_error = 0.0
    for drone in camera_drones:
        sliced_tele_path = drone.telemetry[drone.telemetry_slice_start:-drone.telemetry_slice_end]
        sliced_fused_path = drone.fused[drone.fused_slice_start:-drone.fused_slice_end]

        roque_path_point, estimation_path_point, path_point_distance, point_distance_avg_error = calculate_error_timestamping(sliced_fused_path, sliced_roque_path)
        place_error_fig(ax_error, roque_path_point, estimation_path_point, path_point_distance)

        roque_path_point_dist, camera_path_point, roque_camera_distance, point_distance_avg_error = calculate_error_timestamping(sliced_roque_path, sliced_tele_path, timestamp_error_threshhold=10.0)
        place_error_fig(ax_error, roque_path_point_dist, camera_path_point, roque_camera_distance, label="Follower - Roque Distance",  place_line_every=200, annotate_line_every=1, color='yo-')


    lat_fused_v, lon_fused_v = average_paths(lat_fused_list, lon_fused_list)
    #lat_fused_v, lon_fused_v = slice_path(lat_fused_v, lon_fused_v, fused_path_slice_start, fused_path_slice_end)
    ax_error.scatter(x=lat_fused_v, y=lon_fused_v, s=point_size, color='red', label="Estimation Line")
    
    
    point_distance_avg_error_formatted = "{:.2f}".format(point_distance_avg_error)
    
    print("roque_path_point size:", len(roque_path_point))
    print("estimation_path_point size:", len(estimation_path_point)) 
    print("point_distance_avg_error:", point_distance_avg_error_formatted, "[m]")
    
    roque_path_label = "roque_path_point size: " + str(len(roque_path_point))
    estimation_path_label = "estimation_path_point size: " + str(len(estimation_path_point))
    error_label = "point_distance_avg_error: " + str(point_distance_avg_error_formatted) + "[m]"
    top_left_label = roque_path_label + "\n" + estimation_path_label + "\n" + error_label
    top_left_label = error_label
    ax_error.text(.01, .99, top_left_label, ha='left', va='top', transform = ax_error.transAxes)
    
    fig_error_dist, ax_error_dist = plt.subplots()
    ax_error_dist.set_xlabel("Camera - Drone Distance [m]", labelpad=10)
    ax_error_dist.set_ylabel("Error [m]", labelpad=10)
    
    path_point_distance_norm, roque_camera_distance_norm = normalize_lists_population(path_point_distance, roque_camera_distance)
    ax_error_dist.plot(roque_camera_distance_norm, path_point_distance_norm, label="Error[m] in relation to the drone-camera distance") 
    
    ax_error_dist.legend(loc="lower right")
    ax_error.legend(loc="lower right")
    fig_error.savefig(directory + 'plot_estimation.pdf', format='pdf')
    plt.show(block=False)
    print('Done.')


def create_plot(drone_list):
    list_size = len(drone_list)
    
    fig, ax = plt.subplots(list_size, 6)
    fig.set_facecolor("w")
    
    plt.title("Collaborative Localization - Drone path | crps estimation")

    #plt.legend(loc="lower right")
    point_size = 1.0
    
    roque_lat = []
    roque_lon = []
    
    fig_overlap, ax_overlap = plt.subplots()
    plt.title("Collaborative Localization - Drone path | crps estimation")
    ax_overlap.set_xlabel("Longitude", labelpad=10)
    ax_overlap.set_ylabel("Latitude", labelpad=10)

    lat_fused_list = []
    lon_fused_list = []
    
    lat_aug_list, lon_aug_list = [], []
    
    r = 0
    for drone in drone_list:
        lat, lon = retrieve_geo(drone.telemetry, drone.name)
        lat_crps, lon_crps = retrieve_geo(drone.crps, drone.name + '_crps')
        lat_fused, lon_fused = retrieve_geo(drone.fused, drone.name + '_fused')
        
        lat_mov_avg, lon_mov_avg = smooth_path(lat_fused, lon_fused)
        
        lat_mi_og, lon_mi_og = [], []
        lat_mi_au, lon_mi_au = [], []
             
        is_roque_drone = False
        if (len(lat_crps) * len(lon_crps)) * (len(lat_fused) * len(lon_fused)) == 0:
            is_roque_drone = True
            roque_lat = lat
            roque_lon = lon
            drone.set_roque(True)
        else:
            normalize_latlon_population(lat_crps, lon_crps, lat_fused, lon_fused)
            
            slice_start = 0
            slice_end = 0
            
            # Calculate monitoring internal
            lat_mi_og, lon_mi_og = drone.calculate_monitoring_internal_original()
            lat_mi_au, lon_mi_au = drone.calculate_monitoring_internal_augmented()
        
            #lat_mi_og, lon_mi_og = slice_path(lat_mi_og, lon_mi_og, slice_start, slice_end)
            #lat_mi_au, lon_mi_au = slice_path(lat_mi_au, lon_mi_au, slice_start, slice_end)
            
            #lat_crps, lon_crps = slice_path(lat_crps, lon_crps, slice_start, slice_end)
            #lat_fused, lon_fused = slice_path(lat_fused, lon_fused, slice_start, slice_end)
            
            lat_mov_avg, lon_mov_avg = smooth_path(lat_fused, lon_fused)
            
            lat_fused_list.append(lat_mov_avg)
            lon_fused_list.append(lon_mov_avg)
            
            lat_aug_list.append(lat_mi_au)
            lon_aug_list.append(lon_mi_au)
            
        
        
        '''
        print('\nValue Range:', drone.name)
        lat_min = min(lat)
        lat_max = max(lat)
        
        lon_min = min(lon)
        lon_max = max(lon)
        
        print("lat_min:", lat_min)
        print("lat_max:", lat_max)
        print('')
        print("lon_min:", lon_min)
        print("lon_max:", lon_max)
        '''
        
        if list_size > 1:
            ax[r, 0].plot(lat, lon, label=drone.name)
            ax[r, 0].set_title(drone.name)
            if r == list_size - 1:
                ax[r, 0].set_xlabel("Longitude", labelpad=8)
            ax[r, 0].set_ylabel("Latitude", labelpad=8)
            #ax[r, 0].legend(loc="lower right")
            
            #ax[r, 1].plot(lat_crps, lon_crps, label=drone.name+"_crps")
            ax[r, 1].scatter(x=lat_crps, y=lon_crps, s=point_size, label=drone.name+"_crps")
            ax[r, 1].set_title(drone.name+"_crps")
            #ax[r, 1].legend(loc="lower right")
            
            #ax[r, 2].plot(lat_fused, lon_fused, label=drone.name+"_fused")
            ax[r, 2].scatter(x=lat_fused, y=lon_fused, s=point_size, label=drone.name+"_fused")
            ax[r, 2].set_title(drone.name+"_fused")
            #ax[r, 2].legend(loc="lower right")
            
            #ax[r, 3].plot(lat_mov_avg, lon_mov_avg, label=drone.name+"_fused_smoothed")
            ax[r, 3].scatter(x=lat_mov_avg, y=lon_mov_avg, s=point_size, label=drone.name+"_fused_smoothed")
            ax[r, 3].set_title(drone.name+"_fused_smoothed")
            #ax[r, 3].legend(loc="lower right")
            
            
            #ax[r, 4].plot(lat_mi_og, lon_mi_og, label=drone.name+"_mon_int_original")
            ax[r, 4].scatter(x=lat_mi_og, y=lon_mi_og, s=point_size, label=drone.name+"_mon_int_original")
            ax[r, 4].set_title(drone.name+"_mon_int_original")
            
            #ax[r, 5].plot(lat_mi_au, lon_mi_au, label=drone.name+"_mon_int_augmented")
            ax[r, 5].scatter(x=lat_mi_au, y=lon_mi_au, s=point_size, label=drone.name+"_mon_int_augmented")
            ax[r, 5].set_title(drone.name+"_mon_int_augmented")
        else:
            ax[0].plot(lat, lon, label=drone.name)      
            ax[1].plot(lat_crps, lon_crps, label=drone.name+"_crps")      
            ax[2].plot(lat_fused, lon_fused, label=drone.name+"_fused")
            ax[3].plot(lat_mov_avg, lon_mov_avg, label=drone.name+"_fused_smoothed")
            
            ax[0].set_xlabel("Longitude", labelpad=8)
            ax[0].set_ylabel("Latitude", labelpad=8)
            
        if is_roque_drone:
            ax_overlap.plot(lat, lon, label="Roque:" + drone.name)
        else:
            #ax_overlap.plot(lat, lon, label="Drone:" + drone.name)
            #ax_overlap.plot(lat_mov_avg, lon_mov_avg, label="Drone Estimation:" + drone.name)
            pass
            
        r = r + 1
    
     
    
    
    lat_fused_v, lon_fused_v = average_paths(lat_fused_list, lon_fused_list)
    #ax_overlap.plot(lat_fused_v, lon_fused_v, label="Estimation Line")
    ax_overlap.scatter(x=lat_fused_v, y=lon_fused_v, s=point_size, color='green', label="Estimation Line")
    
    for drone in drone_list:
        if not drone.is_roque():
            lat_d, lon_d = retrieve_geo(drone.telemetry, drone.name)
            ax_overlap.plot(lat_d, lon_d, label="Camera Drone position")
    
    
    print('\nError - Fused - Ground Truth')
    if len(lat_fused_v) * len(lon_fused_v):
        calculate_error(lat_fused_v, lon_fused_v, roque_lat, roque_lon)
    
    for i in range(len(lat_aug_list)):
        #ax_overlap.plot(lat_aug_list[i], lon_aug_list[i], label="Monitoring Internal recalculated (" + str(i) + ")")
        
        print('\nError - Monitoring Internal recalculated (' + str(i) + ')- Ground Truth') 
        calculate_error(lat_aug_list[i], lon_aug_list[i], roque_lat, roque_lon)
    
    


    
    error_x_list, error_y_list = [], []
    error_meters_x_list, error_meters_y_list = [], []
    error_meters_sum_list = []
    
# =============================================================================
#     error_factor_list = np.arange(0.1, 2.1, 0.1)
#     
#     for drone in drone_list:
#         if len(drone.monitoring_internal) > 0:
#             for error_factor in error_factor_list:
#                 print('Finding error values for error_factor of ' + str(error_factor))
#                 lat_mi_au, lon_mi_au = drone.calculate_monitoring_internal_augmented(error_factor=error_factor)
#                 lat_mi_au, lon_mi_au = slice_path(lat_mi_au, lon_mi_au, slice_start, slice_end)
#                 
#                 normalize_latlon_population(lat_mi_au, lon_mi_au, roque_lat, roque_lon)
#                 
#                 #print('len lat_mi_au_norm:', len(lat_mi_au_norm))
#                 #print('len lon_mi_au_norm:', len(lon_mi_au_norm))
#                 
#                 #print('len roque_lat_norm:', len(roque_lat_norm))
#                 #print('len roque_lon_norm:', len(roque_lon_norm))
#                 
#                 #calculate_error(lat_mi_au_norm, lon_mi_au_norm, roque_lat_norm, roque_lon_norm)
#                 
#                 error_x, error_y, error_meters_x, error_meters_y = calculate_error(lat_mi_au, lon_mi_au, roque_lat, roque_lon)
#                 
#                 
#                 error_x_list.append(error_x)
#                 error_y_list.append(error_y)
#                 
#                 error_meters_x_list.append(error_meters_x)
#                 error_meters_y_list.append(error_meters_y)          
#                 print('============')
# =============================================================================
                
    for i in range(len(error_meters_x_list)):
        error_sum = (error_meters_x_list[i] + error_meters_y_list[i]) / 2
        error_meters_sum_list.append(error_sum)
        
    fig_error, ax_error = plt.subplots(2)
    plt.title("Collaborative Localization - Error factor")
    #print(error_x_list)
# =============================================================================
#     ax_error[0].plot(error_factor_list, error_x_list, label="error X")
#     ax_error[0].plot(error_factor_list, error_y_list, label="error Y")
#     
#     ax_error[1].plot(error_factor_list, error_meters_x_list, label="error X")
#     ax_error[1].plot(error_factor_list, error_meters_y_list, label="error Y")
#     ax_error[1].plot(error_factor_list, error_meters_sum_list, label="error Sum")
#     
#     ax_error[0].set_xlabel("error_factor", labelpad=10)
#     ax_error[0].set_ylabel("Average Error (decimal degrees)", labelpad=10)
#     ax_error[1].set_ylabel("Average Error (Meters)", labelpad=10)
#     
#     ax_error[0].legend(loc="lower right")
#     ax_error[1].legend(loc="lower right")
# =============================================================================
    ax_overlap.legend(loc="lower right")
    
    min_index = 0
    min_error = 10000000
    for i in range(len(error_meters_sum_list)):
        if error_meters_sum_list[i] < min_error:
            min_index = i
            min_error = error_meters_sum_list[i]
            
    print('min_error:', min_error, "[m]")
# =============================================================================
#     print('min_error_factor:', error_factor_list[min_index])
# =============================================================================
   
    print('\nBuilding Plot...')
    #plt.savefig("plot.pdf")
    
    fig.savefig('plot.pdf', format='pdf')
    fig_overlap.savefig('plot_overlap.pdf', format='pdf')
    plt.show(block=False)
    print('Done.')   
    try:
        plt.show()
    except:
        pass

