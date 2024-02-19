#!/usr/bin/env python


import csv
from datetime import datetime


csv_filename = "/home/jetson/catkin_ws/logs/" + datetime.now().strftime("%d%m%Y_%H%M%S") + ".csv" 
with open(csv_filename, 'a') as csvfile:
	writer = csv.writer(csvfile, delimiter=',')
	writer.writerow(['perc', 'volt', 'curr'])
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])

with open(csv_filename, 'a') as csvfile:
	writer = csv.writer(csvfile, delimiter=',')
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])
	writer.writerow([9,10,11])


