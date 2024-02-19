import subprocess


class Device:
	def __init__(self, device_string):
		self.parse_device_string(device_string)
		
	def parse_device_string(self, device_string):
		self.bus_no = None
		self.dev_no = None
		self.id = None
		self.name = None
		self.dev_path = None
		
		
		device_string, self.dev_path = device_string.split(' - ')
		_, self.bus_no, _, self.dev_no, _, self.id, _, self.name = device_string.split(' ', 7)
		self.dev_no = self.dev_no.replace(':', '')
		#self.name = device_string
		
	def __str__(self):
		return f"{self.name}\n\tPath:\t{self.dev_path}\n\tBus:\t{self.bus_no}\tDev:\t\t{self.dev_no}\n\tID:\t{self.id}"


def print_prompt():
	print('1. List Devices')
	print('2. Find by Name')
	print('3. Find by Id')
	print('4. Find by path')
	print('9. Exit')


def read_prompt():
	print('Enter command: ', end='')
	terminal_input = input()
	print('====')
	return terminal_input

devices_lsusb = subprocess.check_output(["./list_usb_devices.sh"]).decode('utf')

def list_devices(devices_list):
	for device in devices_list:
		print(device)


def find_by_name(devices_list):
	print('Search Name: ', end='')
	target_name = input(). lower()
	print('====')
	
	for device in devices_list:
		if target_name in device.name.lower():
			print(device)

def find_by_id(devices_list):
	print('Search Id: ', end='')
	target_id = input(). lower()
	print('====')
	
	for device in devices_list:
		if target_id in device.id.lower():
			print(device)
			
def find_by_path(devices_list):
	print('Search Path: ', end='')
	target_path = input(). lower()
	print('====')
	
	for device in devices_list:
		if target_path in device.dev_path.lower():
			print(device)

print (devices_lsusb)

print('Cleaning up..\n')

devices_list = []
for device_string in devices_lsusb.split('\n'):
	if device_string.strip():
		devices_list.append(Device(device_string))
		#print(devices_list[-1])
		

		
cmd = 0	
while(cmd != 9):
	print_prompt()
	cmd = read_prompt()
	try:
		cmd = int(cmd)
	except:
		cmd = 0
	#print(cmd)
	if cmd == 1:
		list_devices(devices_list)
	if cmd == 2:
		find_by_name(devices_list)
	if cmd == 3:
		find_by_id(devices_list)
	if cmd == 4:
		find_by_path(devices_list)
	
	if cmd != 9:
		print('====')
		
print('\nGoodbye.')	
	
