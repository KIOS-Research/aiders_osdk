# Installation

The project requires DJI-OSDK to be installed on your device, as the backend provided by DJI needs some compiled libraries and dependencies
Furthermore, CUDA, NVCC libraries should be properly setup and TensorRT compliled with support for python2, python3 and CUDA.

### Install DJI Onboard-SDK  ###
```
git clone https://github.com/dji-sdk/Onboard-SDK.git

cd Onboard-SDK/

mkdir build; cd build

cmake ..

make

make install
```

### ths permissions ###
Setup THS permissions for /etc/ttyTHS0 and /etc/ttyACM0

```
chmod 666 /etc/ttyTHS0
chmod 666 /etc/ttyACM0
```

### usb permissions ###
Depending on the usb devices you use you might need some additional permission for /etc/ttyUSBx

```
chmod 666 /etc/ttyUSBx
```

### catkin make ###
To build this project properly, you need to compile it for python3. You can do this as such:
```
alias catkin_make3="catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3"
catkin_make3
```

### Start ###
From the aiders_osdk/ directory:

```
launch/AutostartMultimasterOSDK.sh 2> std.err
```

Redirecting error output to a file helps with debugging during runtime.
The dji_sdk produces a h264 codec error when getting frames from the main camera,
which is a good indicator for testing if publishing frames to ros works, but it can quickly clutter output

# Debug
grep -v will remove h264 codec errors from the error output stream

```
cat std.err | grep -v h264
```


### Autostart ###
A service for launching the application during startup is provided.
The autostart service requires a 3G dongle to work properly and uses the WLAN module to host a hotspot for SSH access and debugging when video output is not available.

Output and Error stream is logged to 'aiders_osdk/autostart/autostartService.log'

To read the output of the autostart service;

```
cat aiders_osdk/autostart/autostartService.log | grep -v h264
```


If first time running on a device, you can install service files like so;
```
cd aiders_osdk/autostart/

./autostartInstaller.sh		# You will be asked for superuser password
```

# Enable Service
systemctl enable aiders_osdk.service

# Disable Service
systemctl disable aiders_osdk.service

# Display Service Status 
systemctl status aiders_osdk.service
```

### Enable/Disable Program Modules ###
Open the following file and uncomment/comment the modules to be used
```
vim launch/AutostartMultimasterOSDK.sh
```

