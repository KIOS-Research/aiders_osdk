#!/bin/bash
#
#add this prior the code sudo chmod 666 /dev/ttyTHS0
### BEGIN INIT INFO
# Provides:          OSDK-ROS
# Short-Description: OSDK-ROS Integration
# Description:       This script starts all required tasks for
#					dji osdk and ros with a multimaster approach
### END INIT INFO
#
# Author: Christos Georgiades

#if [ true != "$INIT_D_SCRIPT_SOURCED" ] ; then
#    set "$0" "$@"; INIT_D_SCRIPT_SOURCED=true . /lib/init/init-d-script
#fi




source ~/.bashrc
#echo "Sourced user's .bashrc"
echo "User: $(whoami)"
echo "PWD: $PWD"
echo "PYTHONPATH: $PYTHONPATH"
echo "========"


LAUNCH_PATH="$( cd -- "$(dirname "$0")" </dev/null 2>&1; pwd -P )"
cd $LAUNCH_PATH
cd ..

CATKIN_PATH=$(pwd)
SCRIPTS_PATH="$CATKIN_PATH/src/dji_sdk/scripts"

FLIGHTLOG_TIMESTAMP=$(date +%Y-%m-%d-T%H-%M-%S)
MACHINE_ID=$(cat /etc/machine-id | cut -c1-4)

LOG_PATH="$CATKIN_PATH/flightlogs"
FLIGHTLOG_PATH="matrice300_$MACHINE_ID-$FLIGHTLOG_TIMESTAMP/"

if [ -z "$1" ]; then
    echo "No FLIGHTLOG_PATH argument provided."
else
    FLIGHTLOG_PATH="$1/"
    echo "Set FLIGHTLOG_PATH to: $FLIGHTLOG_PATH"
fi



if [ ! -d "$LOG_PATH" ]; then
    mkdir $LOG_PATH
    echo "Creating Log Directory"
fi
if [ ! -d "$LOG_PATH/$FLIGHTLOG_PATH" ]; then
    mkdir $LOG_PATH/$FLIGHTLOG_PATH
    echo "Creating Flight Log Directory"
fi

echo "========"
echo "CATKIN_PATH: $CATKIN_PATH"
echo "LAUNCH_PATH: $LAUNCH_PATH"
echo "SCRIPTS_PATH: $SCRIPTS_PATH"
echo "FLIGHTLOG_PATH: $LOG_PATH/$FLIGHTLOG_PATH"
echo "========"

if [ ! -d "$CATKIN_PATH/devel" ] | [ ! -d "$CATKIN_PATH/build" ]; then
	echo "Missing devel or build directories. Program will exit."
	echo "Run catkin_make before running again"
	
	exit
fi

#Get Active Ip
ROS_PORT=11311
ROS_IP=$(python3 $LAUNCH_PATH/getActiveIP.py)
ROS_MASTER_URI="http://$ROS_IP:$ROS_PORT"

ubuntu_version=$(lsb_release -sc)
ros_version=$(rosversion -d)

echo "ubuntu_version=$ubuntu_version"
echo "ros_version=$ros_version"
echo "ROS_IP=$ROS_IP"
echo "ROS_IP=$ROS_MASTER_URI"
echo "========"


# Kill previous instances
echo "Killing previous instances..."
netstat -taepn 2>/dev/null | grep 11611
netstat -taepn 2>/dev/null | grep 11611 | cut -d/ -f1 | awk '{print $9}' | xargs kill -9 2> /dev/null

# kill ros instances
ps -aux | grep -i /opt/ros | awk '{print $2}' | xargs -n1 kill 2> /dev/null

# kill aiders_osdk instances
ps -aux | grep -i aiders_osdk | awk '{print $2}' | xargs -n1 kill 2> /dev/null


echo -e "========\n"

cust_func(){
    source $CATKIN_PATH/devel/setup.bash
    #echo Sourced $CATKIN_PATH/devel/setup.bash
    export "PYTHONUNBUFFERED=1"
    export "ROS_IP=${ROS_IP}"
    export "ROS_MASTER_URI=${ROS_MASTER_URI}"
    
    echo -e "\nLaunching $1...\n"

	case "$1" in	
    6)  python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/connectionHandler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/component_communication/component_handler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/metric_publisher.py $FLIGHTLOG_PATH &
    ;;

    7)  # tf2 does not work unless we source setup.bash
        source $CATKIN_PATH/devel/setup.bash
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_positioning/telemetryTranslator.py &
    ;;

    8)  python3 $CATKIN_PATH/src/dji_sdk/scripts/logging_system/logging_handler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/database_synchronization/databaseHandler.py &
    ;;

    9)	export "OPENBLAS_CORETYPE=ARMV8"
        export "OPENBLAS_MAIN_FREE=1"
        export "OPENBLAS_NUM_THREADS=1"
        export DISPLAY=:0
        
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/controlAuthorityHandler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightController.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightStateController.py &


        #python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightController.py &
        #python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightLogic.py &
        #python3 -u $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/autonomous_landing/landing_pad_detection/aruqr_detection.py &
    ;;
    
    10)	#python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/fault_detection.py &
    ;;
    
    11)	python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_camera/orthocamera.py &
    ;;
    
    12) #python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/connectionHandler.py &
    ;;
    
    13) #python2 $CATKIN_PATH/src/dji_sdk/scripts/multispectralHandler.py &
    ;;
    
    14) #python2 $CATKIN_PATH/src/dji_sdk/scripts/weatherstationHandler.py &
    ;;
    
    15) #python2 $CATKIN_PATH/src/dji_sdk/scripts/component_communication/watersamplerHandler.py &
    ;;

    16) #python2 $CATKIN_PATH/src/dji_sdk/scripts/ballisticsHandler.py &
    ;;
    
    17) export "OPENBLAS_CORETYPE=ARMV8"
        export "OPENBLAS_MAIN_FREE=1"
        export "OPENBLAS_NUM_THREADS=1"
        export DISPLAY=:0

        python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/crps_measurement_fusion_coordinator.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/crps_collaboration_handler.py &
    	
    	#python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/Monitoring.py &
        
        #python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/track_module.py &
    ;;

    18) python3 $CATKIN_PATH/src/dji_sdk/scripts/raft_consensus/raft_client.py &
    ;;

    ### uncategorized
    19) #rosrun joy joy_node
        #python3 $CATKIN_PATH/src/dji_sdk/scripts/joyController.py &
        #python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/user_controllers/keyboard_controller.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/rangefinder/rangefinder_handler.py &
    ;;
    
    "ros")
        export "ROS_IP=${ROS_IP}"
        export "ROS_MASTER_URI=${ROS_MASTER_URI}"
        
        roscore &
        
        #master_discovery
        if [ "$ros_version" = "melodic" ]; then
            rosrun master_discovery_fkie master_discovery &> master_discovery_fkie.out
        elif [ "$ros_version" = "noetic" ]; then
            rosrun fkie_master_discovery master_discovery
        fi

        #master_sync
        if [ "$ros_version" = "melodic" ]; then
            rosrun master_sync_fkie master_sync &> master_sync_fkie.out
        elif [ "$ros_version" = "noetic" ]; then
            rosrun fkie_master_sync master_sync
        fi
    ;;

    "rosbridge")
        roslaunch rosbridge_server rosbridge_websocket.launch &
    ;;

    "dji")
        if [[ -n $(ls /dev/ttyACM0) ]]; then
            roslaunch --wait dji_sdk dji_vehicle_node.launch &
        else
            echo -e "/dev/ttyACM0 not found. skipping dji_sdk initialization...\n"
        fi
    ;;

    "telemetry")
        source $CATKIN_PATH/devel/setup.bash
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_positioning/telemetryTranslator.py &
    ;;

    "metrics")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/metric_publisher.py $FLIGHTLOG_PATH &
    ;;

    "components")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/component_communication/component_handler.py &
        #python2 $CATKIN_PATH/src/dji_sdk/scripts/multispectralHandler.py &
        #python2 $CATKIN_PATH/src/dji_sdk/scripts/weatherstationHandler.py &
        #python2 $CATKIN_PATH/src/dji_sdk/scripts/component_communication/watersamplerHandler.py &
        #python2 $CATKIN_PATH/src/dji_sdk/scripts/ballisticsHandler.py &
        #python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/rangefinder/rangefinder_handler.py &
    ;;

    "platform")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/app_communication/connectionHandler.py &
    ;;

    "tasks")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/controlAuthorityHandler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightController.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/drone_movement/flightStateController.py &
    ;;

    "crps")
        export "OPENBLAS_CORETYPE=ARMV8"
        export "OPENBLAS_MAIN_FREE=1"
        export "OPENBLAS_NUM_THREADS=1"
        export DISPLAY=:0

        python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/crps_measurement_fusion_coordinator.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/collaborative_localization/crps_collaboration_handler.py &
    ;;

    "raft")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/raft_consensus/raft_client.py &
    ;;

    "log")
        python3 $CATKIN_PATH/src/dji_sdk/scripts/logging_system/logging_handler.py &
        python3 $CATKIN_PATH/src/dji_sdk/scripts/database_synchronization/databaseHandler.py &
    ;;

    "joy")
        #
    ;;

    esac
}

source $CATKIN_PATH/devel/setup.bash
echo "Sourced devel/setup.bash"

# Define a list of strings
exec_list=("ros" "dji"  "telemetry" "metrics" "components" "platform" "tasks" "crps" "raft" "log")
if [ "$#" -eq 0 ]; then
    echo "No arguments provided. Usage: $0 arg1 arg2 arg3 ..."
else
    exec_list=("$@")
fi


echo "Will execute the following bundles: "${exec_list[@]}""
for exec_bundle in "${exec_list[@]}"; do
    #echo "exec_bundle: $exec_bundle"
    cust_func $exec_bundle &
    sleep 2
done

# for i in {1..1}
# do
# 	# Call script/node as a background process	
# 	#cust_func $i &

    

#     if [ "$i" -le 5 ]; then
#         # ROS initialization sometimes causes key-matching issues
#         # Longer sleep, can help overcome issue
#         sleep 4
#     else
#         sleep 1
#     fi	
# done
 
wait 
