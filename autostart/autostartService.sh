#!/bin/bash

# Define a timestamp function
timestamp() {
    date "+%Y/%m/%d-%H:%M:%S"
}

#echo $(ps -aux | grep openvpn)
ps -aux | grep openvpn | awk '{ print $2}' | xargs -n1 sudo kill -9


echo $(timestamp) - Service started

connected_monitors=$(xrandr | grep ' connected' | wc -l)
ip3g=$(lshw 2> /dev/null | grep CDC | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
ipTun=$(ip addr show dev tun0 | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
echo $(timestamp) - 3G IP: $ip3g

while [ -z "$ip3g" ]
do
    echo $(timestamp) - Polling 3g
    sleep 5
    
    ip3g=$(lshw 2> /dev/null | grep CDC | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
    echo $(timestamp) - 3G IP: $ip3g 
done
echo $(timestamp) - 3G OKAY

machine_id=$(cat /etc/machine-id | cut -c1-4)
echo $(timestamp) - Machine-Id found - $machine_id

#echo $(timestamp) - Starting hotspot
#./Scripts/hotspot.sh &
#sleep 5
#echo $(timestamp) - Hotspot OK


echo $(timestamp) - Starting VPN connection
sudo openvpn --client --config ../configs/matrice300_$machine_id.ovpn --daemon
sleep 5
echo $(timestamp) - VPN connection Established

while [ -z "$ipTun" ]
do
    echo $(timestamp) - Polling VPN connection
    sleep 5
     
    ipTun=$(ip addr show dev tun0 | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
    echo $(timestamp) - TUN IP: $ipTun
done
echo $(timestamp) - VPN OKAY

ps -aux | grep -i ntp | awk '{print $2}' | xargs -n1 sudo kill -9
#sudo ntpdate cy.pool.ntp.org


echo $PWD
directoryOwner=$(echo $PWD | awk -F '/' '{print $3}')
echo "Directory User: $directoryOwner"

#su $directoryOwner
#eval echo ~$USER


echo $(timestamp) - Starting SCRIPT... 
CTS=$(date +%Y-%m-%d-T%H-%M-%S)
LOG_DIRECTORY=matrice300_$machine_id-$CTS

echo "Creating Log Directory"
cd /home/$directoryOwner/Documents/aiders_osdk
#mkdir flightlogs
#cd flightlogs/
#mkdir matrice300_$machine_id-$CTS
su - -c "cd Documents/aiders_osdk; mkdir flightlogs; cd flightlogs/; mkdir matrice300_$machine_id-$CTS" $directoryOwner
#su -c 'script -qec "command 2>&1 | tee output.txt" /dev/null'
#su - -c "cd Documents/aiders_osdk; source ./devel/setup.bash; ./launch/AutostartMultimasterOSDK.sh $LOG_DIRECTORY 2>&1 | tee ./flightlogs/$LOG_DIRECTORY/std-$CTS.out" $directoryOwner
#su - -c "cd Documents/aiders_osdk; source ./devel/setup.bash; script./launch/AutostartMultimasterOSDK.sh $LOG_DIRECTORY 2>&1 | tee ./flightlogs/$LOG_DIRECTORY/std-$CTS.out" $directoryOwner
su - -c "source ~/.bashrc; cd Documents/aiders_osdk; source ./devel/setup.bash; script -c './launch/AutostartMultimasterOSDK.sh $LOG_DIRECTORY 2>&1' > ./flightlogs/$LOG_DIRECTORY/std-$CTS.out" $directoryOwner

wait
