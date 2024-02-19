
#export "LAUNCH_PATH="$( cd -- "$(dirname "$0")" </dev/null 2>&1; pwd -P )""
export "LAUNCH_PATH=$(dirname "$(readlink -f "$BASH_SOURCE")")"

#echo $LAUNCH_PATH

#Get Active Ip
ROS_PORT=11311
ROS_IP=$(python3 $LAUNCH_PATH/getActiveIP.py)
ROS_MASTER_URI="http://$ROS_IP:$ROS_PORT"

MACHINE_ID=$(cat /etc/machine-id | cut -c1-4)
DRONENAME="matrice300_$MACHINE_ID"

ubuntu_version=$(lsb_release -sc)
ros_version=$(rosversion -d)

echo "ubuntu_version=$ubuntu_version"
echo "ros_version=$ros_version"
echo ""
echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "ROS_NAMESPACE=$DRONENAME"


export "ROS_IP=${ROS_IP}"
export "ROS_MASTER_URI=${ROS_MASTER_URI}"
#export "ROS_NAMESPACE=$DRONENAME"

