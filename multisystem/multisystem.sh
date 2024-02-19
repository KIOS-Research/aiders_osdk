#!/bin/bash

#ssh kios@172.20.236.40
#cd ~/Documents/aiders_osdk/; whoami; ls;

# "kios"

user_array=("jetson" "kios" "jetson" )
vpn_array=( "10.8.0.2" "10.8.0.3" "10.8.0.4" )
ip_array=("172.20.245.233" "172.20.239.128" "172.20.245.181")

echo_options () {
	echo -e "1. Update Git Repos\n2. Start Drone OSDK\n3. Stop Drone OSDK\n4. Start VPN\n5. Stop VPN\n9. Exit"
}



force_remote_pull () {
	echo "Updating repository for user $1 at $2"
	ssh -o ConnectTimeout=5 -A $1@$2 "cd ~/Documents/aiders_osdk/; git pull http://cgeorg15:MzKpzCtXqTZwFD1JVzdw@kiosgitlab.koios.ucy.ac.cy/aiders/aiders_osdk"
}


### Start/Stop OSDK ###
start_remote () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
	ssh -o ConnectTimeout=5 -A $1@$2 "cd ~/Documents/aiders_osdk/; nohup launch/AutostartMultimasterOSDK.sh &> std.err &"
}

stop_remote () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
	ssh -o ConnectTimeout=5 -A $1@$2 "cd ~/Documents/aiders_osdk/; launch/AutostartMultimasterOSDK.sh 2> std.err "
}

### Start/Stop VPN ###
start_vpn () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
	ssh -to ConnectTimeout=5 -A $1@$2 "cd ~/Documents/aiders_osdk/configs; ./connect2vpn.sh"
}

stop_vpn () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
	ssh -o ConnectTimeout=5 -A $1@$2 "sudo pkill openvpn"
}



### Retrieve ###
pull_stderr () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
   #scp $1@$2
	scp $1@$2:~/Documents/aiders_osdk/std.err ./std_$1@$2.err
	#scp jetson@172.20.245.181:~/Documents/aiders_osdk/std.err ./
}

pull_database () {
	echo "Starting AutostartMultimasterOSDK.sh for user $1 at $2"
  	scp -r $1@$2:~/Documents/aiders_osdk/database ./database_$1@$2.err
}

clear_database() {
	echo "Clearing database folder for user $1 at $2"
  	ssh -o ConnectTimeout=5 -A $1@$2 "cd ~/Documents/aiders_osdk/; rm -r database"
}

### Main Command Loop ###
echo_action_prompt () {
	echo 	"\t1. Update Git Repos
	\t2. Start Drone OSDK
	\t3. Stop Drone OSDK
	\t4. Start VPN
	\t5. Stop VPN
	\t6. Retrieve Output
	\t7. Retrieve Database
	\t9. Exit\n"
}


echo_actions () {
	#echo "Actions you wish to perform: (default=exit)"
	read -p 'Action you wish to perform: ' as

	if ! [ -z "$as" ]; then
		action_set=$as
	else
		
		action_set=0
	fi

	echo "$action_set"
}

perform_action () {
	action_set=$1

	if [ -z "${action_set##*1*}" ]; then
  		echo -e "1. Update Git Repos\n"
		echo "Pulling for all connected computers"

		for i in "${!user_array[@]}"; do
			force_remote_pull ${user_array[i]} ${ip_array[i]}
		done
	fi
	
	### Start/Stop OSDK ###	
	if [ -z "${action_set##*2*}" ]; then
  		echo -e "2. Start Drone OSDK\n"
		echo "Starting OSDK remotely"
		
		for i in "${!user_array[@]}"; do
			start_remote ${user_array[i]} ${vpn_array[i]}
		done
	fi

	if [ -z "${action_set##*3*}" ]; then
  		echo -e "3. Start Drone OSDK\n"
		echo "Stopping OSDK remotely"
		
		for i in "${!user_array[@]}"; do
			start_remote ${user_array[i]} ${vpn_array[i]}
		done
	fi

	### Start/Stop VPN ###
	if [ -z "${action_set##*4*}" ]; then
  		echo -e "4. Start VPN\n"
		echo "Starting VPN"

		for i in "${!user_array[@]}"; do
			start_vpn ${user_array[i]} ${ip_array[i]}
		done
	fi

	if [ -z "${action_set##*5*}" ]; then
  		echo -e "5. Stop VPN\n"
		echo "Stopping VPN"
		
		for i in "${!user_array[@]}"; do
			start_remote ${user_array[i]} ${vpn_array[i]}
		done
	fi


	### Data Retrieval ###
	if [ -z "${action_set##*4*}" ]; then
  		echo -e "6. Retrieve Output\n"
		echo "Retrieving program output"

		for i in "${!user_array[@]}"; do
			start_vpn ${user_array[i]} ${ip_array[i]}
		done
	fi

	if [ -z "${action_set##*5*}" ]; then
  		echo -e "7. Retrieve Database\n"
		echo "Retrieving latest database instance"
		
		for i in "${!user_array[@]}"; do
			start_remote ${user_array[i]} ${vpn_array[i]}
		done
	fi

}










while [[ $a -ne 9 ]];
do
	echo_options
	#echo_action_prompt
	a=$(echo_actions)
	#echo $a
	echo -e "\n"

	perform_action $a

	echo -e "========"
done

echo "Goodbye..."


#ssh -A kios@172.20.236.40 "cd ~/Documents/aiders_osdk/; git pull http://cgeorg15:MzKpzCtXqTZwFD1JVzdw@kiosgitlab.koios.ucy.ac.cy/aiders/aiders_osdk"
#whoami; pwd; git pull

#ssh -A jetson@172.20.237.38 "cd ~/Documents/aiders_osdk/; git pull http://cgeorg15:MzKpzCtXqTZwFD1JVzdw@kiosgitlab.koios.ucy.ac.cy/aiders/aiders_osdk"
#ssh jetson@172.20.238.78 "cd ~/Documents/"

# MzKpzCtXqTZwFD1JVzdw
# git pull
#git pull http://cgeorg15:MzKpzCtXqTZwFD1JVzdw@kiosgitlab.koios.ucy.ac.cy/aiders/aiders_osdk

