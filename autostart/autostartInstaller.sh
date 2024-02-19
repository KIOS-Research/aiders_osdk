#!/bin/sh
#author: 	christos georgiades
#date:		19/12/2022

if [ $(id -u) != 0 ]; then
	echo "This script requires superuser privilages."
	sudo "$0" "$@"
	exit $?
fi

directoryOriginal=$PWD
directoryOwner=$(echo $PWD | awk -F '/' '{print $3}')
echo "Directory User: $directoryOwner"

if [ ! -d ./jetson_hotspot_service ] ; then
	git clone https://kiostools2.ucy.ac.cy/cgeorg15/jetson_hotspot_service
else
	cd jetson_hotspot_service
	git pull https://kiostools2.ucy.ac.cy/cgeorg15/jetson_hotspot_service
	cd $directoryOriginal
fi

sudo chown -R $directoryOwner jetson_hotspot_service

cd jetson_hotspot_service
chmod +x hotspotServiceInstaller.sh
./hotspotServiceInstaller.sh
cd $directoryOriginal


sed -i "s@WorkingDirectory=.*@WorkingDirectory=$PWD@" aiders_osdk.service 

cp aiders_osdk.service  /etc/systemd/system/

systemctl daemon-reload

printf "\naiders_osdk.service installed.\n\nUse\n > systemctl enable aiders_osdk.service \nto activate service.\n"

