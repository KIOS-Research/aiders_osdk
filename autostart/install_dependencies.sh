#!/bin/sh


sudo apt update
sudo apt upgrade
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy

python3 -m pip install --upgrade numpy scipy scikit-learn

#python3 -m pip install --upgrade numba
sudo apt-get install python3-numba

python3 -m pip install --upgrade execnet
