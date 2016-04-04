#!/bin/bash
#this file gets all the requirements for running avcnn
cd ~
sudo pip3 install pynmea2 dill
mkdir piblaster && cd piblaster
wget https://github.com/sarfarta/pi-blaster/archive/master.zip
unzip master.zip
rm master.zip
cd pi-blaster-master
./autogen.sh
./configure && make
sudo make install

cd ~
sudo apt-get install python3-scipy
sudo pip3 install Theano
sudo pip3 install keras
sudo apt-get install libhdf5-dev
sudo HDF5_DIR=/usr/lib/arm-linux-gnueabihf/hdf5/serial CC=hdcc pip3 install h5py

cd ~
mkdir avc && cd avc
wget https://github.com/TechBotBuilder/avcnn/archive/master.zip
unzip raspi_code.zip
rm raspi_code.zip
