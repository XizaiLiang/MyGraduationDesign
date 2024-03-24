#!/bin/bash
scriptPath=/home/unitree/Unitree/autostart/
export SUDO_ASKPASS=${scriptPath}passwd.sh
localIP=$(cat /etc/network/interfaces | grep "address" | awk '{print $2}')
if [ "${localIP}" == "192.168.123.13" ]; then
./faceLightServer &
fi
