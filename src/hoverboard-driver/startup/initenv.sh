#!/bin/bash
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea06", MODE:="0666", GROUP:="dialout", SYMLINK+="hoverboard_uart"

service udev reload
sleep 2
service udev restart

