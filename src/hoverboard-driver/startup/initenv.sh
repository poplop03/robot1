#!/bin/bash

echo "Creating udev rule for Hoverboard UART..."

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5523", MODE:="0666", GROUP:="dialout", SYMLINK+="hoverboard_uart"' > /etc/udev/rules.d/hoverboard.rules

# Reload udev
echo "Reloading udev rules..."
service udev reload
sleep 2
service udev restart

echo "Done! You can now use /dev/hoverboard_uart."
