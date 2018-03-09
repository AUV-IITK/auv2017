#!/bin/sh

echo "Enter vendor id of the camera"
read vendorId

echo "Enter product id of the camera"
read productId

echo "ACTION==\"add\", KERNEL==\"video0\", SUBSYSTEM==\"video4linux\", SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"$vendorId\", ATTRS{idProduct}==\"$productId\", SYMLINK+=\"front_cam\"" >> /etc/udev/rules.d/83-webcam.rules
echo "ACTION==\"add\", KERNEL==\"video1\", SUBSYSTEM==\"video4linux\", SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"$vendorId\", ATTRS{idProduct}==\"$productId\", SYMLINK+=\"bottom_cam\"" >> /etc/udev/rules.d/83-webcam.rules
