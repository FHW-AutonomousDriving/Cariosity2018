#!/bin/sh
IP=$(ip route get 8.8.8.8 | awk '{print $NF; exit}')
rm -f /tmp/*.ppm
/opt/ADTF/3.3.3/bin/adtf_launcher -session=/home/aadc/AADC/config/LiveVisualization/adtfsessions/camera_tweaking.adtfsession --run