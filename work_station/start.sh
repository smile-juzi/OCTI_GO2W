#! /bin/bash

# robot motion
cd /home/dog/Desktop/work_station/OctiRobotVersion2/build
gnome-terminal -- bash -c "/home/dog/Desktop/work_station/OctiRobotVersion2/build/main; exec bash"

# speaker interactor
cd /home/dog/Desktop/work_station/yy
gnome-terminal -- bash -c "/home/dog/Desktop/work_station/yy/uart; exec bash"

# 
cd /home/dog/Desktop/work_station/http
gnome-terminal -- bash -c "/home/dog/Desktop/work_station/http/server; exec bash"

# vision -> face / helmet / smoke / iray dection
cd /home/dog/Desktop/work_station/vision
gnome-terminal -- bash -c "source /home/dog/Downloads/enter/etc/profile.d/conda.sh; conda activate rknn_vision; python /home/dog/Desktop/work_station/vision/main.py; exec bash"

# meter
cd /home/dog/Desktop/work_station/meter
gnome-terminal -- bash -c "source /home/dog/Downloads/enter/etc/profile.d/conda.sh; conda activate rknn_vision; python /home/dog/Desktop/work_station/meter/main.py; exec bash"

cd /home/dog/Desktop/work_station/yw
gnome-terminal -- bash -c "/home/dog/Desktop/work_station/yw/yw; exec bash"

cd /home/dog/Desktop/work_station/iray_pro/Xtherm_LinuxSDK_V6.3
export LD_LIBRARY_PATH=./
./main1
