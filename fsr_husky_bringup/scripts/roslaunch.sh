#!/bin/bash

source /opt/ros/hydro/setup.bash
source /home/${USER}/fsr_husky_workspace/devel/setup.bash

echo "Launching file $1.launch"
roslaunch fsr_husky_bringup $1.launch
