#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH
PROJECTPATH=/home/nxrobo/leo_agv
source ${PROJECTPATH}/install/setup.bash
echo "ros2 launch leo_teleop leo_joy_teleop.launch.py"
ros2 launch leo_teleop leo_joy_teleop.launch.py 
