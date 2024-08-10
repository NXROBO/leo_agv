#!/bin/bash


gnome-terminal --title="leo_control" --geometry 34x10+63+305 -- bash -c "ros2 run leo_teleop leo_teleop_node 0.24 0.5"

