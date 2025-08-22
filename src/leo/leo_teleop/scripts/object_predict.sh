#!/bin/bash


gnome-terminal --title="leo_yolo" --geometry 34x10+63+305 -- bash -c "ros2 run leo_yolov8 camera_predict_tf"

