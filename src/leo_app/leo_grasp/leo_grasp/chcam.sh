#!/bin/bash

gnome-terminal --title="window" --geometry 34x10+63+305 -- bash -c "python3 ./discam.py" 


file_path="../../../../doc/camera.txt"
if [ ! -f "$file_path" ]; then
  touch "$file_path"
else
  :
fi


BUFF=$(sed -n '1P' ../../../../doc/camera.txt)
servo1=$(sed -n '2P' ../../../../doc/camera.txt)
servo2=$(sed -n '3P' ../../../../doc/camera.txt)

serial_numbers=($(rs-enumerate-devices | grep 'Serial Number' | grep -v 'Asic' | awk -F': ' '{print $2}'))

# 判断相机序列号是否为空
if [ -n "$servo1" ]; then
  :
else
  buff="False"
  SERIAL1=${serial_numbers[0]}
  SERIAL2=${serial_numbers[1]} 
  
  echo $buff > ../../../../doc/camera.txt
  echo $SERIAL1 >> ../../../../doc/camera.txt
  echo $SERIAL2 >> ../../../../doc/camera.txt
fi


read -p "是否切换上下相机图像[y/n]: " input

if [ "$input" = "y" ]; then
    echo "切换成功..."
    if [ "$BUFF" = "False" ]; then
      buff="True"
    else
      buff="False"
    fi
    
    SERIAL1=${servo2}
    SERIAL2=${servo1}
    
    echo $buff > ../../../../doc/camera.txt
    echo $SERIAL1 >> ../../../../doc/camera.txt
    echo $SERIAL2 >> ../../../../doc/camera.txt
    
else
    echo "退出程序..."
fi

pid=$(ps aux | grep discam.py | grep -v grep | awk '{print $2}')
kill -SIGINT $pid


