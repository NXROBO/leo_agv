#!/bin/bash

# gnome-terminal --title="window" --geometry 34x10+63+305 -- bash -c "python3 ./discam.py" 

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gnome-terminal --title="window" --geometry 34x10+63+305 -- bash -c "cd $SCRIPT_DIR && python3 ./discam.py"


file_path="/opt/camera.txt"
if [ ! -f "$file_path" ]; then
  touch "$file_path"
else
  :
fi


BUFF=$(sed -n '1P' /opt/camera.txt)
servo1=$(sed -n '2P' /opt/camera.txt)
servo2=$(sed -n '3P' /opt/camera.txt)

serial_numbers=($(rs-enumerate-devices | grep 'Serial Number' | grep -v 'Asic' | awk -F': ' '{print $2}'))

# 判断相机序列号是否为空
if [ -n "$servo1" ] || [ -n "$servo2" ]; then
  buff="False"
  SERIAL1=${serial_numbers[0]}
  SERIAL2=${serial_numbers[1]} 
  
  echo $buff > /opt/camera.txt
  echo $SERIAL1 >> /opt/camera.txt
  echo $SERIAL2 >> /opt/camera.txt
else
  buff="False"
  SERIAL1=${serial_numbers[0]}
  SERIAL2=${serial_numbers[1]} 
  
  echo $buff > /opt/camera.txt
  echo $SERIAL1 >> /opt/camera.txt
  echo $SERIAL2 >> /opt/camera.txt
fi

servo1=$(sed -n '2P' /opt/camera.txt)
servo2=$(sed -n '3P' /opt/camera.txt)

if [ ${#serial_numbers[@]} -eq 2 ]; then
    # echo "$servo1"
    # echo "$servo2"
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
      
      echo $buff > /opt/camera.txt
      echo $SERIAL1 >> /opt/camera.txt
      echo $SERIAL2 >> /opt/camera.txt
      
  else
      echo "退出程序..."
  fi

else
  read -p "当前为单相机模式，请输入[n]: " input
  echo "退出程序..."
fi


pid=$(ps aux | grep discam.py | grep -v grep | awk '{print $2}')
kill -SIGINT $pid


