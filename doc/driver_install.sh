#!/bin/bash

Green_font_prefix="\033[32m" && Font_color_suffix="\033[0m"

device_check(){
echo -e "
请选择机械臂型号:
${Green_font_prefix}  1.${Font_color_suffix} aubo_C3
${Green_font_prefix}  2.${Font_color_suffix} aubo_ES3
${Green_font_prefix}  3.${Font_color_suffix} None
"
  read -p "请输入数字:" yon
  case "$yon" in
  1)
  device="aubo_C3"
  ;;
  2)
  device="aubo_ES3"
  ;;
  3)
  device="None"
  ;;
  esac

  if [[ "${yon}" != "3" ]]; then
    read -p "请输入机械臂IP地址:" device_ip
    DEVICE_IP=$device_ip
  else
    DEVICE_IP="127.0.0.1"
  fi

  read -p "请输入安装的深度摄像头数量:" val
  camera=$val
  echo -e "安装完成。"
}

echo -e "根据提示完成LEO驱动安装
请选择LEO底盘类型：
${Green_font_prefix}  1.${Font_color_suffix} 差速底盘
${Green_font_prefix}  2.${Font_color_suffix} 麦克纳姆轮底盘
"
read -p "请输入数字:" num
case "$num" in
	1)
  driver="diff"
  device_check
	;;
	2)
	driver="omni"
	device_check
	;;
esac

echo $driver > /opt/leo_driver.txt
echo $device >> /opt/leo_driver.txt
echo $camera >> /opt/leo_driver.txt
echo $DEVICE_IP >> /opt/leo_driver.txt
