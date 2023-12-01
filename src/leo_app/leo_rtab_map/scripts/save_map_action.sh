#!/bin/bash
#!SPARK技术讨论与反馈群：8346256
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Font_color_suffix="\033[0m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
echo -e "${Info} 地图将会被保存到工作空间下的 install/leo_navigation2/share/leo_navigation2/map/目录下"
echo -e "${Info} 是否开始保存当前的地图？"
echo -e "${Info} 确定保存请按任意键，退出请输入：Ctrl + c    " 
echo && stty erase '^H' && read -p "按任意键开始：" 
PROJECTPATH=$(cd `dirname $0`; pwd)
gnome-terminal --title="save_map_action"  -- bash -c "ros2 run nav2_map_server map_saver_cli -f ${PROJECTPATH}/../../../leo_navigation2/share/leo_navigation2/map/map --ros-args -p save_map_timeout:=100000"
