#!/usr/bin/env bash
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH

#=================================================
#	  System Required: Ubuntu 22.04
#        Author: litian.zhuang 
#     email:litian.zhuang@nxrobo.com
#	  Site: http://www.nxrobo.com/
#   ROS技术讨论与反馈QQ群：6646169  8346256
#=================================================

GAME_ENABLE="yes"
sh_ver="2.0"
filepath=$(cd "$(dirname "$0")"; pwd)
Green_font_prefix="\033[32m" && Red_font_prefix="\033[31m" && Green_background_prefix="\033[42;37m" && Red_background_prefix="\033[41;37m" && Yellow_background_prefix="\033[43;37m" && Font_color_suffix="\033[0m" && Yellow_font_prefix="\e[1;33m" && Blue_font_prefix="\e[0;34m"
Info="${Green_font_prefix}[信息]${Font_color_suffix}"
Error="${Red_font_prefix}[错误]${Font_color_suffix}"
Warn="${Yellow_font_prefix}[警告]${Font_color_suffix}"
Tip="${Green_font_prefix}[注意]${Font_color_suffix}"
Separator_1="——————————————————————————————"
password="leo"
Version=$(lsb_release -r --short)
Codename=$(lsb_release -c --short)
OSDescription=$(lsb_release -d --short)
OSArch=$(uname -m)
calibra_default="${filepath}/../.ros/camera_info"


calibration="calibration"
color_block="color_block"

#检查系统要求
check_sys(){
        if [[ "${Version}" == "20.04" ]]; then
                ROS_Ver="foxy"
        elif [[ "${Version}" == "22.04" ]]; then
                ROS_Ver="humble"
        else
                echo -e "${Error} 当前系统 ${OSDescription} ,此版本暂未被适配!" && exit 1
        fi
}

#检查设备连接
check_dev(){
	#检查底盘
	if [ ! -n "$(lsusb -d f618:0600)" ]; then
		echo -e "${Error} 底盘没有正确连接，请确认正确连接！！"
	fi
	# #检查机械臂
	# if [ -n "$(lsusb -d 2341:0042)" ]; then
	# 	echo -e "${Info} 正在使用UARM机械臂"	
	# 	ARMTYPE="uarm"		
	# elif [ -n "$(lsusb -d 2e88:4603)" ]; then
	# 	echo -e "${Info} 正在使用射手座机械臂"
	# 	ARMTYPE="sagittarius_arm"	
	# else
	# 	echo -e "${Error} 机械臂没有正确连接或未上电，请确认正确连接！！"	
	# 	ARMTYPE="uarm"				
	# fi

	
	#检查摄像头
	check_camera
	#检查雷达
	check_lidar
}

#检查雷达设备
check_lidar(){
	BASEPATH=$(cd `dirname $0`; pwd)
	TYPE_LIDAR=$(cat /opt/lidar.txt)

	if [[ "${TYPE_LIDAR}" == "ydlidar_g6" ]]; then
		LIDARTYPE="ydlidar_g6"
		rm -f ${BASEPATH}/src/leo_driver/lidar/ydlidar_g6/CATKIN_IGNORE
	elif [[ "${TYPE_LIDAR}" == "ydlidar_g2" ]]; then
		LIDARTYPE="ydlidar_g2"
		touch ${BASEPATH}/src/leo_driver/lidar/ydlidar_g6/CATKIN_IGNORE
	elif [[ "${TYPE_LIDAR}" == "3iroboticslidar2" ]]; then
		LIDARTYPE="3iroboticslidar2"
		touch ${BASEPATH}/src/leo_driver/lidar/ydlidar_g6/CATKIN_IGNORE
	else
		echo "暂不支持的雷达：${TYPE_LIDAR}，使用默认的杉川雷达运行"
		LIDARTYPE="3iroboticslidar2"
		touch ${BASEPATH}/src/leo_driver/lidar/ydlidar_g6/CATKIN_IGNORE	
	fi
	lidar_flag=0

	#检查使用哪种设备
	if [ -n "$(lsusb -d 10c4:ea60)" ]; then
		lidar_flag=$[$lidar_flag + 1]
	fi

	if [ $lidar_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个雷达设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $lidar_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${LIDARTYPE}雷达"
	elif [ $lidar_flag -eq 0 ]; then
		echo -e "${Error} 没有找到雷达，请确认雷达已正确连接！！"
	fi	
}
#检查摄像头设备
check_camera(){

	camera_flag=0
	calibra_backup="/opt/nxrobo/camera_info"

	#检查使用哪种设备
	if [ -n "$(lsusb -d 2bc5:0403)" ]; then
		CAMERATYPE="astrapro"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 2bc5:0401)" ]; then
		CAMERATYPE="astra"

		camera_flag=$[$camera_flag + 1]
	fi
	if [ -n "$(lsusb -d 8086:0b07)" ]; then
		CAMERATYPE="d435"

		camera_flag=$[$camera_flag + 1]
	fi

	if [ $camera_flag -ge 2 ]; then
		echo -e "${Warn} 正在使用多个摄像头设备，请退出并拔掉其中一个再使用!"
		echo -e "${Warn} 退出请输入：Ctrl + c！"
	elif [ $camera_flag -eq 1 ]; then
		echo -e "${Info} 正在使用${CAMERATYPE}摄像头"
	elif [ $camera_flag -eq 0 ]; then
		echo -e "${Error} 没有找到摄像头，请确认摄像头已正确连接！！"
	fi	
	
	#根据摄像头类型检查标定文件和备份文件
	case $CAMERATYPE in
		astrapro) 
			#检查是否有默认文件夹和备份文件夹
			if [ ! -d "$calibra_default" ]; then
				mkdir -p "$calibra_default"
			fi

			if [ ! -d "$calibra_backup" ]; then
				echo -e "${Info} 创建标定文件备份文件夹..."
				sudo mkdir  -p "$calibra_backup"
			fi
			#如果没有就拷贝一份
			if [ ! -f "$calibra_backup/camera.yaml" ]||[ ! -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ -f "$calibra_default/camera.yaml" ]&&[ -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 备份标定文件..."
					sudo cp -r "$calibra_default/." "$calibra_backup/."
				elif [ ! -f "$calibra_default/camera.yaml" ]||[ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Warn} 标定文件缺少其中一个，请确认rgb和depth标定文件都存在"
				fi
			elif [ -f "$calibra_backup/camera.yaml" ]&&[ -f "$calibra_backup/depth_Astra_Orbbec.yaml" ]; then
				if [ ! -f "$calibra_default/camera.yaml" ]; then
					echo -e "${Info} 缺少彩色摄像头标定文件，已拷贝"
					cp "$calibra_backup/camera.yaml" $calibra_default
				fi
				if [ ! -f "$calibra_default/depth_Astra_Orbbec.yaml" ]; then
					echo -e "${Info} 缺少深度摄像头标定文件，已拷贝"
					cp "$calibra_backup/depth_Astra_Orbbec.yaml" $calibra_default	
				fi
			fi
			;;
		astra)
			#astra自带标定文件			
			#如果检查到有标定文件就删除
			if [ -d "$calibra_default" ]; then
				rm -rf "$calibra_default"
			fi
			;;
	esac

}

#安装ROS完整版
install_ros_full(){
		sudo sh -c 'echo 1 > /proc/sys/net/ipv6/conf/all/disable_ipv6'
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654		
		sudo apt-get update
		sudo apt-get install -y ros-${ROS_Ver}-desktop-full
		sudo rosdep init
		rosdep update
		echo "source /opt/ros/${ROS_Ver}/setup.bash" >> ~/.bashrc
		source /opt/ros/${ROS_Ver}/setup.bash
		sudo apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
}

#检测是否需要安装完整版
check_install_ros_full(){
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 检测到当前系统已安装了ROS的${ROSVER}版本!" 
			echo && stty erase ^? && read -p "请选择是否继续安装？ y/n：" choose
			if [[ "${choose}" == "y" ]]; then
				echo -e "${Info}准备安装ROS系统！" 
			else
				exit
			fi
		fi
	fi
	install_ros_full 
}



#安装intel_movidius的相关驱动和程序
install_intel_movidius(){
	BASEPATH=$(cd `dirname $0`; pwd)

	if [[ ! -d "$BASEPATH/src/3rd_app/intel/ncappzoo" ]] || [[ ! -d "$BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs" ]]  || [[ ! -d "/opt/movidius" ]] ; then
		echo && stty erase ^? && read -p "检测到未安装INTEL　MOVIDIUS的相关驱动和程序，是否现在安装y/n?" yorn 
		if [[ "${yorn}" == "y" ]]; then
			echo -e "${Info} 准备安装intel movidius的相关驱动和代码…… "
			echo -e "${Info} 安装过程中可能会花费挺长时间的。请耐心等待！"
			
			cd $BASEPATH
			mkdir $BASEPATH/src/3rd_app/intel
			cd $BASEPATH/src/3rd_app/intel
			echo -e "${Info} git clone https://github.com/movidius/ncsdk"
			git clone https://github.com/movidius/ncsdk
			echo -e "${Info} git clone https://github.com/movidius/ncappzoo"
			git clone https://github.com/movidius/ncappzoo
			echo -e "${Info} git clone https://github.com/intel/object_msgs"
			git clone https://github.com/intel/object_msgs
			echo -e "${Info} git clone https://github.com/intel/ros_intel_movidius_ncs.git"
			git clone https://github.com/intel/ros_intel_movidius_ncs.git
			cd $BASEPATH/src/3rd_app/intel/ncsdk
			make install
			make examples
			echo -e "${Info} sudo ln -s $BASEPATH/src/3rd_app/intel/ncappzoo /opt/movidius/ncappzoo"
			sudo ln -s $BASEPATH/src/3rd_app/intel/ncappzoo /opt/movidius/ncappzoo
			cd $BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs
			git checkout master
	
			cp $BASEPATH/src/3rd_app/intel/ros_intel_movidius_ncs/data/labels/* /opt/movidius/ncappzoo/data/ilsvrc12/

		#	AlexNet
			echo -e "${Info} compile NCS graph--AlexNet"
			cd /opt/movidius/ncappzoo/caffe/AlexNet
			make
		#	GoogleNet
			echo -e "${Info} compile NCS graph--GoogleNet"
			cd /opt/movidius/ncappzoo/caffe/GoogLeNet
			make
		#	SqueezeNet
			echo -e "${Info} compile NCS graph--SqueezeNet"
			cd /opt/movidius/ncappzoo/caffe/SqueezeNet
			make
		#	Inception_V1
			echo -e "${Info} compile NCS graph--Inception_V1"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v1/
			make
		#	Inception_V2
			echo -e "${Info} compile NCS graph--Inception_V2"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v2/
			make
		#	Inception_V3
			echo -e "${Info} compile NCS graph--Inception_V3"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v3/
			make
		#	Inception_V4
			echo -e "${Info} compile NCS graph--Inception_V4"
			cd /opt/movidius/ncappzoo/tensorflow/inception_v4/
			make
		#	MobileNet
			echo -e "${Info} compile NCS graph--MobileNet"
			cd /opt/movidius/ncappzoo/tensorflow/mobilenets/
			make

		#	MobileNet_SSD
			echo -e "${Info} compile NCS graph--MobileNet_SSD"
			cd /opt/movidius/ncappzoo/caffe/SSD_MobileNet
			make	
		#	TinyYolo
			echo -e "${Info} compile NCS graph--TinyYolo"
			cd /opt/movidius/ncappzoo/caffe/TinyYolo
			make

			echo -e "${Info} finish compiling..."	
			echo -e "${Info} start to catkin_make..."
	
			cd $BASEPATH
			catkin_make
			echo -e "${Info} finsh catkin_make..."
		else
			echo -e "${Info} 取消安装."
			exit
		fi
	fi

}

#编译leo
install_leo(){
	source /opt/ros/${ROS_Ver}/setup.bash
	colcon build
}




#远程设置
master_uri_setup(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	wlo1_ip=`/sbin/ifconfig wlo1|grep 'inet '|awk '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep 'inet '|awk '{print $2}'`
	if [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		local_ip=$eth_ip
	elif [ $wlo1_ip ]; then
		echo -e "${Info}使用无线网络wlo1" 
	  	local_ip=$wlo1
	elif [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	local_ip=$wlp1s_ip
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	local_ip=$wlp2s_ip
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	local_ip=$wlan_ip
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		local_ip=$enp3s_ip	
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用无线网络wlp0s20f3" 
		local_ip=$wlp0s_ip			
	fi
	export ROS_HOSTNAME=$local_ip
	export ROS_MASTER_URI="http://${local_ip}:11311"
	echo -e "${Info}Using ROS MASTER at ${Red_font_prefix}$ROS_MASTER_URI${Font_color_suffix} from ${Red_font_prefix}$ROS_HOSTNAME${Font_color_suffix}"
}

print_command()
{
	echo -e "${Yellow_background_prefix}${Red_font_prefix}${1}${Font_color_suffix}"
}

#让机器人动起来
let_robot_go(){
	echo -e "${Info}" 
	echo -e "${Info}      让机器人动起来" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}请选择遥控的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} 键盘控制
	  ${Green_font_prefix}2.${Font_color_suffix} 手柄控制
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-3]：" ctrlnum
	case "$ctrlnum" in
		2)
		echo -e "${Info}" 
		echo -e "${Info}手柄控制时，请按住‘X’键，然后通过左边的摇杆控制前后左右"
		echo -e "${Info}松开‘X’键后，轮子再释放扭力"
		echo -e "${Info}    退出请输入：Ctrl + c    " 
		echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

		print_command "ros2 launch leo_teleop leo_joy_teleop.launch.py  camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} "
		ros2 launch leo_teleop leo_joy_teleop.launch.py  camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} 
		;;
		1)
		echo -e "${Info}" 
		echo -e "${Info}    请在新的终端窗口操作"
		echo -e "${Info}键盘“wsad”分别对应“前后左右”"
		echo -e "${Info}                           " 
		echo -e "${Info}           w前进           "
		echo -e "${Info}    a左转         d右转    "
		echo -e "${Info}           s后退           " 
		echo -e "${Info}                           " 
		echo -e "${Info}    退出请输入：Ctrl + c    " 
		echo && stty erase ^? && read -p "按回车键（Enter）开始：" 


		print_command "ros2 launch leo_teleop leo_keyboard_teleop.launch.py  camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} "
		ros2 launch leo_teleop leo_keyboard_teleop.launch.py  camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} 
		;;
	esac

}

#让LEO跟着你走
people_follow(){
	echo -e "${Info}                  " 
	echo -e "${Info}让leo跟着你走" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}                  " 
	echo -e "${Info}请站在leo的正前方，与leo保持一米左右的距离，然后走动"
	echo -e "${Info}                  " 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch leo_follower leo_follower.launch.py camera_type_tel:=${CAMERATYPE}"
	ros2 launch leo_follower leo_follower.launch.py camera_type_tel:=${CAMERATYPE} 
}


#让leo使用激光雷达进行导航
leo_navigation_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让leo使用激光雷达进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}       A.激光雷达已上电连接"
	echo -e "${Info}       B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
	echo -e "${Info}       C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase '^?' && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch leo_navigation2 start_leo_navigation2.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}"
	ros2 launch leo_navigation2 start_leo_navigation2.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE}
}
#让leo使用深度摄像头进行导航
leo_navigation_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让leo使用深度摄像头进行导航" 
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash
	echo -e "${Info}请选择导航方式："
	echo -e "${Info}1.使用2D地图"
	echo -e "${Info}2.使用rtab_map地图"
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" slamnum

	echo -e "${Info}" 
	echo -e "${Info}请注意："
	echo -e "${Info}A.摄像头已连接"
	case "$slamnum" in
		1)
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
		2)
		SLAMTYPE="rtab_map"
		echo -e "${Info}B.把机器人放到原来建图的原点。导航正常启动后，如需查看原来建立的３Ｄ地图，点击rviz的Display->Rtabmap cloud->Download map加载３Ｄ地图。"
		;;
		*)
		echo -e "${Error} 错误，默认使用2D地图"
		SLAMTYPE="2d"
		echo -e "${Info}B.导航正常启动后，点击‘2D Pose Estimate’后在地图上进行手动定位。"
		;;
	esac

	echo -e "${Info}C.手动定位成功后，点击‘2D Nav Goal’后在地图上指定导航的目标点，机器人将进入自主导航。" 
	echo -e "${Info}退出请输入：Ctrl + c " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	if [[ "${SLAMTYPE}" == "2d" ]]; then
		print_command "roslaunch leo_navigation amcl_demo_rviz.launch camera_type_tel:=${CAMERATYPE}"
		roslaunch leo_navigation amcl_demo_rviz.launch camera_type_tel:=${CAMERATYPE} 
	else
		print_command "ros2 launch leo_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='true'"	
		ros2 launch leo_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='true'

	fi	
}

#让leo使用激光雷达绘制地图(gmapping)
leo_build_map_2d(){
	echo -e "${Info}" 
	echo -e "${Info}让leo使用激光雷达绘制地图" 
	echo -e "${Info}" 
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} gmapping
	  ${Green_font_prefix}2.${Font_color_suffix} cartographer
	  ${Green_font_prefix}3.${Font_color_suffix} slam_toolbox
	  ${Green_font_prefix}4.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-4]：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="gmapping"
		;;
		2)
		SLAMTYPE="cartographer"
		;;
		3)
		SLAMTYPE="slam_toolbox"
		;;
		*)
		echo -e "${Error} 错误，默认使用gmapping"
		SLAMTYPE="gmapping"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 
	print_command "ros2 launch leo_slam_transfer start_build_map_${SLAMTYPE}.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} enable_arm_tel:='false'"
	ros2 launch leo_slam_transfer start_build_map_${SLAMTYPE}.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} enable_arm_tel:='false'	
}

#让leo使用深度摄像头绘制地图
leo_build_map_3d(){
	echo -e "${Info}" 
	echo -e "${Info}让leo使用深度摄像头绘制地图" 
	echo -e "${Info}"
	echo -e "${Info}请选择SLAM的方式：
	  ${Green_font_prefix}1.${Font_color_suffix} rtab_map
	  ${Green_font_prefix}2.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 ：" slamnum
	case "$slamnum" in
		1)
		SLAMTYPE="rtab_map"
		;;
		*)
		echo -e "${Error} 错误，默认使用rtab_map"
		SLAMTYPE="rtab_map"
		;;
	esac
	PROJECTPATH=$(cd `dirname $0`; pwd)
	source ${PROJECTPATH}/install/setup.bash

	echo -e "${Info}" 
	echo -e "${Info}    请在新的终端窗口操作"
	echo -e "${Info}键盘“wsad”分别对应“前后左右”"
	echo -e "${Info}                           " 
	echo -e "${Info}           w前进           "
	echo -e "${Info}    a左转         d右转    "
	echo -e "${Info}           s后退           " 
	echo -e "${Info}                           " 
	echo -e "${Info}退出请输入：Ctrl + c        " 
	echo -e "${Info}" 
	echo && stty erase ^? && read -p "按回车键（Enter）开始：" 

	if [[ "${SLAMTYPE}" == "rtab_map" ]]; then
		echo -e "${Tip}" 
		echo -e "${Tip}现在使用rtab_map建图，将会删除之前保存的地图，选择‘y’继续建图，其它键直接退出。" 
		echo -e "${Tip}" 
		echo && stty erase ^? && read -p "请选择是否继续y/n：" choose
		if [[ "${choose}" == "y" ]]; then
			print_command "ros2 launch leo_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='false'"	
			ros2 launch leo_rtab_map start_rtabmap_rgbd_sync.launch.py camera_type_tel:=${CAMERATYPE} lidar_type_tel:=${LIDARTYPE} localization:='false'
		else
			return
		fi
        else
        	print_command "roslaunch leo_slam depth_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} camera_type_tel:=${CAMERATYPE}"
		roslaunch leo_slam depth_slam_teleop.launch slam_methods_tel:=${SLAMTYPE} camera_type_tel:=${CAMERATYPE}
	fi
	
}



qrcode_transfer_files(){
	eth_ip=`/sbin/ifconfig eth0|grep 'inet '|awk '{print $2}'`
	wlp1s_ip=`/sbin/ifconfig wlp1s0|grep 'inet '|awk '{print $2}'`
	wlp2s_ip=`/sbin/ifconfig wlp2s0|grep 'inet '|awk '{print $2}'`
	wlan_ip=`/sbin/ifconfig wlan0|grep 'inet '|awk '{print $2}'`
	enp3s_ip=`/sbin/ifconfig enp3s0|grep 'inet '|awk '{print $2}'`
	wlp0s_ip=`/sbin/ifconfig wlp0s20f3|grep 'inet '|awk '{print $2}'`
	wlo1_ip=`/sbin/ifconfig wlo1|grep 'inet '|awk '{print $2}'`
	if [ $wlp1s_ip ]; then
		echo -e "${Info}使用无线网络wlp1s0" 
	  	net_interface="wlp1s0"
	elif [ $wlo1_ip ]; then
		echo -e "${Info}使用无线网络wlo1" 
	  	net_interface="wlo1"	  	
	elif [ $wlp2s_ip ]; then
		echo -e "${Info}使用无线网络wlp2s0" 
	  	net_interface="wlp2s0"
	elif [ $wlan_ip ]; then
		echo -e "${Info}使用无线网络wlan0" 
	  	net_interface="wlan0"
	elif [ $enp3s_ip ]; then
		echo -e "${Info}使用无线网络enp3s0" 
		net_interface="enp3s0"
	elif [ $wlp0s_ip ]; then
		echo -e "${Info}使用无线网络wlp0s20f3" 
		net_interface="wlp0s20f3"		
	elif [ $eth_ip ]; then
		echo -e "${Info}使用有线网络eth0" 
		net_interface="eth0"
	fi
	echo -e "${Info}" 
	echo -e "${Info}通过局域网收发文件" 
	echo -e "${Info}" 
	echo -e "${Info}请选择：
	  ${Green_font_prefix}1.${Font_color_suffix} 发送文件（文件名，带上文件绝对路径）
	  ${Green_font_prefix}2.${Font_color_suffix} 接收文件（默认存放在~/Downloads路径中）
	  ${Green_font_prefix}3.${Font_color_suffix} 退出请输入：Ctrl + c" 
	echo && stty erase ^? && read -p "请输入数字 [1-2]：" cnum
	case "$cnum" in
		1)
		echo -e "${Info}请输入文件名，带上文件绝对路径，如 /home/${USER}/a.jpg：
		 退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入要发送的文件：" s_file
		if [ -f "$s_file" ]; then
			echo -e "${Info}本机即将发送文件：${Green_font_prefix}"$s_file"${Font_color_suffix}，请接收端扫码或者直接输入下面的网址接收文件"
		else 
			echo -e "${Info}请输入带绝对路径的文件名"
			exit
		fi
		
		qrcp send  -i $net_interface $s_file
		;;
		2)
		echo -e "${Info}请输入接收到的文件存放的路径，默认为 /home/${USER}/Downloads：
		退出请输入：Ctrl + c" 
		echo && stty erase ^? && read -p "请输入文件存放的文件夹路径：" s_file
		if [ -d "$s_file" ]; then
			echo ""
		else 
			echo -e "${Info}${Red_font_prefix}文件夹不存在，将存放在默认文件夹/home/${USER}/Downloads中${Font_color_suffix}"
			s_file="/home/${USER}/Downloads"
		fi
		echo -e "${Info}接收的文件将存放在：${Green_font_prefix}"$s_file"${Font_color_suffix}，目录下，请发送端扫码或者直接输入下面的网址选择文件发送"
		qrcp  -i $net_interface receive --output=$s_file
		;;
		*)
		echo -e "${Error} 错误，退出"
		;;
	esac
}

coming_soon(){
	echo -e "${Tip} coming_soon!" 
}


#printf
menu_status(){
	echo -e "${Tip} 当前系统版本 ${OSDescription} !" 
	if [ -f "/usr/bin/rosversion" ]; then
		ROSVER=`/usr/bin/rosversion -d`
		if [ $ROSVER ]; then
			echo -e "${Tip} 当前ROS版本 ${ROSVER} !"
			return
		fi 
	fi
	echo -e "${Error} 未检测到ROS版本，请先安装ROS！可以选择102直接安装。" 
}

tell_us(){
	echo -e ""
	echo -e "${Tip} --------------分隔线----------------" 
	echo -e "${Tip} 网址：www.nxrobo.com" 
	echo -e "${Tip} leo技术讨论与反馈QQ群：一群 8346256(已满)；二群 6646169" 
	echo -e "${Tip} ---------QQ扫描加入我们--------------"
	echo 'https://jq.qq.com/?_wv=1027&k=1JV8oyB8'|qrencode -o - -t UTF8
	echo -e ""
}

qrcode_picture()
{
	echo 'www.NXROBO.com'|qrencode -o - -t UTF8
}
check_sys
echo -e "————————————
  leo 一键安装管理脚本 ${Red_font_prefix}[v${sh_ver}]${Font_color_suffix}
"
qrcode_picture

echo -e "  
  请根据右侧的功能说明选择相应的序号。
  
  ${Green_font_prefix}  0.${Font_color_suffix} 单独编译LEO
————————————
  ${Green_font_prefix}  1.${Font_color_suffix} 让机器人动起来
  ${Green_font_prefix}  2.${Font_color_suffix} 让LEO跟着你走
  ${Green_font_prefix}  3.${Font_color_suffix} 让LEO使用激光雷达绘制地图
  ${Green_font_prefix}  4.${Font_color_suffix} 让LEO使用深度摄像头绘制地图
  ${Green_font_prefix}  5.${Font_color_suffix} 让LEO使用激光雷达进行导航
  ${Green_font_prefix}  6.${Font_color_suffix} 让LEO使用深度摄像头进行导航
  
————————————

  ${Green_font_prefix}100.${Font_color_suffix} 问题反馈
  ${Green_font_prefix}104.${Font_color_suffix} 文件传输
 "
menu_status
check_dev
echo && stty erase ^? && read -p "请输入数字：" num
case "$num" in
	0)
	install_leo	
	;;
	1)
	let_robot_go
	;;
	2)
	people_follow
	;;
	3)
	leo_build_map_2d
	;;
	4)
	leo_build_map_3d
	;;
	5)
	leo_navigation_2d
	;;
	6)
	leo_navigation_3d
	;;
	100)
	tell_us
	;;
	104)
	qrcode_transfer_files
	;;	
	*)
	echo -e "${Error} 请输入正确的数字 "
	;;
esac
