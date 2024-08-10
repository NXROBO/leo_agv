# 本Python示例默认使用仿真机械臂

1. 启动python api示例
```
ros2 launch motion_planning_python_tutorial motion_planning_python_api_tutorial.launch.py
```
2. 启动planning_scene示例
```
ros2 launch motion_planning_python_tutorial motion_planning_python_api_tutorial.launch.py example_file:=motion_planning_python_api_planning_scene.py
```
# 如果连接真实机械可参考如下launch模板文件

**warnning:确保程序下发的目标点真实机械臂可到达且运动过程不会和物理环境其他设备产生碰撞**

启动示例aubo_C3真实机械臂IP：`192.168.1.120`
```shell
ros2 launch motion_planning_python_tutorial motion_planning_python_template.launch.py robot_ip:=192.168.1.120 aubo_type:=aubo_C3 use_fake_hardware:=False
```




