# Copyright (c) 2024 NXROBO
#
# /* Author: haijie.huo */
# /* email: haijie.huo@nxrobo.com */
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import math
import _thread
import threading
import os
import pickle
import asyncio

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_ros
import tf_transformations
from rclpy.time import Time

import time
from rclpy.duration import Duration

from turtlesim.srv import Spawn
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from aubo_msgs.srv import AddSignal, SetOutputSignal
from scipy.spatial.transform import Rotation

from moveit.core.kinematic_constraints import construct_joint_constraint

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

# 机械臂运动规划
class RobotMoveitAction(Node):
    def __init__(self):
        super().__init__('RobotMoveitAction')

        # 创建日志记录器
        logger = get_logger("moveit_py.pose_goal")

        # 创建 MoveItPy 实例
        self.aubo = MoveItPy()

        # 获取 Aubo 机械臂的运动规划组件
        self.aubo_arm = self.aubo.get_planning_component("aubo_arm")

        self.robot_model = self.aubo.get_robot_model()
        self.robot_state = RobotState(self.robot_model)

            # 初始姿态
        self.first_values = {
            "foreArm_joint": -2.4108334358394976,"wrist1_joint": 0.19090251921361948,"upperArm_joint": -1.034534295735007,"wrist2_joint": -1.5654776861404676,"wrist3_joint": -0.10093312750890894,"shoulder_joint": 1.5002197993246449,
        }

        # 抓取姿态
        self.ready_values = {
            "foreArm_joint": -0.9656377251059474,"wrist1_joint": 0.26844829995519176,"upperArm_joint": 0.33847095519177534,"wrist2_joint": -1.5632255169254201,"wrist3_joint": -0.30126247877899875,"shoulder_joint": 1.2959143056618647,
        }
        self.ready2_values = {
            "foreArm_joint": -0.2885711017691204,"wrist1_joint": 0.7373690042739373,"upperArm_joint": 0.5566415948364458,"wrist2_joint": -1.5620737561216333,"wrist3_joint": -0.3733832460530537,"shoulder_joint": 1.2129104992004374,
        }
        
        # 抓取后（前）姿态
        self.catch_values = {
            "foreArm_joint": -1.9574101499793466,"wrist1_joint": 0.2739686821516851,"upperArm_joint": -0.6628273384951079,"wrist2_joint": -1.55778949937379,"wrist3_joint": -0.05559996899298767,"shoulder_joint": 1.5361444659242836,
        }

        # 放置物体姿态
        self.place_values = {
            "foreArm_joint": -0.965094856956392,"wrist1_joint": -0.35666070622992196,"upperArm_joint": 0.9411279617591244,"wrist2_joint": -1.624441236843877,"wrist3_joint": -0.18545916560591436,"shoulder_joint": 1.3522111999819844,
        }
        self.place2_values = {
            "foreArm_joint": -0.9197213501320577,"wrist1_joint": -0.045931047086039824,"upperArm_joint": 0.7063044748240355,"wrist2_joint": -1.5742185969539186,"wrist3_joint": -0.20890886884988916,"shoulder_joint": 1.3430447979161793,
        }

        self.place_left = {
            "foreArm_joint": -0.9526895861334413,"wrist1_joint": -0.04406035278689589,"upperArm_joint": 0.6441424036718938,
            "wrist2_joint": -1.5667394877853806,"wrist3_joint": -0.0603353931894481,"shoulder_joint": 3.1180952659813306,
        }

        self.posture = {
            "zero": self.first_values,
            "ready": self.ready_values,
            "ready2": self.ready2_values,
            "catch": self.catch_values,
            "place": self.place_values,
            "place2": self.place2_values,
            "place_left": self.place_left,
        }

        # 记录日志：MoveItPy 实例已创建
        logger.info("MoveItPy instance created")


        # 创建一个 PoseStamped 消息对象
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "aubo_base"

        self.signal_add = self.create_client(AddSignal, "/io_and_status_controller/modbus_add_signal")
        self.signal_output = self.create_client(SetOutputSignal, "/io_and_status_controller/modbus_set_output_signal")


        if self.signal_add.wait_for_service(timeout_sec=3.0):
            if self.signal_output.wait_for_service(timeout_sec=3.0):
                self.get_logger().info("夹爪服务已连接！")
        else:
            self.get_logger().info("未发现夹爪服务...")
            
        # # 连接夹爪
        self.add = AddSignal.Request()
        self.output = SetOutputSignal.Request()


    # Modbus信号注册
    def add_signal(self):
        # 从站编号
        self.add.slave_number = 1  
        # 信号地址
        self.add.signal_address = 10  
        # 信号类型
        self.add.signal_type = 3  
        # 信号名称
        self.add.signal_name_index = 0  
        # 数据发送
        self.signal_add.call_async(self.add)

    # Modbus信号输出
    def output_signal(self, value):
        # 信号名称（要与注册时的信号名称一致）
        self.output.signal_name_index = 0  
        # 夹爪控制数值（0-1000）
        self.output.value = value  
        # 数据发送
        self.signal_output.call_async(self.output)


    # 机械臂运动规划与执行
    def plan_and_execute(self,robot,planning_component,logger,
                         single_plan_parameters=None,
                         multi_plan_parameters=None,sleep_time=0.0,):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            # 如果有多个规划参数，则进行多点规划
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            # 如果有单个规划参数，则进行单点规划
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            # 否则，进行默认规划
            plan_result = planning_component.plan()

        # 执行规划结果
        if plan_result:
            # 如果规划成功，则执行规划
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory
            robot.execute(robot_trajectory, controllers=[])
            return True
        else:
            # 如果规划失败，则记录错误日志
            logger.error("Planning failed")
            return False

        time.sleep(sleep_time)
    

    def determine_pose(self,pose):
        logger = get_logger("moveit_py.pose_goal")

        # 根据传入的 pose 参数进行判断并设置不同的预设位姿值
        if pose != None:

            self.aubo_arm.set_start_state_to_current_state()

            self.robot_state.joint_positions = self.posture[pose]
            joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.aubo.get_robot_model().get_joint_model_group("aubo_arm"),
            )
            self.aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
            
            # plan to goal
            self.plan_and_execute(self.aubo, self.aubo_arm, logger, sleep_time=3.0)

        else:
            raise ValueError("Invalid pose parameter. Please choose from 'predict_pose', 'catch_pose_front', 'catch_pose_back', or 'place_pose'.")

        return self.pose_goal

                
    # 抓取物体
    def pitch_object(self, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w):

        logger = get_logger("AutoAction.pose_object")

        for i in range(3):
            self.pose_goal.pose.orientation.x = rot_x
            self.pose_goal.pose.orientation.y = rot_y
            self.pose_goal.pose.orientation.z = rot_z
            self.pose_goal.pose.orientation.w = rot_w
            self.pose_goal.pose.position.x = trans_y
            self.pose_goal.pose.position.y = trans_x
            self.pose_goal.pose.position.z = trans_z
            self.aubo_arm.set_goal_state(pose_stamped_msg=self.pose_goal, pose_link="wrist3_Link")
            self.aubo_arm.set_start_state_to_current_state()
            ret = self.plan_and_execute(self.aubo, self.aubo_arm, logger, sleep_time=3.0)
            if ret != False:
                logger.info("--------------------------------规划成功，机械臂移动----------------------------------------")
                break
            else:
                logger.info("----------------------------------机械臂运动规划失败----------------------------------------")
                return False
        return ret


class AutoAction(Node):
    def __init__(self):

        super().__init__('AutoAction')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.table_h = float(self.declare_parameter("table_h").get_parameter_value().string_value)
        self.get_logger().info(f'table_h: {self.table_h}')
   
        self.key = None

        self.moveit = RobotMoveitAction()

        self.transform_stamped = None
        # 创建日志记录器
        logger = get_logger("moveit_py.pose_goal")
        logger.info("创建物体TF监听")
        logger.info("========  程序初始化成功 ======= ")

        self.moveit.add_signal() # 夹爪使能
        self.moveit.output_signal(1000) # 机械臂夹爪释放

        # 创建物体tf监听
        self.grasp_timer = self.create_timer(0.1, self.get_trans)

        # 创建主程序节点
        self.task_timer = self.create_timer(0.1, self.run_task)

        self.task_run_th = threading.Thread(target=lambda: "pass") # 创建线程对象

    def get_trans(self):
        # 获取物体坐标转换
        try:
            # print("================获取坐标转换中=================")
            if self.tf_buffer.can_transform("base_link", "object", Time()):
                self.transform_stamped = self.tf_buffer.lookup_transform("base_link", "object", Time())
                # self.get_logger().info("======================================")
                
                # print("坐标变换消息:", self.transform_stamped)
                return True, self.transform_stamped
            else:
                # self.get_logger().info("base_link to object转换失败！！！")
                return False, None
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to object: {ex}')
            return False, None

    def delay(self, duration):
        # 非阻塞延迟函数
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('----------over-----------')

    def motion(self):
        logger = get_logger("AutoAction.pose_goal")
            
        buff, tf_trans = self.get_trans()
        if buff != False:
            trans_x = tf_trans.transform.translation.x - 0.165
            trans_y = tf_trans.transform.translation.y - 0.005  # + 0.025
            # trans_z = tf_trans.transform.translation.z - 0.065
            # trans_z = 0.167411
            trans_z = (self.table_h+0.053)/1.84211
            rot_x = tf_trans.transform.rotation.x
            rot_y = tf_trans.transform.rotation.y
            rot_z = tf_trans.transform.rotation.z
            rot_w = tf_trans.transform.rotation.w

            logger.info("\n物体坐标变换消息:\nTranslation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\nRotation:\n  x:%.6f\n  y:%.6f\n  z:%.6f\n  w:%.6f\n" \
                                    % (trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w))
            
            logger.info("========  开始抓取 ======= ")        
            self.moveit.pitch_object(-trans_x, trans_y, trans_z + 0.15, rot_x, rot_y, rot_z, rot_w)

            time.sleep(1)
            self.moveit.pitch_object(-trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w)

            self.moveit.output_signal(530)     # 机械臂夹爪抓取 550
            time.sleep(1)
            self.moveit.pitch_object(-trans_x, trans_y, trans_z + 0.15, rot_x, rot_y, rot_z, rot_w)

            logger.info("========  物体放置中 ======= ")

            time.sleep(1)
            self.moveit.determine_pose("catch")  
            time.sleep(1)
            pose = "place_left"
            self.moveit.determine_pose(pose)  # 移动机械臂到提取区
            self.moveit.output_signal(1000)  # 机械臂夹爪释放

            time.sleep(1)
            self.moveit.determine_pose("catch")  
            
            logger.info("========  物体抓取结束  ===== ")
            time.sleep(1)

            self.moveit.determine_pose("ready") # 移动机械臂到检测区


    def run_task(self):
        logger = get_logger("AutoAction.pose_goal")
        self.moveit.output_signal(1000) # 机械臂夹爪释放
        self.moveit.determine_pose("ready") # 移动机械臂到检测区
        while True:
            logger.info("========  视觉识别中 ======= ")
            self.delay(4)
            self.motion()

    
def main():
    rclpy.init()
    node = AutoAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
