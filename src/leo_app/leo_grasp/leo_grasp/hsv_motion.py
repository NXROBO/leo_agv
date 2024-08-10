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

        # 抓取姿态
        self.ready_values = {
            "foreArm_joint": -0.9656377251059474,"wrist1_joint": 0.26844829995519176,"upperArm_joint": 0.33847095519177534,"wrist2_joint": -1.5632255169254201,"wrist3_joint": -0.30126247877899875,"shoulder_joint": 1.2959143056618647,
        }

        self.posture = {
            "ready": self.ready_values,
        }

        # 记录日志：MoveItPy 实例已创建
        logger.info("MoveItPy instance created")

        # 创建一个 PoseStamped 消息对象
        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "aubo_base"
    
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


class AutoAction(Node):
    def __init__(self):

        super().__init__('AutoAction')

        self.moveit = RobotMoveitAction()

        self.transform_stamped = None
        # 创建日志记录器
        logger = get_logger("moveit_py.pose_goal")

        self.moveit.determine_pose("ready") # 移动机械臂到检测区
    
def main():
    rclpy.init()
    node = AutoAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()