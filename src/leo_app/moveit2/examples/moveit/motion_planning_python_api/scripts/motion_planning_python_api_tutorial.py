#!/usr/bin/env python3
"""
A script to outline the fundamentals of the moveit_py motion planning API.
"""

import time

# generic ros libraries
from ament_index_python import get_package_share_directory
from moveit_configs_utils.moveit_configs_builder import MoveItConfigsBuilder
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("成功规划")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("规划失败")

    time.sleep(sleep_time)


def main():

    ###################################################################
    # MoveItPy Setup
    ###################################################################
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    # instantiate MoveItPy instance and get planning component
    aubo = MoveItPy(node_name="moveit_py")
    aubo_arm = aubo.get_planning_component("aubo_arm")
    logger.warn("MoveItPy instance created")
    time.sleep(3)

    ###########################################################################
    # Plan 1 - set states with predefined string
    ###########################################################################

    # # set plan start state using predefined state
    # aubo_arm.set_start_state(configuration_name="zero")
    # aubo_arm.set_start_state_to_current_state()
    # # set pose goal using predefined state
    # aubo_arm.set_goal_state(configuration_name="home")

    # # plan to goal
    # plan_and_execute(aubo, aubo_arm, logger, sleep_time=3.0)

    # ###########################################################################
    # # Plan 2 - set goal state with RobotState object
    # ###########################################################################

    # # instantiate a RobotState instance using the current robot model
    # robot_model = aubo.get_robot_model()
    # robot_state = RobotState(robot_model)

    # # randomize the robot state
    # robot_state.set_to_random_positions()

    # # set plan start state to current state
    # aubo_arm.set_start_state_to_current_state()

    # # set goal state to the initialized robot state
    # logger.info("Set goal state to the initialized robot state")
    # aubo_arm.set_goal_state(robot_state=robot_state)

    # # plan to goal
    # plan_and_execute(aubo, aubo_arm, logger, sleep_time=3.0)

    # ###########################################################################
    # # Plan 3 - set goal state with PoseStamped message
    # ###########################################################################

    # set plan start state to current state
    aubo_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped,Quaternion
    #         /*

    # 看物品: Translation: [0.655, -0.003, 0.677]
    #              Quaternion [0.711, 0.703, 0.011, 0.016]

    # 运货-前: Translation: [0.341, 0.109, 0.592]
    #               Quaternion [0.704, 0.691, 0.118, 0.115]

    # 运货-后: Translation: [-0.086, 0.029, 0.561]
    #               Quaternion [-0.450, 0.878, 0.145, -0.078
                
    # 放置物品: Translation: [0.731, -0.025, 0.354]
    #                 Quaternion [0.719, 0.673, 0.131, 0.115]

    # */


    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "aubo_base"
    # pose_goal.pose.orientation.w = 1.0
    # pose_goal.pose.position.x = 0.18
    # pose_goal.pose.position.y = -0.15
    # pose_goal.pose.position.z = 0.35
    pose_goal.pose.orientation.x = 0.711
    pose_goal.pose.orientation.y = 0.703
    pose_goal.pose.orientation.z = 0.011
    pose_goal.pose.orientation.w = 0.016
    pose_goal.pose.position.x = 0.712912
    pose_goal.pose.position.y = -0.007153
    pose_goal.pose.position.z = 0.465334
    aubo_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="wrist3_Link")

    aubo_arm.set_start_state_to_current_state()
        # plan to goal
    plan_and_execute(aubo, aubo_arm, logger, sleep_time=5.0)


    # from scipy.spatial.transform import Rotation
    # pose_goal = PoseStamped()
    # rot = Rotation.from_euler('xyz', [3.14, 0., 1.4], degrees=False)
    # rot_quat = rot.as_quat()
    # pose_goal.header.frame_id = "aubo_base"
    # pose_goal.pose.orientation.x = rot_quat[0]
    # pose_goal.pose.orientation.y = rot_quat[1]
    # pose_goal.pose.orientation.z = rot_quat[2]
    # pose_goal.pose.orientation.w = rot_quat[3]
    # pose_goal.pose.position.x = -0.41
    # pose_goal.pose.position.y = -0.153
    # pose_goal.pose.position.z = 0.352
    # aubo_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "wrist3_Link")

    # multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
    #         aubo, ["ompl_rrtc", "pilz_lin"]
    #     )
    # # plan to goal
    # plan_and_execute(aubo, aubo_arm, logger, multi_plan_parameters=multi_pipeline_plan_request_params,
    #                 sleep_time=3.0)



    # ###########################################################################
    # # Plan 4 - set goal state with constraints
    # ###########################################################################
    '''
    robot_model = aubo.get_robot_model()
    robot_state = RobotState(robot_model)
    # set plan start state to current state
    aubo_arm.set_start_state_to_current_state()

    # set constraints message
    from moveit.core.kinematic_constraints import construct_joint_constraint

    # 初始姿态
    first_values = {
        "foreArm_joint": -2.4108334358394976,
        "wrist1_joint": 0.19090251921361948,
        "upperArm_joint": -1.034534295735007,
        "wrist2_joint": -1.5654776861404676,
        "wrist3_joint": -0.10093312750890894,
        "shoulder_joint": 1.5002197993246449,
    }
    # 抓取前姿态
    grasp_values = {
        "foreArm_joint": -0.9656377251059474,
        "wrist1_joint": 0.26844829995519176,
        "upperArm_joint": 0.33847095519177534,
        "wrist2_joint": -1.5632255169254201,
        "wrist3_joint": -0.30126247877899875,
        "shoulder_joint": 1.2959143056618647,
    }
    # 抓取后（前）姿态
    front_values = {
        "foreArm_joint": -1.9574101499793466,
        "wrist1_joint": 0.2739686821516851,
        "upperArm_joint": -0.6628273384951079,
        "wrist2_joint": -1.55778949937379,
        "wrist3_joint": -0.05559996899298767,
        "shoulder_joint": 1.5361444659242836,
    }
    # 抓取后（后）姿态
    back_values = {
        "foreArm_joint": 2.1753790480819526,
        "wrist1_joint": -1.7941755662528698,
        "upperArm_joint": 0.953665281591426,
        "wrist2_joint": -1.6257140425729024,
        "wrist3_joint": -3.079727692708692,
        "shoulder_joint": 1.5436565873451593,
    }
    # 放置物体姿态
    set_values = {
        "foreArm_joint": -0.965094856956392,
        "wrist1_joint": -0.35666070622992196,
        "upperArm_joint": 0.9411279617591244,
        "wrist2_joint": -1.624441236843877,
        "wrist3_joint": -0.18545916560591436,
        "shoulder_joint": 1.3522111999819844,
    }
    
    for i in [first_values, grasp_values, front_values, back_values, set_values]:
        
        robot_state.joint_positions = i
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=aubo.get_robot_model().get_joint_model_group("aubo_arm"),
        )
        aubo_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        plan_and_execute(aubo, aubo_arm, logger, sleep_time=3.0)
    '''
    # ###########################################################################
    # # Plan 5 - Planning with Multiple Pipelines simultaneously
    # ###########################################################################

    # # set plan start state to current state
    # aubo_arm.set_start_state_to_current_state()

    # # set pose goal with PoseStamped message
    # aubo_arm.set_goal_state(configuration_name="ready")

    # # initialise multi-pipeline plan request parameters

    # multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
    #     aubo, ["ompl_rrtc", "pilz_lin"]
    # )

    # # plan to goal
    # plan_and_execute(
    #     aubo,
    #     aubo_arm,
    #     logger,
    #     multi_plan_parameters=multi_pipeline_plan_request_params,
    #     sleep_time=3.0,
    # )


if __name__ == "__main__":
    main()
