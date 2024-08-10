/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title       : servo.hpp
 *      Project     : moveit_servo
 *      Created     : 05/17/2023
 *      Author      : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 *
 *      Description : The core servoing logic.
 */

#pragma once

#include <moveit_servo_lib_parameters.hpp>
#include <moveit_servo/collision_monitor.hpp>
#include <moveit_servo/utils/command.hpp>
#include <moveit_servo/utils/datatypes.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <variant>

namespace moveit_servo
{

class Servo
{
public:
  Servo(const rclcpp::Node::SharedPtr& node, std::shared_ptr<const servo::ParamListener> servo_param_listener,
        const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  ~Servo();

  // Disable copy construction.
  Servo(const Servo&) = delete;

  // Disable copy assignment.
  Servo& operator=(Servo&) = delete;

  /**
   * \brief Computes the joint state required to follow the given command.
   * @param command The command to follow, std::variant type, can handle JointJog, Twist and Pose.
   * @return The required joint state.
   */
  KinematicState getNextJointState(const ServoInput& command);

  /**
   * \brief Set the type of incoming servo command.
   * @param command_type The type of command servo should expect.
   */
  void setCommandType(const CommandType& command_type);

  /**
   * \brief Get the type of command that servo is currently expecting.
   * @return The type of command.
   */
  CommandType getCommandType();

  /**
   * \brief Get the current status of servo.
   * @return The current status.
   */
  StatusCode getStatus();

  /**
   * \brief Get the message associated with the current servo status.
   * @return The status message.
   */
  const std::string getStatusMessage();

  /**
   * \brief Returns the end effector pose in planning frame
   */
  const Eigen::Isometry3d getEndEffectorPose();

  /**
   * \brief Start/Stop collision checking thread.
   * @param check_collision Stops collision checking thread if false, starts it if true.
   */
  void setCollisionChecking(const bool check_collision);

  /**
   * \brief Returns the most recent servo parameters.
   */
  servo::Params& getParams();

  /**
   * \brief Get the current state of the robot as given by planning scene monitor.
   * @return The current state of the robot.
   */
  KinematicState getCurrentRobotState();

  /**
   * \brief Smoothly halt at a commanded state when command goes stale.
   * @param The last commanded joint states.
   * @return The next state stepping towards the required halting state.
   */
  std::pair<bool, KinematicState> smoothHalt(KinematicState halt_state);

private:
  /**
   * \brief Convert a give twist command to planning frame,
   * The command frame specified by `command.frame_id` is expected to be a stationary frame or end-effector frame.
   * References:
   * https://core.ac.uk/download/pdf/154240607.pdf
   * https://www.seas.upenn.edu/~meam520/notes02/Forces8.pdf
   * @param command The twist command to be converted
   * @return The transformed twist command.
   */
  const TwistCommand toPlanningFrame(const TwistCommand& command);

  /**
   * \brief Convert a given pose command to planning frame
   * @param command The pose command to be converted
   * @return The transformed pose command
   */
  const PoseCommand toPlanningFrame(const PoseCommand& command);

  /**
   * \brief Compute the change in joint position required to follow the received command.
   * @param command The incoming servo command.
   * @return The joint position change required (delta).
   */
  Eigen::VectorXd jointDeltaFromCommand(const ServoInput& command, moveit::core::RobotStatePtr& robot_state);

  /**
   * \brief Updates data depending on joint model group
   */
  void updateJointModelGroup();

  /**
   * \brief Validate the servo parameters
   * @param servo_params The servo parameters
   * @return True if parameters are valid, else False
   */
  bool validateParams(const servo::Params& servo_params);

  /**
   * \brief Updates the servo parameters and performs validations.
   */
  bool updateParams();

  /**
   * \brief Create and initialize the smoothing plugin to be used by servo.
   */
  void setSmoothingPlugin();

  /**
   * \brief Apply halting logic to specified joints.
   * @param joints_to_halt The indices of joints to be halted.
   * @param current_state The current kinematic state.
   * @param target_state The target kinematic state.
   * @return The bounded kinematic state.
   */
  KinematicState haltJoints(const std::vector<int>& joints_to_halt, const KinematicState& current_state,
                            const KinematicState& target_state);

  // Variables

  StatusCode servo_status_;
  // This needs to be threadsafe so it can be updated in realtime.
  std::atomic<CommandType> expected_command_type_;

  servo::Params servo_params_;
  const rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // This value will be updated by CollisionMonitor in a separate thread.
  std::atomic<double> collision_velocity_scale_ = 1.0;
  std::unique_ptr<CollisionMonitor> collision_monitor_;

  pluginlib::UniquePtr<online_signal_smoothing::SmoothingBaseClass> smoother_ = nullptr;

  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;
};

}  // namespace moveit_servo
