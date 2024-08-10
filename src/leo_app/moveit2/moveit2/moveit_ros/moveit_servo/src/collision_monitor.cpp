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
/*
 * Title      : collision_monitor.cpp
 * Project    : moveit_servo
 * Created    : 06/08/2023
 * Author     : Brian O'Neil, Andy Zelenak, Blake Anderson, V Mohammed Ibrahim
 */

#include <moveit_servo/collision_monitor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_servo.servo");
}

namespace moveit_servo
{

CollisionMonitor::CollisionMonitor(const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                   const servo::Params& servo_params, std::atomic<double>& collision_velocity_scale)
  : servo_params_(servo_params)
  , planning_scene_monitor_(planning_scene_monitor)
  , robot_state_(planning_scene_monitor->getPlanningScene()->getCurrentState())
  , collision_velocity_scale_(collision_velocity_scale)
{
}

void CollisionMonitor::start()
{
  stop_requested_ = false;
  if (!monitor_thread_.joinable())
  {
    monitor_thread_ = std::thread(&CollisionMonitor::checkCollisions, this);
    RCLCPP_INFO_STREAM(LOGGER, "Collision monitor started");
  }
  else
  {
    RCLCPP_INFO_STREAM(LOGGER, "Collision monitor could not be started");
  }
}

void CollisionMonitor::stop()
{
  stop_requested_ = true;
  if (monitor_thread_.joinable())
  {
    monitor_thread_.join();
  }
  RCLCPP_INFO_STREAM(LOGGER, "Collision monitor stopped");
}

void CollisionMonitor::checkCollisions()
{
  rclcpp::WallRate rate(servo_params_.collision_check_rate);

  bool approaching_self_collision, approaching_scene_collision;
  double self_collision_threshold_delta, scene_collision_threshold_delta;
  double self_collision_scale, scene_collision_scale;
  const double self_velocity_scale_coefficient{ -log(0.001) / servo_params_.self_collision_proximity_threshold };
  const double scene_velocity_scale_coefficient{ -log(0.001) / servo_params_.scene_collision_proximity_threshold };

  while (rclcpp::ok() && !stop_requested_)
  {
    // Reset the scale on every iteration.
    collision_velocity_scale_ = 1.0;

    if (servo_params_.check_collisions)
    {
      // Fetch latest robot state using planning scene instead of state monitor due to
      // https://github.com/ros-planning/moveit2/issues/2748
      robot_state_ = planning_scene_monitor_->getPlanningScene()->getCurrentState();
      // This must be called before doing collision checking.
      robot_state_.updateCollisionBodyTransforms();

      // Get a read-only copy of planning scene.
      planning_scene_monitor::LockedPlanningSceneRO locked_scene(planning_scene_monitor_);

      // Check collision with environment.
      scene_collision_result_.clear();
      locked_scene->getCollisionEnv()->checkRobotCollision(scene_collision_request_, scene_collision_result_,
                                                           robot_state_, locked_scene->getAllowedCollisionMatrix());

      // Check robot self collision.
      self_collision_result_.clear();
      locked_scene->getCollisionEnvUnpadded()->checkSelfCollision(
          self_collision_request_, self_collision_result_, robot_state_, locked_scene->getAllowedCollisionMatrix());

      // If collision detected scale velocity to 0, else start decelerating exponentially.
      // velocity_scale = e ^ k * (collision_distance - threshold)
      // k = - ln(0.001) / collision_proximity_threshold
      // velocity_scale should equal one when collision_distance is at collision_proximity_threshold.
      // velocity_scale should equal 0.001 when collision_distance is at zero.

      if (self_collision_result_.collision || scene_collision_result_.collision)
      {
        collision_velocity_scale_ = 0.0;
      }
      else
      {
        self_collision_scale = scene_collision_scale = 1.0;

        approaching_scene_collision =
            scene_collision_result_.distance < servo_params_.scene_collision_proximity_threshold;
        approaching_self_collision = self_collision_result_.distance < servo_params_.self_collision_proximity_threshold;

        if (approaching_scene_collision)
        {
          scene_collision_threshold_delta =
              scene_collision_result_.distance - servo_params_.scene_collision_proximity_threshold;
          scene_collision_scale = std::exp(scene_velocity_scale_coefficient * scene_collision_threshold_delta);
        }

        if (approaching_self_collision)
        {
          self_collision_threshold_delta =
              self_collision_result_.distance - servo_params_.self_collision_proximity_threshold;
          self_collision_scale = std::exp(self_velocity_scale_coefficient * self_collision_threshold_delta);
        }

        // Use the scaling factor with lower value, i.e maximum scale down.
        collision_velocity_scale_ = std::min(scene_collision_scale, self_collision_scale);
      }
    }
    rate.sleep();
  }
}
}  // namespace moveit_servo
