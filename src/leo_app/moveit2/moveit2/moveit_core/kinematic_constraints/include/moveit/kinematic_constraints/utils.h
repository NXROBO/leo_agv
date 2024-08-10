/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#pragma once

#include <moveit_msgs/msg/constraints.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <moveit/robot_state/robot_state.h>
#include <limits>

namespace XmlRpc
{
class XmlRpcValue;
}

namespace kinematic_constraints
{
/**
 * \brief Merge two sets of constraints into one.
 *
 * This just does appending of all constraints except joint
 * constraints. For members of type \ref JointConstraint, the bounds
 * specified in the parameter \e first take precedence over parameter
 * \e second
 *
 * @param [in] first The first constraint to merge
 * @param [in] second The second constraint to merge
 *
 * @return The merged set of constraints
 */
moveit_msgs::msg::Constraints mergeConstraints(const moveit_msgs::msg::Constraints& first,
                                               const moveit_msgs::msg::Constraints& second);

std::size_t countIndividualConstraints(const moveit_msgs::msg::Constraints& constr);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a joint group.  The full constraint will contain a
 * vector of type \ref JointConstraint, one for each DOF in the group.
 *
 * @param [in] state The state from which to generate goal joint constraints
 * @param [in] jmg The group for which to generate goal joint constraints
 * @param [in] tolerance_below The below tolerance to apply to all constraints [rad or meters for prismatic joints]
 * @param [in] tolerance_above The above tolerance to apply to all constraints [rad or meters for prismatic joints]
 *
 * @return A full constraint message containing all the joint constraints
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                       const moveit::core::JointModelGroup* jmg, double tolerance_below,
                                                       double tolerance_above);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a joint group.  The full constraint will contain a
 * vector of type \ref JointConstraint, one for each DOF in the group.
 *
 * @param [in] state The state from which to generate goal joint constraints
 * @param [in] jmg The group for which to generate joint constraints
 * @param [in] tolerance An angular tolerance to apply both above and below for all constraints [rad or meters for
 * prismatic joints]
 *
 * @return A full constraint message containing all the joint constraints
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const moveit::core::RobotState& state,
                                                       const moveit::core::JointModelGroup* jmg,
                                                       double tolerance = std::numeric_limits<double>::epsilon());

/**
 * \brief Update joint constraints with a new JointModelGroup state
 *
 * @param [in, out] constraints Previously-constructed constraints to update
 * @param [in] state The new target state
 * @param [in] jmg Specify which JointModelGroup to update
 *
 * @return true if all joint constraints were updated
 */
bool updateJointConstraints(moveit_msgs::msg::Constraints& constraints, const moveit::core::RobotState& state,
                            const moveit::core::JointModelGroup* jmg);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint will contain a
 * \ref PositionConstraint and a \ref OrientationConstraint,
 * constructed from the pose. A sphere will be used to represent the
 * constraint region for the \ref PositionConstraint.
 *
 * @param [in] link_name The link name for both constraints
 * @param [in] pose The pose stamped to be used for the target region.
 * @param [in] tolerance_pos The radius of a sphere defining a \ref PositionConstraint
 * @param [in] tolerance_angle The value to assign to the absolute tolerances of the \ref OrientationConstraint
 *
 * @return A full constraint message containing both constraints
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PoseStamped& pose,
                                                       double tolerance_pos = 1e-3, double tolerance_angle = 1e-2);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint will contain a
 * \ref PositionConstraint and a \ref OrientationConstraint,
 * constructed from the pose. A box  will be used to represent the
 * constraint region for the \ref PositionConstraint.
 *
 * @param [in] link_name The link name for both constraints
 * @param [in] pose The pose stamped to be used for the target region.
 * @param [in] tolerance_pos The dimensions of the box (xyz) associated with the target region of the \ref
 *PositionConstraint
 * @param [in] tolerance_angle The values to assign to the absolute tolerances (xyz) of the \ref OrientationConstraint
 *
 * @return A full constraint message containing both constraints
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PoseStamped& pose,
                                                       const std::vector<double>& tolerance_pos,
                                                       const std::vector<double>& tolerance_angle);

/**
 * \brief Update a pose constraint for one link with a new pose
 *
 * @param [in, out] constraints Previously-constructed constraints to update
 * @param [in] link The link to update for
 * @param [in] pose The new target pose
 *
 * @return true if the constraint was updated
 */
bool updatePoseConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                          const geometry_msgs::msg::PoseStamped& pose);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link. The full constraint message will
 * contain only an \ref OrientationConstraint.
 *
 * @param [in] link_name The link name for the \ref OrientationConstraint
 * @param [in] quat The quaternion for the \ref OrientationConstraint
 * @param [in] tolerance The absolute axes tolerances to apply to the \ref OrientationConstraint
 *
 * @return A full constraint message containing the orientation constraint
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::QuaternionStamped& quat,
                                                       double tolerance = 1e-2);

/**
 * \brief Update an orientation constraint for one link with a new quaternion
 *
 * @param [in, out] constraints Previously-constructed constraints to update
 * @param [in] link The link to update for
 * @param [in] quat The new target quaternion
 *
 * @return true if the constraint was updated
 */
bool updateOrientationConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                                 const geometry_msgs::msg::QuaternionStamped& quat);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint message will
 * contain only a \ref PositionConstraint.  A sphere will be used to
 * represent the constraint region.
 *
 * @param [in] link_name The link name for the \ref PositionConstraint
 * @param [in] reference_point A point corresponding to the target_point_offset of the \ref PositionConstraint
 * @param [in] goal_point The position associated with the constraint region
 * @param [in] tolerance The radius of a sphere defining a \ref PositionConstraint
 *
 * @return A full constraint message containing the position constraint
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::Point& reference_point,
                                                       const geometry_msgs::msg::PointStamped& goal_point,
                                                       double tolerance = 1e-3);

/**
 * \brief Generates a constraint message intended to be used as a goal
 * constraint for a given link.  The full constraint message will
 * contain only a \ref PositionConstraint.  A sphere will be used to
 * represent the constraint region.
 *
 * @param [in] link_name The link name for the \ref PositionConstraint
 * @param [in] goal_point The position associated with the constraint region
 * @param [in] tolerance The radius of a sphere defining a \ref PositionConstraint
 *
 * @return A full constraint message containing the position constraint
 */
moveit_msgs::msg::Constraints constructGoalConstraints(const std::string& link_name,
                                                       const geometry_msgs::msg::PointStamped& goal_point,
                                                       double tolerance = 1e-3);

/**
 * \brief Update a position constraint for one link with a new position
 *
 * @param [in, out] constraints Previously-constructed constraints to update
 * @param [in] link The link to update for
 * @param [in] goal_point The new target point
 *
 * @return true if the constraint was updated
 */
bool updatePositionConstraint(moveit_msgs::msg::Constraints& constraints, const std::string& link_name,
                              const geometry_msgs::msg::PointStamped& goal_point);

/**
 * \brief extract constraint message from node parameters.
 *
 * This can be used to construct a Constraints message from
 * specifications provided from a yaml file.
 * An example for a constraint yaml structure (loaded at constraint_param):
 * """
 * name: constraint_name
 * constraint_ids: [constraint_1, constraint_2]
 * constraints:
 *   constraint_1:
 *     type: orientation
 *     frame_id: world
 *     link_name: tool0
 *     orientation: [0, 0, 0]  # [r, p, y]
 *     tolerances: [0.01, 0.01, 3.15]
 *     weight: 1.0
 *   constraint_2:
 *     type: position
 *     frame_id: base_link
 *     link_name: tool0
 *     target_offset: [0.1, 0.1, 0.1]  # [x, y, z]
 *     region:
 *       x: [0.1, 0.4]  # [min, max]
 *       y: [0.2, 0.3]
 *       z: [0.1, 0.6]
 *     weight: 1.0
 * """
 *
 * @param [in] params Node node to load the parameters from
 * @param [in] params string namespace from where to load the constraints parameters
 * @param [out] constraints The constructed constraints message
 *
 * @return was the construction successful?
 */
bool constructConstraints(const rclcpp::Node::SharedPtr& node, const std::string& constraints_param,
                          moveit_msgs::msg::Constraints& constraints);

/**
 * \brief Resolves frames used in constraints to links in the robot model.
 *
 * The link_name field of a constraint is changed from the name of an object's frame or subframe
 * to the name of the robot link that the object is attached to.
 *
 * This is used in a planning request adapter which ensures that the planning problem is defined
 * properly (the attached objects' frames are not known to the planner).
 *
 * @param [in] state The RobotState used to resolve frames.
 * @param [in] constraints The constraint to resolve.
 */
bool resolveConstraintFrames(const moveit::core::RobotState& state, moveit_msgs::msg::Constraints& constraints);
}  // namespace kinematic_constraints
