/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan
 * Desc: Fix start state collision adapter which will attempt to sample a new collision-free configuration near a
 * specified configuration (in collision) by perturbing the joint values by a small amount. The amount that it will
 * perturb the values by is specified by the jiggle_fraction parameter that controls the perturbation as a percentage of
 * the total range of motion for the joint. The other parameter for this adapter specifies how many random perturbations
 * the adapter will sample before giving up.
 */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <class_loader/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_value.hpp>

#include <default_plan_request_adapter_parameters.hpp>

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.fix_start_state_collision");

/** @brief This fix start state collision adapter will attempt to sample a new collision-free configuration near a
 * specified configuration (in collision) by perturbing the joint values by a small amount.*/
class FixStartStateCollision : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    param_listener_ =
        std::make_unique<default_plan_request_adapter_parameters::ParamListener>(node, parameter_namespace);
  }

  std::string getDescription() const override
  {
    return "Fix Start State In Collision";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res) const override
  {
    RCLCPP_DEBUG(LOGGER, "Running '%s'", getDescription().c_str());

    // get the specified start state
    moveit::core::RobotState start_state = planning_scene->getCurrentState();
    moveit::core::robotStateMsgToRobotState(planning_scene->getTransforms(), req.start_state, start_state);

    collision_detection::CollisionRequest creq;
    creq.group_name = req.group_name;
    collision_detection::CollisionResult cres;
    planning_scene->checkCollision(creq, cres, start_state);
    if (cres.collision)
    {
      // Rerun in verbose mode
      collision_detection::CollisionRequest vcreq = creq;
      collision_detection::CollisionResult vcres;
      vcreq.verbose = true;
      planning_scene->checkCollision(vcreq, vcres, start_state);

      if (creq.group_name.empty())
      {
        RCLCPP_INFO(LOGGER, "Start state appears to be in collision");
      }
      else
      {
        RCLCPP_INFO(LOGGER, "Start state appears to be in collision with respect to group %s", creq.group_name.c_str());
      }

      auto prefix_state = std::make_shared<moveit::core::RobotState>(start_state);
      random_numbers::RandomNumberGenerator& rng = prefix_state->getRandomNumberGenerator();

      const std::vector<const moveit::core::JointModel*>& jmodels =
          planning_scene->getRobotModel()->hasJointModelGroup(req.group_name) ?
              planning_scene->getRobotModel()->getJointModelGroup(req.group_name)->getJointModels() :
              planning_scene->getRobotModel()->getJointModels();

      bool found = false;
      const auto params = param_listener_->get_params();

      for (int c = 0; !found && c < params.max_sampling_attempts; ++c)
      {
        for (std::size_t i = 0; !found && i < jmodels.size(); ++i)
        {
          std::vector<double> sampled_variable_values(jmodels[i]->getVariableCount());
          const double* original_values = prefix_state->getJointPositions(jmodels[i]);
          jmodels[i]->getVariableRandomPositionsNearBy(rng, &sampled_variable_values[0], original_values,
                                                       jmodels[i]->getMaximumExtent() * params.jiggle_fraction);
          start_state.setJointPositions(jmodels[i], sampled_variable_values);
          collision_detection::CollisionResult cres;
          planning_scene->checkCollision(creq, cres, start_state);
          if (!cres.collision)
          {
            found = true;
            RCLCPP_INFO(LOGGER, "Found a valid state near the start state at distance %lf after %d attempts",
                        prefix_state->distance(start_state), c);
          }
        }
      }

      if (found)
      {
        planning_interface::MotionPlanRequest req2 = req;
        moveit::core::robotStateToRobotStateMsg(start_state, req2.start_state);
        bool solved = planner(planning_scene, req2, res);
        if (solved && !res.trajectory->empty())
        {
          // heuristically decide a duration offset for the trajectory (induced by the additional point added as a
          // prefix to the computed trajectory)
          res.trajectory->setWayPointDurationFromPrevious(0, std::min(params.start_state_max_dt,
                                                                      res.trajectory->getAverageSegmentDuration()));
          res.trajectory->addPrefixWayPoint(prefix_state, 0.0);
          // we add a prefix point, so we need to bump any previously added index positions
          for (std::size_t& added_index : res.added_path_index)
          {
            added_index++;
          }
          res.added_path_index.push_back(0);
        }
        return solved;
      }
      else
      {
        RCLCPP_WARN(
            LOGGER,
            "Unable to find a valid state nearby the start state (using jiggle fraction of %lf and %lu sampling "
            "attempts). Passing the original planning request to the planner.",
            params.jiggle_fraction, params.max_sampling_attempts);
        res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;  // skip remaining adapters and/or planner
      }
    }
    else
    {
      if (creq.group_name.empty())
      {
        RCLCPP_DEBUG(LOGGER, "Start state is valid");
      }
      else
      {
        RCLCPP_DEBUG(LOGGER, "Start state is valid with respect to group %s", creq.group_name.c_str());
      }
      return planner(planning_scene, req, res);
    }
  }

private:
  std::unique_ptr<default_plan_request_adapter_parameters::ParamListener> param_listener_;
};
}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::FixStartStateCollision,
                            planning_request_adapter::PlanningRequestAdapter)
