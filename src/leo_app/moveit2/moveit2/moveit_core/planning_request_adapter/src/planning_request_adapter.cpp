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

/* Author: Ioan Sucan */

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/logger.hpp>
#include <functional>
#include <algorithm>

namespace planning_request_adapter
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.planning_request_adapter");

namespace
{
bool callPlannerInterfaceSolve(const planning_interface::PlannerManager& planner,
                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const planning_interface::MotionPlanRequest& req,
                               planning_interface::MotionPlanResponse& res)
{
  planning_interface::PlanningContextPtr context = planner.getPlanningContext(planning_scene, req, res.error_code);
  if (context)
  {
    return context->solve(res);
  }
  else
  {
    return false;
  }
}

bool callAdapter(const PlanningRequestAdapter& adapter, const PlanningRequestAdapter::PlannerFn& planner,
                 const planning_scene::PlanningSceneConstPtr& planning_scene,
                 const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res)
{
  try
  {
    bool result = adapter.adaptAndPlan(planner, planning_scene, req, res);
    RCLCPP_DEBUG_STREAM(LOGGER, adapter.getDescription() << ": " << moveit::core::error_code_to_string(res.error_code));
    return result;
  }
  catch (std::exception& ex)
  {
    RCLCPP_ERROR(LOGGER, "Exception caught executing adapter '%s': %s\nSkipping adapter instead.",
                 adapter.getDescription().c_str(), ex.what());
    return planner(planning_scene, req, res);
  }
}

}  // namespace

bool PlanningRequestAdapter::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                          const planning_scene::PlanningSceneConstPtr& planning_scene,
                                          const planning_interface::MotionPlanRequest& req,
                                          planning_interface::MotionPlanResponse& res) const
{
  return adaptAndPlan(
      [&planner](const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
                 planning_interface::MotionPlanResponse& res) {
        return callPlannerInterfaceSolve(*planner, scene, req, res);
      },
      planning_scene, req, res);
}

void PlanningRequestAdapterChain::addAdapter(const PlanningRequestAdapterConstPtr& adapter)
{
  adapters_.push_back(adapter);
}

bool PlanningRequestAdapterChain::adaptAndPlan(const planning_interface::PlannerManagerPtr& planner,
                                               const planning_scene::PlanningSceneConstPtr& planning_scene,
                                               const planning_interface::MotionPlanRequest& req,
                                               planning_interface::MotionPlanResponse& res) const
{
  // if there are no adapters, run the planner directly
  if (adapters_.empty())
  {
    return callPlannerInterfaceSolve(*planner, planning_scene, req, res);
  }

  // if there are adapters, construct a function for each, in order,
  // so that in the end we have a nested sequence of functions that calls all adapters
  // and eventually the planner in the correct order.
  PlanningRequestAdapter::PlannerFn fn = [&planner = *planner](const planning_scene::PlanningSceneConstPtr& scene,
                                                               const planning_interface::MotionPlanRequest& req,
                                                               planning_interface::MotionPlanResponse& res) {
    return callPlannerInterfaceSolve(planner, scene, req, res);
  };

  for (int i = adapters_.size() - 1; i >= 0; --i)
  {
    fn = [&adapter = *adapters_[i],
          fn](const planning_scene::PlanningSceneConstPtr& scene, const planning_interface::MotionPlanRequest& req,
              planning_interface::MotionPlanResponse& res) { return callAdapter(adapter, fn, scene, req, res); };
  }

  bool result = fn(planning_scene, req, res);

  std::sort(res.added_path_index.begin(), res.added_path_index.end());
  return result;
}

}  // end of namespace planning_request_adapter
