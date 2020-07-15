#pragma once

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <thread>
#include <random>

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_msgs/msg/planner_config.hpp>
#include <tesseract_msgs/action/solve_plan.hpp>


namespace tesseract_planning_nodes
{
class PlannerBT
{
public:
  PlannerBT(const tesseract_msgs::action::SolvePlan::Goal solve_plan_cfg);

  void RegisterNodes(BT::BehaviorTreeFactory& factory);

  trajectory_msgs::msg::JointTrajectory getResult()
  {
    return result_;
  }

//  bool isDone()
//  {
//    return is_done_;
//  }

protected:
  BT::NodeStatus planJointInterp();

  BT::NodeStatus planTrajOpt();

  BT::NodeStatus planOMPL();

private:
  tesseract_msgs::msg::PlannerConfig config_;

  trajectory_msgs::msg::JointTrajectory result_;

//  std::atomic_bool succeeded_;

  std::shared_ptr<tesseract::Tesseract> tesseract_local_;

};
}


//namespace PlanFreespace
//{

//struct PlanRequest
//{
//  std::double_t start, end;
//};

//struct PlanResult
//{
//  std::vector<std::double_t> traj;
//};

//class PlanFreespaceManager
//{
//public:
//  PlanFreespaceManager(const PlanFreespace::PlanRequest& req);

//  void RegisterNodes(BT::BehaviorTreeFactory& factory);

//  PlanResult getResult();

//protected:
//  BT::NodeStatus planJointInterp();

//  BT::NodeStatus planTrajOpt();

//  BT::NodeStatus planOMPL();

//  PlanRequest req_;

//  bool result_valid_;
//  PlanResult res_;

//  std::mt19937 mt_rand;

//  bool CAN_PLAN_INTERP, CAN_PLAN_TRAJOPT, CAN_PLAN_OMPL;
//};
//}
