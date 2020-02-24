#include <tesseract_planning_nodes/planning_worker_node.h>

#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_rosutils/conversions.h>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;

PlanningWorkerNode::PlanningWorkerNode()
  : rclcpp::Node ("planning_worker_node")
  , solve_plan_as_(rclcpp_action::create_server<SolvePlan>(
                     this->get_node_base_interface(),
                     this->get_node_clock_interface(),
                     this->get_node_logging_interface(),
                     this->get_node_waitables_interface(),
                     "solve_plan",
                     std::bind(&PlanningWorkerNode::solve_plan_handle_goal, this, _1, _2),
                     std::bind(&PlanningWorkerNode::solve_plan_handle_cancel, this, _1),
                     std::bind(&PlanningWorkerNode::solve_plan_handle_accepted, this, _1)))
  , environment_state_sub_(this->create_subscription<tesseract_msgs::msg::TesseractState>("monitored_tesseract", 10, std::bind(&PlanningWorkerNode::on_environment_updated, this, _1)))
{
  this->declare_parameter("robot_description");
  this->declare_parameter("robot_description_semantic");

  std::string urdf_path, srdf_path;
  if (!this->get_parameter("robot_description", urdf_path))
  {
    return;
  }
  if (!this->get_parameter("robot_description_semantic", srdf_path))
  {
    return;
  }
  std::stringstream urdf_xml_string, srdf_xml_string;
  std::ifstream urdf_in(urdf_path);
  urdf_xml_string << urdf_in.rdbuf();
  std::ifstream srdf_in(srdf_path);
  srdf_xml_string << srdf_in.rdbuf();

  tesseract_local_ = std::make_shared<tesseract::Tesseract>();
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);
}

rclcpp_action::GoalResponse PlanningWorkerNode::solve_plan_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SolvePlan::Goal> goal)
{
  (void) uuid;

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningWorkerNode::solve_plan_handle_cancel(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningWorkerNode::solve_plan_handle_accepted(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  std::thread{std::bind(&PlanningWorkerNode::solve_plan_execute, this, _1), goal_handle}.detach();
}

void PlanningWorkerNode::solve_plan_execute(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  auto solve_plan_result = std::make_shared<SolvePlan::Result>();

  auto goal = goal_handle->get_goal();

  tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_local_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(goal->planner_config.manipulator);


  // Deserialize the planner config message into a tesseract_motion_planners planner object.
  // For this initial implementation only RRTconnect is supported.

  std::vector<tesseract_motion_planners::OMPLPlannerConfigurator::ConstPtr> ompl_configurators;

  for (auto configurator : goal->planner_config.configurators)
  {
    if (configurator.type != tesseract_msgs::msg::PlannerConfigurator::RRT_CONNECT)
    {
      goal_handle->abort(solve_plan_result);
      return;
    }

    auto rrt_connect_configurator = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
    rrt_connect_configurator->range = configurator.range;

    ompl_configurators.insert(ompl_configurators.end(), 1, rrt_connect_configurator);
  }

  auto planner_config = std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig>(tesseract_local_, goal->planner_config.manipulator, ompl_configurators);

  auto config_msg = goal->planner_config;

  planner_config->start_waypoint = tesseract_rosutils::toWaypoint(config_msg.start_state);
  planner_config->end_waypoint = tesseract_rosutils::toWaypoint(config_msg.end_state);
  planner_config->collision_safety_margin = config_msg.collision_safety_margin;
  planner_config->simplify = config_msg.simplify;
  planner_config->planning_time = config_msg.planning_time;
  planner_config->collision_continuous = config_msg.collision_continuous;
  planner_config->collision_check = config_msg.collision_check;
  planner_config->max_solutions = config_msg.max_solutions;
  planner_config->longest_valid_segment_length = config_msg.longest_valid_segment_length;
  planner_config->n_output_states = config_msg.n_output_states;

  tesseract_motion_planners::OMPLMotionPlanner planner;

  if (!planner.setConfiguration(planner_config))
  {
    goal_handle->abort(solve_plan_result);
    return;
  }

  tesseract_motion_planners::PlannerResponse planner_res;
  tesseract_common::StatusCode status = planner.solve(planner_res);

  if (status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::SolutionFound && status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    goal_handle->abort(solve_plan_result);
    return;
  }

  tesseract_rosutils::toMsg(solve_plan_result->trajectory, kin->getJointNames(), planner_res.joint_trajectory.trajectory);

  goal_handle->succeed(solve_plan_result);
}

void PlanningWorkerNode::on_environment_updated(const tesseract_msgs::msg::TesseractState::SharedPtr msg)
{
  const tesseract_environment::Environment::Ptr env = tesseract_local_->getEnvironment();
  tesseract_rosutils::processMsg(env, *msg);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tesseract_planning_nodes::PlanningWorkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
