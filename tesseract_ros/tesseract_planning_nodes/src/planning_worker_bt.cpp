#include <tesseract_planning_nodes/planning_worker_bt.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_rosutils/conversions.h>

using namespace tesseract_planning_nodes;

PlannerBT::PlannerBT(const tesseract_msgs::action::SolvePlan::Goal solve_plan_cfg)
  : config_(solve_plan_cfg.planner_config)
//  , succeeded_(false)
  , tesseract_local_(std::make_shared<tesseract::Tesseract>())
{
  tesseract_scene_graph::ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  std::stringstream urdf_xml_string, srdf_xml_string;
  std::ifstream urdf_in(solve_plan_cfg.urdf_path);
  urdf_xml_string << urdf_in.rdbuf();
  std::ifstream srdf_in(solve_plan_cfg.srdf_path);
  srdf_xml_string << srdf_in.rdbuf();
  tesseract_local_->init(urdf_xml_string.str(), srdf_xml_string.str(), locator);
  tesseract_rosutils::processMsg(tesseract_local_->getEnvironment(), solve_plan_cfg.tesseract_state);
}

void PlannerBT::RegisterNodes(BT::BehaviorTreeFactory& factory)
{
    factory.registerSimpleAction("PlanJointInterp", std::bind(&PlannerBT::planJointInterp, this));
//    factory.registerSimpleAction("PlanTrajOpt", std::bind(&PlanFreespace::PlanFreespaceManager::planTrajOpt, this));
    factory.registerSimpleAction("PlanOMPL", std::bind(&PlannerBT::planOMPL, this));
}

BT::NodeStatus PlannerBT::planJointInterp()
{
  std::cout << "Trying to solve as joint interpolated trajectory..." << std::endl;

  result_ = trajectory_msgs::msg::JointTrajectory();

  Eigen::MatrixXd traj_interp;
  traj_interp.resize(config_.n_output_states, static_cast<Eigen::Index>(config_.end_state.position.size()));
  for (std::size_t i = 0; i < config_.end_state.position.size(); i++)
  {
    traj_interp.col(i) = Eigen::VectorXd::LinSpaced(config_.n_output_states, config_.start_state.position.at(i), config_.end_state.position.at(i));
  }

  if(config_.collision_check)
  {
    auto state_solver = tesseract_local_->getEnvironment()->getStateSolver();
    auto contact_manager = tesseract_local_->getEnvironment()->getContinuousContactManager();
    contact_manager->setContactDistanceThreshold(config_.collision_safety_margin);

    std::vector<tesseract_collision::ContactResultMap> contact_result;
    tesseract_collision::ContactRequest request(tesseract_collision::ContactTestType::FIRST);
    if(tesseract_environment::checkTrajectory(contact_result, *contact_manager, *state_solver, config_.end_state.name, traj_interp))
    {
      std::cout << "Joint interpolation failed" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  tesseract_rosutils::toMsg(result_, config_.end_state.name, traj_interp);

  std::cout << "Joint interpolation succeeded" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus PlannerBT::planOMPL()
{
  std::cout << "Trying to solve as OMPL FS trajectory..." << std::endl;

  result_ = trajectory_msgs::msg::JointTrajectory();

  tesseract_kinematics::ForwardKinematics::ConstPtr kin = tesseract_local_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(config_.manipulator);

  // Deserialize the planner config message into a tesseract_motion_planners planner object.
  // For this initial implementation only RRTconnect is supported.

  std::vector<tesseract_motion_planners::OMPLPlannerConfigurator::ConstPtr> ompl_configurators;

  for (auto configurator : config_.configurators)
  {
    if (configurator.type != tesseract_msgs::msg::PlannerConfigurator::RRT_CONNECT)
    {
      return BT::NodeStatus::FAILURE;
    }

    auto rrt_connect_configurator = std::make_shared<tesseract_motion_planners::RRTConnectConfigurator>();
    rrt_connect_configurator->range = configurator.range;

    ompl_configurators.insert(ompl_configurators.end(), 1, rrt_connect_configurator);
  }

  auto planner_config = std::make_shared<tesseract_motion_planners::OMPLPlannerFreespaceConfig>(tesseract_local_, config_.manipulator, ompl_configurators);

  planner_config->start_waypoint = tesseract_rosutils::toWaypoint(config_.start_state);
  planner_config->end_waypoint = tesseract_rosutils::toWaypoint(config_.end_state);
  planner_config->collision_safety_margin = config_.collision_safety_margin;
  planner_config->simplify = config_.simplify;
  planner_config->planning_time = config_.planning_time;
  planner_config->collision_continuous = config_.collision_continuous;
  planner_config->collision_check = config_.collision_check;
  planner_config->max_solutions = config_.max_solutions;
  planner_config->longest_valid_segment_length = config_.longest_valid_segment_length;
  planner_config->n_output_states = config_.n_output_states;

  tesseract_motion_planners::OMPLMotionPlanner planner;

  if (!planner.setConfiguration(planner_config))
  {
    return BT::NodeStatus::FAILURE;
  }

  tesseract_motion_planners::PlannerResponse planner_res;
  tesseract_common::StatusCode status = planner.solve(planner_res);

  if (status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::SolutionFound && status.value() != tesseract_motion_planners::OMPLMotionPlannerStatusCategory::ErrorFoundValidSolutionInCollision)
  {
    return BT::NodeStatus::FAILURE;
  }

//  succeeded_ = true;
  tesseract_rosutils::toMsg(result_, kin->getJointNames(), planner_res.joint_trajectory.trajectory);

  return BT::NodeStatus::SUCCESS;
}
