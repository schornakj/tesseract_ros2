#include <tesseract_planning_nodes/test/motion_manager_node.h>
#include <tesseract_rosutils/utils.h>

#include <iterative_spline_parameterization/iterative_spline_parameterization.h>


#include <fstream>

#include <Eigen/Dense>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

const static std::vector<double> start_state = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const static std::vector<double> end_state = { 0.5, -0.5, 0.0, 0.5, 0.5, 0.5 };

//Eigen::VectorXd randomWithinLimits(const Eigen::MatrixX2d& limits)
//{

//}

MotionManagerNode::MotionManagerNode()
  : rclcpp::Node("motion_manager_node")
  , do_motion_cb_group_(this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant))
  , solve_plan_cb_group_(this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant))
  , do_motion_srv_(this->create_service<Trigger>("do_motion",
                                                 std::bind(&MotionManagerNode::handle_do_motion, this, _1, _2, _3),
                                                 rmw_qos_profile_services_default,
                                                 do_motion_cb_group_))
  , solve_plan_client_(rclcpp_action::create_client<SolvePlan>(this->get_node_base_interface(),
                                                               this->get_node_graph_interface(),
                                                               this->get_node_logging_interface(),
                                                               this->get_node_waitables_interface(),
                                                               "/solve_plan"))
  , follow_traj_client_(rclcpp_action::create_client<FollowJointTraj>(this->get_node_base_interface(),
                                                                      this->get_node_graph_interface(),
                                                                      this->get_node_logging_interface(),
                                                                      this->get_node_waitables_interface(),
                                                                      "/follow_joint_trajectory"))
  , tesseract_(std::make_shared<tesseract::Tesseract>())
  , done_(true)
  , succeeded_(true)
{
  this->declare_parameter("urdf_path");
  this->declare_parameter("srdf_path");
  if (!this->get_parameter("urdf_path", urdf_path_))
  {
    return;
  }
  if (!this->get_parameter("srdf_path", srdf_path_))
  {
    return;
  }

  std::cout << urdf_path_ << std::endl;
  std::cout << srdf_path_ << std::endl;

  std::stringstream urdf_xml_string, srdf_xml_string;
  std::ifstream urdf_in(urdf_path_);
  urdf_xml_string << urdf_in.rdbuf();
  std::ifstream srdf_in(srdf_path_);
  srdf_xml_string << srdf_in.rdbuf();

  urdf_ = urdf_xml_string.str();
  srdf_ = srdf_xml_string.str();

  tesseract_->init(urdf_, srdf_, std::make_shared<tesseract_rosutils::ROSResourceLocator>());
}

void MotionManagerNode::solve_plan_response_cb(std::shared_future<ClientGoalHandleSolvePlan::SharedPtr> future)
{
  auto gh = future.get();
  if (!gh)
  {
    CONSOLE_BRIDGE_logWarn("Goal rejected");
    done_ = true;
    succeeded_ = false;
  }
}

void MotionManagerNode::solve_plan_result_cb(const ClientGoalHandleSolvePlan::WrappedResult& result)
{
  CONSOLE_BRIDGE_logWarn("Got a result!");

  if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED)
  {
    CONSOLE_BRIDGE_logWarn("Goal aborted or canceled");
    done_ = true;
    succeeded_ = false;
    return;
  }

  trajectory_msgs::msg::JointTrajectory traj = result.result->trajectory;
  trajectories_.push_back(traj);
  done_ = true;
  succeeded_ = true;
  return;
}

void MotionManagerNode::handle_do_motion(const std::shared_ptr<rmw_request_id_t> srv_request_header,
                                         const std::shared_ptr<Trigger::Request> srv_request,
                                         const std::shared_ptr<Trigger::Response> srv_response)
{
  // generate N reachable joint poses
  // insert the current joint pose at the beginning
  // pair them into N-1 motion segments, where each segment is from pose[N] to pose[N+1]
  // structure the segments into a graph representing their dependencies
  // while there are unsolved segments, peel them off the graph, compose them into SolvePlan goals, and send them to the server (in parallel!)
  // once all segments are solved, joint them all into a trajectory and then do time parameterization

  std::cout << "0 (start)" << std::endl;


  auto kin = tesseract_->getFwdKinematicsManager()->getFwdKinematicSolver("manipulator");
  auto limits = kin->getLimits();

  std::cout << "1" << std::endl;

  SolvePlan::Goal solve_plan_goal;
  solve_plan_goal.urdf_path = urdf_path_;
  solve_plan_goal.srdf_path = srdf_path_;
  tesseract_rosutils::toMsg(solve_plan_goal.tesseract_state, *tesseract_->getEnvironment());

  std::cout << "2" << std::endl;

  sensor_msgs::msg::JointState start;
  start.name = kin->getJointNames();
  start.position = start_state;

  sensor_msgs::msg::JointState end;
  end.name = kin->getJointNames();
  end.position = end_state;

  std::cout << "3" << std::endl;

  tesseract_msgs::msg::PlannerConfigurator rrt_connect_cfg;
  rrt_connect_cfg.type = tesseract_msgs::msg::PlannerConfigurator::RRT_CONNECT;
  rrt_connect_cfg.range = 0.5;


  tesseract_msgs::msg::PlannerConfig cfg;
  cfg.configurators = { rrt_connect_cfg };
  cfg.manipulator = "manipulator";
  cfg.start_state = start;
  cfg.end_state = end;
  cfg.collision_safety_margin = 0.01;
  cfg.simplify = false;
  cfg.planning_time = 30.0;
  cfg.collision_continuous = true;
  cfg.collision_check = true;
  cfg.max_solutions = 1;
  cfg.longest_valid_segment_length = 0.01;
  cfg.n_output_states = 100;

  solve_plan_goal.planner_config = cfg;

  std::cout << "4/5" << std::endl;

  auto send_goal_options = rclcpp_action::Client<SolvePlan>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(&MotionManagerNode::solve_plan_response_cb, this, _1);
  send_goal_options.result_callback = std::bind(&MotionManagerNode::solve_plan_result_cb, this, _1);

  trajectories_.clear();
  done_ = false;
  auto goal_handle_future = solve_plan_client_->async_send_goal(solve_plan_goal, send_goal_options);


  rclcpp::Rate rate(100);

  while(!done_)
  {
    rate.sleep();
  }

  std::cout << "6" << std::endl;


  if (!succeeded_)
  {
    srv_response->success = false;
    return;
  }

  std::cout << "7" << std::endl;

  auto traj_parameterized = applyTimeParameterization(trajectories_.front());
  traj_parameterized.joint_names = kin->getJointNames();
  traj_parameterized.header.frame_id = "world";

  FollowJointTraj::Goal ft_goal;
  ft_goal.trajectory = traj_parameterized;

  auto gh_follow_traj_future = follow_traj_client_->async_send_goal(ft_goal);

}

trajectory_msgs::msg::JointTrajectory MotionManagerNode::applyTimeParameterization(const trajectory_msgs::msg::JointTrajectory &traj_in)
{
  trajectory_msgs::msg::JointTrajectory traj_out;

  std::vector<iterative_spline_parameterization::TrajectoryState> waypoints;

  for (auto point : traj_in.points)
  {
    // Convert joint angles in message (vector of doubles) to Eigen MatrixXd
    Eigen::Matrix<double, 6, 1>  joint_angles(point.positions.data());

    waypoints.push_back(
      iterative_spline_parameterization::TrajectoryState(joint_angles,
                                                         Eigen::Matrix<double, 6, 1>::Zero(),
                                                         Eigen::Matrix<double, 6, 1>::Zero(),
                                                         0.0));
  }

  iterative_spline_parameterization::IterativeSplineParameterization isp(false);

  isp.computeTimeStamps(waypoints, 1.0, 1.0);

  for (auto point : waypoints)
  {
    // Convert joint angles from parameterization output (Eigen::MatrixXd)
    // to format in ROS message (vector of doubles)
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.time_from_start = rclcpp::Duration(std::chrono::duration<double>(point.time));

    pt.positions.resize(static_cast<std::size_t>(point.positions.size()));
    Eigen::VectorXd::Map(&pt.positions[0], point.positions.size()) = point.positions;

    pt.velocities.resize(static_cast<std::size_t>(point.velocities.size()));
    Eigen::VectorXd::Map(&pt.velocities[0], point.velocities.size()) = point.velocities;

    pt.accelerations.resize(static_cast<std::size_t>(point.accelerations.size()));
    Eigen::VectorXd::Map(&pt.accelerations[0], point.accelerations.size()) = point.accelerations;

    traj_out.points.push_back(pt);
  }

  return traj_out;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(std::chrono::seconds(3));
  auto node = std::make_shared<tesseract_planning_nodes::MotionManagerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
