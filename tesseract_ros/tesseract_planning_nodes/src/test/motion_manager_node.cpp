#include <tesseract_planning_nodes/test/motion_manager_node.h>
#include <tesseract_rosutils/utils.h>

#include <fstream>

#include <Eigen/Dense>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

const static std::vector<double> start_state = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const static std::vector<double> end_state = { 0.2, 0.0, 0.0, 0.0, 0.0, 0.0 };

//Eigen::VectorXd randomWithinLimits(const Eigen::MatrixX2d& limits)
//{

//}

MotionManagerNode::MotionManagerNode()
  : rclcpp::Node("motion_manager_node")
  , do_motion_srv_(this->create_service<Trigger>("do_motion",
                                                 std::bind(&MotionManagerNode::handle_do_motion, this, _1, _2, _3)))
  , solve_plan_client_(rclcpp_action::create_client<SolvePlan>(this->get_node_base_interface(),
                                                               this->get_node_graph_interface(),
                                                               this->get_node_logging_interface(),
                                                               this->get_node_waitables_interface(),
                                                               "/solve_plan"))
  , tesseract_(std::make_shared<tesseract::Tesseract>())
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
  cfg.simplify = true;
  cfg.planning_time = 30.0;
  cfg.collision_continuous = true;
  cfg.collision_check = true;
  cfg.max_solutions = 1;
  cfg.longest_valid_segment_length = 0.01;
  cfg.n_output_states = 100;

  solve_plan_goal.planner_config = cfg;

  std::cout << "4" << std::endl;


  std::vector<trajectory_msgs::msg::JointTrajectory> trajectories;
  bool succeeded = false;

  auto goal_response_cb = [srv_response](std::shared_future<ClientGoalHandleSolvePlan::SharedPtr> future)
  {
    auto gh = future.get();
    if (!gh)
    {
      CONSOLE_BRIDGE_logWarn("Goal rejected");
    }
  };

  auto result_cb = [&trajectories, &succeeded](const ClientGoalHandleSolvePlan::WrappedResult& result)
  {
    CONSOLE_BRIDGE_logWarn("Got a result!");

    if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED)
    {
      CONSOLE_BRIDGE_logWarn("Goal aborted or canceled");
      return;
    }

    trajectory_msgs::msg::JointTrajectory traj = result.result->trajectory;
    trajectories.push_back(traj);
    succeeded = true;
    return;
  };

  std::cout << "5" << std::endl;


  auto send_goal_options = rclcpp_action::Client<SolvePlan>::SendGoalOptions();
  send_goal_options.goal_response_callback = goal_response_cb;
  send_goal_options.result_callback = result_cb;
  auto goal_handle_future = solve_plan_client_->async_send_goal(solve_plan_goal, send_goal_options);
  goal_handle_future.wait();

  std::cout << "6" << std::endl;


  if (!succeeded)
  {
    srv_response->success = false;
    return;
  }

  std::cout << "7" << std::endl;

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::sleep_for(std::chrono::seconds(3));
  auto node = std::make_shared<tesseract_planning_nodes::MotionManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
