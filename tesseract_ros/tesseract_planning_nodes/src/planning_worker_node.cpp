#include <tesseract_planning_nodes/planning_worker_node.h>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

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
{

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

}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("env_monitor");
//  auto monitor = std::make_shared<tesseract_monitoring::EnvironmentMonitor>("env", node);
//  monitor->postInitialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
//  monitor.reset();
  return 0;
}
