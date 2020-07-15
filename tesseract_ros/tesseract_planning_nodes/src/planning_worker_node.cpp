#include <tesseract_planning_nodes/planning_worker_node.h>
#include <tesseract_planning_nodes/planning_worker_bt.h>

#include <tesseract_motion_planners/ompl/config/ompl_planner_freespace_config.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/core/waypoint.h>
#include <tesseract_rosutils/conversions.h>

#include <console_bridge/console.h>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence>
            <Fallback name="root_Fallback">
                <PlanJointInterp/>
                <PlanOMPL/>
            </Fallback>
        </Sequence>
    </BehaviorTree>
</root>
 )";

PlanningWorkerNode::PlanningWorkerNode()
  : PlanningWorkerNode("planning_worker_test")
{
}

PlanningWorkerNode::PlanningWorkerNode(const std::string& id)
  : rclcpp::Node (id)
  , id_(id)
  , solve_plan_as_(rclcpp_action::create_server<SolvePlan>(
                     this->get_node_base_interface(),
                     this->get_node_clock_interface(),
                     this->get_node_logging_interface(),
                     this->get_node_waitables_interface(),
                     id_ + "/solve_plan",
                     std::bind(&PlanningWorkerNode::solve_plan_handle_goal, this, _1, _2),
                     std::bind(&PlanningWorkerNode::solve_plan_handle_cancel, this, _1),
                     std::bind(&PlanningWorkerNode::solve_plan_handle_accepted, this, _1)))
  , worker_status_client_(this->create_client<UpdatePlanningWorkerStatus>("update_planning_worker_status"))
{
}

PlanningWorkerNode::~PlanningWorkerNode()
{
  notify_manager(UpdatePlanningWorkerStatus::Request::DEREGISTER, 0);
}

void PlanningWorkerNode::initialize()
{
  worker_status_client_->wait_for_service();
  notify_manager(UpdatePlanningWorkerStatus::Request::REGISTER, 0);
}

void PlanningWorkerNode::notify_manager(const uint8_t action, const uint8_t status)
{
  auto notify_req = std::make_shared<UpdatePlanningWorkerStatus::Request>();
  notify_req->id = id_;
  notify_req->action = action;
  notify_req->status = status;
  auto res_future = worker_status_client_->async_send_request(notify_req);
  res_future.wait_for(std::chrono::duration<double>(2.0));
}

rclcpp_action::GoalResponse PlanningWorkerNode::solve_plan_handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SolvePlan::Goal> goal)
{
  (void) uuid;

  // TODO: check validity of message contents, and reject if they're incorrect

  notify_manager(UpdatePlanningWorkerStatus::Request::UPDATE, UpdatePlanningWorkerStatus::Request::BUSY);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningWorkerNode::solve_plan_handle_cancel(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningWorkerNode::solve_plan_handle_accepted(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  PlanningWorkerNode::solve_plan_execute(goal_handle);
}

void PlanningWorkerNode::solve_plan_execute(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  CONSOLE_BRIDGE_logInform("Executing solve_plan request");
  auto solve_plan_result = std::make_shared<SolvePlan::Result>();

  PlannerBT planner_bt(*(goal_handle->get_goal()));

  BT::BehaviorTreeFactory factory;
  planner_bt.RegisterNodes(factory);
  auto tree = factory.createTreeFromText(xml_text);


  rclcpp::Rate rate(100);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while( status == BT::NodeStatus::RUNNING)
  {
      status = tree.tickRoot();
      rate.sleep();
  }

  if (status == BT::NodeStatus::FAILURE)
  {
      goal_handle->abort(solve_plan_result);
      notify_manager(UpdatePlanningWorkerStatus::Request::UPDATE, UpdatePlanningWorkerStatus::Request::IDLE);
      return;
  }

  solve_plan_result->trajectory = planner_bt.getResult();
  goal_handle->succeed(solve_plan_result);
  notify_manager(UpdatePlanningWorkerStatus::Request::UPDATE, UpdatePlanningWorkerStatus::Request::IDLE);
  return;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // TODO: request a unique process ID from the manager, or find a ROS2 way to generate one
  auto id = std::chrono::high_resolution_clock::now().time_since_epoch().count();

  {
    auto node = std::make_shared<tesseract_planning_nodes::PlanningWorkerNode>("planning_worker_" + std::to_string(id));
    rclcpp::sleep_for(std::chrono::seconds(1));
    node->initialize();
    rclcpp::spin(node);
  }

  // worker node deregisters from manager when it is destroyed, call shutdown only after it has gone out of scope
  rclcpp::shutdown();
  return 0;
}
