#include <tesseract_planning_nodes/planning_manager_node.h>
#include <console_bridge/console.h>

using namespace tesseract_planning_nodes;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

PlanningManagerNode::PlanningManagerNode()
  : rclcpp::Node ("planning_manager_node")
  , worker_status_srv_(this->create_service<UpdatePlanningWorkerStatus>("update_planning_worker_status",
                                                                        std::bind(&PlanningManagerNode::handle_update_planning_worker_status, this, _1, _2, _3)))
  , solve_plan_cb_group_(this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant))
  , solve_plan_as_(rclcpp_action::create_server<SolvePlan>(
                     this->get_node_base_interface(),
                     this->get_node_clock_interface(),
                     this->get_node_logging_interface(),
                     this->get_node_waitables_interface(),
                     "solve_plan",
                     std::bind(&PlanningManagerNode::solve_plan_handle_goal, this, _1, _2),
                     std::bind(&PlanningManagerNode::solve_plan_handle_cancel, this, _1),
                     std::bind(&PlanningManagerNode::solve_plan_execute, this, _1),
                     rcl_action_server_get_default_options(),
                     solve_plan_cb_group_))
{

}

unsigned long PlanningManagerNode::getNumClients()
{
  return solve_plan_clients_.size();
}

void PlanningManagerNode::handle_update_planning_worker_status(const std::shared_ptr<rmw_request_id_t> request_header,
                                                               const std::shared_ptr<UpdatePlanningWorkerStatus::Request> request,
                                                               const std::shared_ptr<UpdatePlanningWorkerStatus::Response> response)
{
  std::string id = request->id;

  RCLCPP_INFO(this->get_logger(), "Received service request from worker id %s", id.c_str());

  if (request->action == UpdatePlanningWorkerStatus::Request::REGISTER)
  {
    auto res = solve_plan_clients_.emplace(id, std::make_pair(false, rclcpp_action::create_client<SolvePlan>(this->get_node_base_interface(),
                                                                                                             this->get_node_graph_interface(),
                                                                                                             this->get_node_logging_interface(),
                                                                                                             this->get_node_waitables_interface(),
                                                                                                             id + "/solve_plan")));
    if (!res.second)
    {
      // a client with this ID already exists in the map
      response->success = false;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Registered worker with id %s", id.c_str());
  }
  else if (request->action == UpdatePlanningWorkerStatus::Request::DEREGISTER)
  {
    auto client_mapped = solve_plan_clients_.find(id);

    if (client_mapped == solve_plan_clients_.end())
    {
      // no client with this ID  exists in the map
      response->success = false;
      return;
    }

    if (client_mapped->second.first)
    {
      // cannot erase a client marked as busy
      response->success = false;
      return;
    }

    solve_plan_clients_.erase(client_mapped);
    RCLCPP_INFO(this->get_logger(), "Deregistered worker with id %s", id.c_str());
  }
  else if (request->action == UpdatePlanningWorkerStatus::Request::UPDATE)
  {
    auto client_mapped = solve_plan_clients_.find(id);
    if (request->status == UpdatePlanningWorkerStatus::Request::BUSY)
    {
      client_mapped->second.first = true;
      RCLCPP_INFO(this->get_logger(), "Worker with id %s is now BUSY", id.c_str());
    }
    else if (request->status == UpdatePlanningWorkerStatus::Request::IDLE)
    {
      client_mapped->second.first = false;
      RCLCPP_INFO(this->get_logger(), "Worker with id %s is now IDLE", id.c_str());
    }
    else
    {
      response->success = false;
      return;
    }
  }
  else
  {
    response->success = false;
    return;
  }

  response->success = true;
  return;
}

rclcpp_action::GoalResponse PlanningManagerNode::solve_plan_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SolvePlan::Goal> goal)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanningManagerNode::solve_plan_handle_cancel(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  (void) goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanningManagerNode::PlanningManagerNode::solve_plan_execute(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle)
{
  auto goal_response_cb = [goal_handle](std::shared_future<ClientGoalHandleSolvePlan::SharedPtr> future)
  {
    auto gh = future.get();
    if (!gh)
    {
      CONSOLE_BRIDGE_logWarn("Goal rejected");
      auto result = std::make_shared<SolvePlan::Result>();
      goal_handle->abort(result);
    }
  };

  auto result_cb = [goal_handle](const ClientGoalHandleSolvePlan::WrappedResult& result)
  {
    auto solve_plan_result = std::make_shared<SolvePlan::Result>();
    if (result.code == rclcpp_action::ResultCode::ABORTED || result.code == rclcpp_action::ResultCode::CANCELED)
    {
      CONSOLE_BRIDGE_logWarn("Goal aborted or canceled");
      goal_handle->abort(solve_plan_result);
      return;
    }

    solve_plan_result = result.result;
    goal_handle->succeed(solve_plan_result);
    return;
  };

  // get an idle client
  for (auto it = solve_plan_clients_.begin(); it != solve_plan_clients_.end(); ++it)
  {
    if (!it->second.first)
    {
      SolvePlan::Goal goal;
      goal.planner_config = goal_handle->get_goal()->planner_config;
      goal.urdf_path = goal_handle->get_goal()->urdf_path;
      goal.srdf_path = goal_handle->get_goal()->srdf_path;
      goal.tesseract_state = goal_handle->get_goal()->tesseract_state;

      auto send_goal_options = rclcpp_action::Client<SolvePlan>::SendGoalOptions();
      send_goal_options.goal_response_callback = goal_response_cb;
      send_goal_options.result_callback = result_cb;
      auto goal_handle_future = it->second.second->async_send_goal(goal, send_goal_options);
      return;
    }
  }

  RCLCPP_WARN(this->get_logger(), "No idle workers available: aborting");

  goal_handle->abort(std::make_shared<SolvePlan::Result>());
}

int main(int argc, char** argv)
{
  rclcpp::InitOptions init_ops;
  init_ops.shutdown_on_sigint = false;
  rclcpp::init(argc, argv, init_ops);

  auto node = std::make_shared<tesseract_planning_nodes::PlanningManagerNode>();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);

  auto start = node->now();
  auto elapsed = rclcpp::Duration(std::chrono::seconds(0));

  std::cout << elapsed.seconds() << std::endl;

  rclcpp::Rate rate(10);
  while (true)
  {
    if (node->getNumClients() == 0 && elapsed.seconds() >= 5.0)
      break;

    elapsed = node->now() - start;
    exec.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
