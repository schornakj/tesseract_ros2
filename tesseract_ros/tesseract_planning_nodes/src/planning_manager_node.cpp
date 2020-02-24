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
{

}

void PlanningManagerNode::handle_update_planning_worker_status(const std::shared_ptr<rmw_request_id_t> request_header,
                                                               const std::shared_ptr<UpdatePlanningWorkerStatus::Request> request,
                                                               const std::shared_ptr<UpdatePlanningWorkerStatus::Response> response)
{
  std::string id = request->id;

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

    CONSOLE_BRIDGE_logInform("Registered worker with id %s", id.c_str());
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
    CONSOLE_BRIDGE_logInform("Deregistered worker with id %s", id.c_str());
  }
  else if (request->action == UpdatePlanningWorkerStatus::Request::UPDATE)
  {
    auto client_mapped = solve_plan_clients_.find(id);
    if (request->status == UpdatePlanningWorkerStatus::Request::BUSY)
    {
      client_mapped->second.first = true;
      CONSOLE_BRIDGE_logInform("Worker with id %s is now BUSY", id.c_str());
    }
    else if (request->status == UpdatePlanningWorkerStatus::Request::IDLE)
    {
      client_mapped->second.first = false;
      CONSOLE_BRIDGE_logInform("Worker with id %s is now IDLE", id.c_str());
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<tesseract_planning_nodes::PlanningManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
