#ifndef TESSERACT_PLANNING_WORKER_NODE_H
#define TESSERACT_PLANNING_WORKER_NODE_H

#define BOOST_BIND_NO_PLACEHOLDERS

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <tesseract_msgs/action/solve_plan.hpp>
#include <tesseract_msgs/msg/tesseract_state.hpp>
#include <tesseract_msgs/srv/update_planning_worker_status.hpp>

#include <tesseract_rosutils/utils.h>

namespace tesseract_planning_nodes
{

  class PlanningWorkerNode : public rclcpp::Node
  {
  public:
    using SolvePlan = tesseract_msgs::action::SolvePlan;
    using ServerGoalHandleSolvePlan = rclcpp_action::ServerGoalHandle<SolvePlan>;
    using UpdatePlanningWorkerStatus = tesseract_msgs::srv::UpdatePlanningWorkerStatus;

    PlanningWorkerNode();

    PlanningWorkerNode(const std::string& id);

    ~PlanningWorkerNode();

    void initialize();

  private:
    std::string id_;

    rclcpp_action::Server<SolvePlan>::SharedPtr solve_plan_as_;

    rclcpp_action::GoalResponse solve_plan_handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const SolvePlan::Goal> goal);

    rclcpp_action::CancelResponse solve_plan_handle_cancel(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle);

    void register_worker();

    void deregister_worker();

    void solve_plan_handle_accepted(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle);

    void solve_plan_execute(const std::shared_ptr<ServerGoalHandleSolvePlan> goal_handle);

    rclcpp::Subscription<tesseract_msgs::msg::TesseractState>::SharedPtr environment_state_sub_;

    void on_environment_updated(const tesseract_msgs::msg::TesseractState::SharedPtr msg);

    rclcpp::Client<UpdatePlanningWorkerStatus>::SharedPtr worker_status_client_;

    tesseract::Tesseract::Ptr tesseract_local_;
  };

}

#endif  // TESSERACT_PLANNING_WORKER_NODE_H
