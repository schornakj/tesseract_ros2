#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tesseract/tesseract.h>
#include <tesseract_msgs/action/solve_plan.hpp>


namespace tesseract_planning_nodes
{
  class MotionManagerNode : public rclcpp::Node
  {
  public:
    using Trigger =  std_srvs::srv::Trigger;
    using SolvePlan = tesseract_msgs::action::SolvePlan;
    using ClientGoalHandleSolvePlan = rclcpp_action::ClientGoalHandle<SolvePlan>;

    MotionManagerNode();

  private:
    void handle_do_motion(const std::shared_ptr<rmw_request_id_t> request_header,
                          const std::shared_ptr<Trigger::Request> request,
                          const std::shared_ptr<Trigger::Response> response);

    rclcpp::Service<Trigger>::SharedPtr do_motion_srv_;

    rclcpp_action::Client<SolvePlan>::SharedPtr solve_plan_client_;

    std::string urdf_, urdf_path_;
    std::string srdf_, srdf_path_;

    std::shared_ptr<tesseract::Tesseract> tesseract_;
  };
}  // namespace tesseract_planning_nodes
