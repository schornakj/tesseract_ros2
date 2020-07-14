#pragma once

#define BOOST_BIND_NO_PLACEHOLDERS

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <std_srvs/srv/trigger.hpp>

#include <tesseract/tesseract.h>
#include <tesseract_msgs/action/solve_plan.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <random>

namespace tesseract_planning_nodes
{
  class MotionManagerNode : public rclcpp::Node
  {
  public:
    using Trigger =  std_srvs::srv::Trigger;
    using SolvePlan = tesseract_msgs::action::SolvePlan;
    using ClientGoalHandleSolvePlan = rclcpp_action::ClientGoalHandle<SolvePlan>;

    using FollowJointTraj = control_msgs::action::FollowJointTrajectory;
    using ClientGoalHandleFollowJointTraj = rclcpp_action::ClientGoalHandle<FollowJointTraj>;

    MotionManagerNode();

  private:
    rclcpp::callback_group::CallbackGroup::SharedPtr do_motion_cb_group_;
    rclcpp::callback_group::CallbackGroup::SharedPtr solve_plan_cb_group_;

    void handle_do_motion(const std::shared_ptr<rmw_request_id_t> request_header,
                          const std::shared_ptr<Trigger::Request> request,
                          const std::shared_ptr<Trigger::Response> response);

    void solve_plan_response_cb(std::shared_future<ClientGoalHandleSolvePlan::SharedPtr> future);

    void solve_plan_result_cb(const ClientGoalHandleSolvePlan::WrappedResult& result);

    void joint_state_cb(const sensor_msgs::msg::JointState::SharedPtr msg);

    trajectory_msgs::msg::JointTrajectory applyTimeParameterization(const trajectory_msgs::msg::JointTrajectory& traj_in);

    rclcpp::Service<Trigger>::SharedPtr do_motion_srv_;

    rclcpp_action::Client<SolvePlan>::SharedPtr solve_plan_client_;

    rclcpp_action::Client<FollowJointTraj>::SharedPtr follow_traj_client_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

    std::string urdf_, urdf_path_;
    std::string srdf_, srdf_path_;

    std::atomic_bool done_;
    std::atomic_bool succeeded_;
    std::vector<trajectory_msgs::msg::JointTrajectory> trajectories_;

    std::shared_ptr<tesseract::Tesseract> tesseract_;

    std::map<std::string, std::double_t> js_map_;

    std::mt19937 mt_gen_;
    std::uniform_real_distribution<std::double_t> dist_;
  };
}  // namespace tesseract_planning_nodes
