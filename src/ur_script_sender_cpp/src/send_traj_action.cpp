#include <chrono>
#include <string>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;

static std::string get_action_name(int argc, char** argv) {
  // argv[2] optional
  if (argc >= 3) return argv[2];
  return "/scaled_joint_trajectory_controller/follow_joint_trajectory";
}

class TrajActionSender : public rclcpp::Node {
public:
  explicit TrajActionSender(const std::string& action_name)
  : Node("ur_traj_action_sender_cpp"), action_name_(action_name) {
    client_ = rclcpp_action::create_client<FollowJT>(this, action_name_);
  }

  void send_from_yaml(const std::string& yaml_path) {
    YAML::Node root = YAML::LoadFile(yaml_path);

    auto joint_names = root["joint_names"].as<std::vector<std::string>>();
    YAML::Node points = root["points"];
    if (!points || !points.IsSequence() || points.size() == 0) {
      throw std::runtime_error("YAML 'points' must be a non-empty sequence.");
    }

    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names;

    double last_t = -1.0;
    for (const auto& p : points) {
      double t = p["t"].as<double>();
      if (t < 0.0 || t < last_t) {
        throw std::runtime_error("Each point.t must be non-negative and non-decreasing.");
      }
      last_t = t;

      trajectory_msgs::msg::JointTrajectoryPoint jp;
      jp.positions = p["positions"].as<std::vector<double>>();

      if (p["velocities"]) jp.velocities = p["velocities"].as<std::vector<double>>();
      if (p["accelerations"]) jp.accelerations = p["accelerations"].as<std::vector<double>>();

      // time_from_start
      int32_t sec = static_cast<int32_t>(t);
      uint32_t nanosec = static_cast<uint32_t>((t - sec) * 1e9);
      jp.time_from_start.sec = sec;
      jp.time_from_start.nanosec = nanosec;

      goal.trajectory.points.push_back(jp);
    }

    RCLCPP_INFO(get_logger(), "Waiting for action server: %s", action_name_.c_str());
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      throw std::runtime_error("Action server not available. Check controller/action name.");
    }

    rclcpp_action::Client<FollowJT>::SendGoalOptions opts;
    opts.feedback_callback =
      [this](GoalHandleFollowJT::SharedPtr, const std::shared_ptr<const FollowJT::Feedback> /*fb*/) {
        // 필요하면 feedback 출력
      };

    opts.result_callback =
      [this](const GoalHandleFollowJT::WrappedResult & result) {
        const auto& res = result.result;
        RCLCPP_INFO(this->get_logger(), "Done. error_code=%d, error_string='%s'",
          res->error_code, res->error_string.c_str());
        rclcpp::shutdown();
      };

    RCLCPP_INFO(get_logger(), "Sending trajectory with %zu points...", goal.trajectory.points.size());
    auto future = client_->async_send_goal(goal, opts);

    // goal accept/reject는 future 완료 후 확인 가능
    // 여기서는 spin으로 진행
  }

private:
  std::string action_name_;
  rclcpp_action::Client<FollowJT>::SharedPtr client_;
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: send_traj_action <path.yaml> [action_name]\n";
    return 2;
  }

  std::string yaml_path = argv[1];
  std::string action_name = get_action_name(argc, argv);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajActionSender>(action_name);

  try {
    node->send_from_yaml(yaml_path);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  return 0;
}
