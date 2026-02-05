#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <yaml-cpp/yaml.h>

using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;
using JointTolerance = control_msgs::msg::JointTolerance;

static builtin_interfaces::msg::Duration to_duration(double sec_f) {
  builtin_interfaces::msg::Duration d;
  if (sec_f < 0.0) sec_f = 0.0;
  int32_t sec = static_cast<int32_t>(std::floor(sec_f));
  uint32_t nsec = static_cast<uint32_t>(std::llround((sec_f - sec) * 1e9));
  if (nsec >= 1000000000u) { sec += 1; nsec -= 1000000000u; }
  d.sec = sec;
  d.nanosec = nsec;
  return d;
}

static std::vector<JointTolerance> make_tolerances(const std::vector<std::string>& joints, double tol_rad) {
  std::vector<JointTolerance> out;
  out.reserve(joints.size());
  for (const auto& j : joints) {
    JointTolerance jt;
    jt.name = j;
    jt.position = tol_rad;
    jt.velocity = 0.0;
    jt.acceleration = 0.0;
    out.push_back(jt);
  }
  return out;
}

struct Options {
  std::string action_name = "/scaled_joint_trajectory_controller/follow_joint_trajectory";
  std::string controller_name = "scaled_joint_trajectory_controller";
  std::string yaml_path;

  bool activate = false;
  bool auto_start = false;

  double wait_joint_states = 2.0;

  bool has_path_tol = false;
  bool has_goal_tol = false;
  bool has_goal_time_tol = false;

  double path_tol = 0.0;
  double goal_tol = 0.0;
  double goal_time_tol = 0.0;
};

static std::string default_yaml_path() {
  auto share = ament_index_cpp::get_package_share_directory("ur_script_sender_cpp");
  return (std::filesystem::path(share) / "paths" / "demo_joint_path.yaml").string();
}

static Options parse_args(int argc, char** argv) {
  Options opt;
  opt.yaml_path = default_yaml_path();

  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];

    auto need_value = [&](const std::string& flag) {
      if (i + 1 >= argc) throw std::runtime_error("Missing value after " + flag);
      return std::string(argv[++i]);
    };

    if (a == "--file" || a == "-f") {
      opt.yaml_path = need_value(a);
    } else if (a == "--action") {
      opt.action_name = need_value(a);
    } else if (a == "--controller") {
      opt.controller_name = need_value(a);
    } else if (a == "--activate") {
      opt.activate = true;
    } else if (a == "--auto-start") {
      opt.auto_start = true;
    } else if (a == "--wait-joint-states") {
      opt.wait_joint_states = std::stod(need_value(a));
    } else if (a == "--path-tol") {
      opt.has_path_tol = true;
      opt.path_tol = std::stod(need_value(a));
    } else if (a == "--goal-tol") {
      opt.has_goal_tol = true;
      opt.goal_tol = std::stod(need_value(a));
    } else if (a == "--goal-time-tol") {
      opt.has_goal_time_tol = true;
      opt.goal_time_tol = std::stod(need_value(a));
    } else if (a == "--help" || a == "-h") {
      std::cout
        << "Usage:\n"
        << "  send_traj_action [--file PATH] [--action NAME] [--controller NAME]\n"
        << "                 [--activate] [--auto-start] [--wait-joint-states SEC]\n"
        << "                 [--path-tol RAD] [--goal-tol RAD] [--goal-time-tol SEC]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("Unknown arg: " + a);
    }
  }
  return opt;
}

static void validate_points(const std::vector<std::string>& joint_names, const YAML::Node& points) {
  const size_t n = joint_names.size();
  if (n == 0) throw std::runtime_error("joint_names is empty");

  double last_t = -1.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    if (!p["t"] || !p["positions"]) {
      throw std::runtime_error("points[" + std::to_string(i) + "] must have 't' and 'positions'");
    }
    double t = p["t"].as<double>();
    if (t < 0.0 || t < last_t) {
      throw std::runtime_error("Each point.t must be non-negative and non-decreasing.");
    }
    last_t = t;

    auto pos = p["positions"].as<std::vector<double>>();
    if (pos.size() != n) {
      throw std::runtime_error("points[" + std::to_string(i) + "].positions size mismatch");
    }
    for (double x : pos) {
      if (!std::isfinite(x)) throw std::runtime_error("points has NaN/inf");
    }
  }
}

class TrajActionSenderCpp : public rclcpp::Node {
public:
  TrajActionSenderCpp(const Options& opt, rclcpp::Executor& exec)
  : Node("ur_traj_action_sender_cpp"), opt_(opt), exec_(exec) {
    action_client_ = rclcpp_action::create_client<FollowJT>(this, opt_.action_name);
    list_cli_ = this->create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    switch_cli_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");
  }

  void run() {
    if (opt_.activate) {
      activate_controller(opt_.controller_name);
    }

    YAML::Node root = YAML::LoadFile(opt_.yaml_path);
    auto joint_names = root["joint_names"].as<std::vector<std::string>>();
    YAML::Node points = root["points"];
    if (!points || !points.IsSequence() || points.size() == 0) {
      throw std::runtime_error("YAML 'points' must be a non-empty sequence.");
    }
    validate_points(joint_names, points);

    if (opt_.auto_start) {
      auto cur = wait_joint_states(opt_.wait_joint_states);
      YAML::Node new_points(YAML::NodeType::Sequence);
      YAML::Node p0;
      p0["t"] = 1.0;

      YAML::Node pos0(YAML::NodeType::Sequence);
      for (const auto& j : joint_names) {
        auto it = cur.find(j);
        if (it == cur.end()) throw std::runtime_error("joint '" + j + "' missing in /joint_states");
        pos0.push_back(it->second);
      }
      p0["positions"] = pos0;
      new_points.push_back(p0);

      for (const auto& p : points) {
        YAML::Node cp = YAML::Clone(p);
        cp["t"] = cp["t"].as<double>() + 1.0;
        new_points.push_back(cp);
      }
      points = new_points;
      validate_points(joint_names, points);
    }

    FollowJT::Goal goal;
    goal.trajectory.joint_names = joint_names;
    for (const auto& p : points) {
      trajectory_msgs::msg::JointTrajectoryPoint jp;
      jp.positions = p["positions"].as<std::vector<double>>();
      if (p["velocities"]) jp.velocities = p["velocities"].as<std::vector<double>>();
      if (p["accelerations"]) jp.accelerations = p["accelerations"].as<std::vector<double>>();
      jp.time_from_start = to_duration(p["t"].as<double>());
      goal.trajectory.points.push_back(jp);
    }

    if (opt_.has_path_tol) goal.path_tolerance = make_tolerances(joint_names, opt_.path_tol);
    if (opt_.has_goal_tol) goal.goal_tolerance = make_tolerances(joint_names, opt_.goal_tol);
    if (opt_.has_goal_time_tol) goal.goal_time_tolerance = to_duration(opt_.goal_time_tol);

    RCLCPP_INFO(get_logger(), "Waiting for action server: %s", opt_.action_name.c_str());
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      throw std::runtime_error("Action server not available. Check controller/action name.");
    }

    rclcpp_action::Client<FollowJT>::SendGoalOptions sopts;
    sopts.feedback_callback =
      [this](GoalHandleFollowJT::SharedPtr, const std::shared_ptr<const FollowJT::Feedback> fb) {
        auto t = fb->actual.time_from_start;
        RCLCPP_INFO(this->get_logger(), "feedback actual t=%d.%09u", t.sec, t.nanosec);
      };

    sopts.goal_response_callback =
      [this](GoalHandleFollowJT::SharedPtr gh) {
        if (!gh) RCLCPP_ERROR(this->get_logger(), "Goal rejected (null goal handle).");
        else RCLCPP_INFO(this->get_logger(), "Goal accepted. Waiting for result...");
      };

    sopts.result_callback =
      [this](const GoalHandleFollowJT::WrappedResult& result) {
        // ✅ null 방어 + 종료 처리
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_ERROR(this->get_logger(), "ResultCode=%d (not SUCCEEDED)", static_cast<int>(result.code));
        }
        if (result.result) {
          RCLCPP_INFO(this->get_logger(), "Done. error_code=%d, error_string='%s'",
            result.result->error_code, result.result->error_string.c_str());
        } else {
          RCLCPP_ERROR(this->get_logger(), "Done. result.result is null");
        }
        rclcpp::shutdown();
      };

    RCLCPP_INFO(get_logger(), "Sending %zu points... yaml=%s",
      goal.trajectory.points.size(), opt_.yaml_path.c_str());

    action_client_->async_send_goal(goal, sopts);
  }

private:
  Options opt_;
  rclcpp::Executor& exec_;

  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_cli_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_cli_;

  std::unordered_map<std::string, double> last_joint_state_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;

  std::unordered_map<std::string, double> wait_joint_states(double timeout_sec) {
    last_joint_state_.clear();

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        if (m->name.size() != m->position.size()) return;
        for (size_t i = 0; i < m->name.size(); ++i) last_joint_state_[m->name[i]] = m->position[i];
      });

    auto t0 = std::chrono::steady_clock::now();
    while (rclcpp::ok() && last_joint_state_.empty()) {
      exec_.spin_some();
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_sec) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    js_sub_.reset();
    if (last_joint_state_.empty()) throw std::runtime_error("Failed to receive /joint_states.");
    return last_joint_state_;
  }

    void activate_controller(const std::string& controller_name) {
    using List = controller_manager_msgs::srv::ListControllers;
    using Switch = controller_manager_msgs::srv::SwitchController;

    if (!list_cli_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "list_controllers service not available.");
      return;
    }

    auto lf = list_cli_->async_send_request(std::make_shared<List::Request>());
    if (exec_.spin_until_future_complete(lf, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to call list_controllers.");
      return;
    }

    // ✅ 핵심: 응답을 깊게 복사해서 lifetime 문제 제거
    const auto resp = *lf.get();

    std::string state;
    bool found = false;
    for (const auto& c : resp.controller) {
      // 방어: 이름이 비어있거나 깨진 케이스 방지(정상적이면 필요 없지만 안전)
      if (c.name.empty()) continue;
      if (c.name == controller_name) {
        state = c.state;
        found = true;
        break;
      }
    }

    if (!found) {
      RCLCPP_WARN(get_logger(), "Controller '%s' not found in list_controllers response.", controller_name.c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "Controller '%s' state: %s", controller_name.c_str(), state.c_str());
    if (state == "active") return;

    if (!switch_cli_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "switch_controller service not available.");
      return;
    }

    auto req = std::make_shared<Switch::Request>();
    req->activate_controllers = {controller_name};
    req->strictness = Switch::Request::STRICT;
    req->start_asap = true;
    req->timeout = to_duration(2.0);

    auto sf = switch_cli_->async_send_request(req);
    if (exec_.spin_until_future_complete(sf, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "Failed to call switch_controller.");
      return;
    }

    const auto sresp = *sf.get();
    if (sresp.ok) RCLCPP_INFO(get_logger(), "Activated controller '%s'.", controller_name.c_str());
    else RCLCPP_WARN(get_logger(), "Failed to activate controller '%s' (ok=false).", controller_name.c_str());
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  Options opt;
  try {
    opt = parse_args(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Arg error: " << e.what() << "\nTry --help\n";
    return 2;
  }

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<TrajActionSenderCpp>(opt, exec);
  exec.add_node(node);

  try {
    node->run();
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  exec.spin();
  return 0;
}
