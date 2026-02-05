#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

static const char* SCRIPT_TOPIC = "/urscript_interface/script_command";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ur_script_sender_cpp");
  auto pub = node->create_publisher<std_msgs::msg::String>(SCRIPT_TOPIC, 10);

  std::string script =
    "def my_prog():\n"
    "  set_digital_out(1, True)\n"
    "  movej(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)\n"
    "  textmsg(\"motion finished\")\n"
    "end\n";

  std_msgs::msg::String msg;
  msg.data = script;

  // subscriber 매칭 시간을 조금 주는 게 안전
  rclcpp::sleep_for(std::chrono::milliseconds(200));
  pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "URScript published.");

  rclcpp::shutdown();
  return 0;
}
