import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

SCRIPT_TOPIC = "/urscript_interface/script_command"

class UrScriptSender(Node):
    def __init__(self):
        super().__init__("ur_script_sender")
        self.pub = self.create_publisher(String, SCRIPT_TOPIC, 10)

    def send_once(self):
        script = (
            'def my_prog():\n'
            '  set_digital_out(1, True)\n'
            '  movej(p[0.2, 0.3, 0.8, 0, 0, 3.14], a=1.2, v=0.25, r=0)\n'
            '  textmsg("motion finished")\n'
            'end\n'
        )
        msg = String()
        msg.data = script
        self.pub.publish(msg)
        self.get_logger().info("URScript published.")

def main():
    rclpy.init()
    node = UrScriptSender()

    # subscriber 매칭 시간 조금 주기 (안전)
    time.sleep(0.2)
    node.send_once()

    # publish가 wire로 나갈 시간 약간
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
