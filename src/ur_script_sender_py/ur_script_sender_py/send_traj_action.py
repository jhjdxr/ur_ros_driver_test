import sys
import time
import yaml
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

DEFAULT_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def default_yaml_path():
    share = get_package_share_directory("ur_script_sender_py")
    return os.path.join(share, "paths", "demo_joint_path.yaml")


class TrajActionSender(Node):
    def __init__(self, action_name: str):
        super().__init__("ur_traj_action_sender")
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    def send_from_yaml(self, yaml_path: str):
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        joint_names = data["joint_names"]
        pts = data["points"]

        n = len(joint_names)
        for i, p in enumerate(pts):
            if len(p["positions"]) != n:
                raise ValueError(
                    f"points[{i}].positions length {len(p['positions'])} != joint_names length {n}"
                )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        last_t = -1.0
        for p in pts:
            t = float(p["t"])
            if t < 0 or t < last_t:
                raise ValueError("Each point.t must be non-negative and non-decreasing.")
            last_t = t

            jp = JointTrajectoryPoint()
            jp.positions = [float(x) for x in p["positions"]]

            if "velocities" in p:
                jp.velocities = [float(x) for x in p["velocities"]]
            if "accelerations" in p:
                jp.accelerations = [float(x) for x in p["accelerations"]]

            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            jp.time_from_start.sec = sec
            jp.time_from_start.nanosec = nanosec

            goal.trajectory.points.append(jp)

        self.get_logger().info(f"Waiting for action server: {DEFAULT_ACTION} ...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Action server not available. Check controller/action name.")

        time.sleep(0.1)

        self.get_logger().info(f"Sending {len(goal.trajectory.points)} points... yaml={yaml_path}")
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected by controller.")

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        res = result_future.result().result
        self.get_logger().info(f"Done. error_code={res.error_code}, error_string='{res.error_string}'")

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        t = fb.actual.time_from_start
        self.get_logger().info(f"feedback actual t={t.sec}.{t.nanosec:09d}")


def main():
    # 사용법:
    # 1) ros2 run ... send_traj_action               -> share/paths/demo_joint_path.yaml 사용
    # 2) ros2 run ... send_traj_action <yaml>        -> 지정 yaml 사용
    # 3) ros2 run ... send_traj_action <yaml> <action_name>
    if len(sys.argv) >= 2:
        yaml_path = sys.argv[1]
    else:
        yaml_path = default_yaml_path()

    action_name = sys.argv[2] if len(sys.argv) >= 3 else DEFAULT_ACTION

    # 만약 yaml_path가 "paths/xxx.yaml"처럼 상대경로면 현재 cwd 기준이라 깨질 수 있음.
    # 필요하면 여기서 절대경로로 강제할 수도 있음:
    # yaml_path = os.path.abspath(yaml_path)

    rclpy.init()
    node = TrajActionSender(action_name)
    try:
        node.send_from_yaml(yaml_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
