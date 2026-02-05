#!/usr/bin/env python3
import sys
import time
import yaml
import os
import argparse
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

from ament_index_python.packages import get_package_share_directory

DEFAULT_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
DEFAULT_CONTROLLER = "scaled_joint_trajectory_controller"


def default_yaml_path() -> str:
    share = get_package_share_directory("ur_script_sender_py")
    return os.path.join(share, "paths", "demo_joint_path.yaml")


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="send_traj_action",
        description="Send JointTrajectory from YAML via FollowJointTrajectory action."
    )
    p.add_argument("--file", "-f", default=None,
                   help="Path to YAML trajectory. If omitted, uses package share paths/demo_joint_path.yaml.")
    p.add_argument("--action", default=DEFAULT_ACTION,
                   help=f"Action name. Default: {DEFAULT_ACTION}")
    p.add_argument("--controller", default=DEFAULT_CONTROLLER,
                   help=f"Controller name for activation. Default: {DEFAULT_CONTROLLER}")
    p.add_argument("--activate", action="store_true",
                   help="Try to activate controller via controller_manager services before sending.")
    p.add_argument("--auto-start", action="store_true",
                   help="Insert first point at t=1.0 using current /joint_states (safer start).")
    p.add_argument("--wait-joint-states", type=float, default=2.0,
                   help="Seconds to wait for /joint_states when needed. Default: 2.0")

    # tolerance options
    p.add_argument("--path-tol", type=float, default=None,
                   help="Path tolerance (radians) for all joints. Example: 0.2")
    p.add_argument("--goal-tol", type=float, default=None,
                   help="Goal tolerance (radians) for all joints. Example: 0.05")
    p.add_argument("--goal-time-tol", type=float, default=None,
                   help="Goal time tolerance (seconds). Example: 2.0")

    return p.parse_args(argv)


class TrajActionSender(Node):
    def __init__(self, action_name: str):
        super().__init__("ur_traj_action_sender")
        self._client = ActionClient(self, FollowJointTrajectory, action_name)

    # -------- joint states helpers --------
    def _wait_for_one_message(self, msg_type, topic: str, timeout_sec: float):
        holder = {"msg": None}
        sub = self.create_subscription(msg_type, topic, lambda m: holder.__setitem__("msg", m), 10)

        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < timeout_sec and holder["msg"] is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.destroy_subscription(sub)
        return holder["msg"]

    def _get_current_joint_positions(self, timeout_sec: float) -> Dict[str, float]:
        msg = self._wait_for_one_message(JointState, "/joint_states", timeout_sec)
        if msg is None:
            raise RuntimeError("Failed to receive /joint_states. Is the driver running?")
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            raise RuntimeError("Invalid /joint_states message.")
        return {n: float(p) for n, p in zip(msg.name, msg.position)}

    # -------- controller activation (optional) --------
    def try_activate_controller(self, controller_name: str) -> None:
        from controller_manager_msgs.srv import SwitchController, ListControllers  # type: ignore

        list_cli = self.create_client(ListControllers, "/controller_manager/list_controllers")
        if not list_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("controller_manager list_controllers service not available.")
            return

        fut = list_cli.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is None:
            self.get_logger().warn("Failed to call list_controllers.")
            return

        state_map = {c.name: c.state for c in fut.result().controller}
        state = state_map.get(controller_name, None)
        self.get_logger().info(f"Controller '{controller_name}' state: {state}")
        if state == "active":
            return

        sw_cli = self.create_client(SwitchController, "/controller_manager/switch_controller")
        if not sw_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("controller_manager switch_controller service not available.")
            return

        req = SwitchController.Request()
        req.activate_controllers = [controller_name]
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.STRICT
        req.start_asap = True
        req.timeout.sec = 2
        req.timeout.nanosec = 0

        fut2 = sw_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut2, timeout_sec=3.0)
        if fut2.result() is None:
            self.get_logger().warn("Failed to call switch_controller.")
            return

        if fut2.result().ok:
            self.get_logger().info(f"Activated controller '{controller_name}'.")
        else:
            self.get_logger().warn(f"Failed to activate controller '{controller_name}' (ok=false).")

    # -------- YAML -> goal --------
    @staticmethod
    def _load_yaml(yaml_path: str) -> Tuple[List[str], List[dict]]:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        return data["joint_names"], data["points"]

    @staticmethod
    def _validate_points(joint_names: List[str], pts: List[dict]) -> None:
        n = len(joint_names)
        if n == 0:
            raise ValueError("joint_names is empty.")
        if not pts:
            raise ValueError("points is empty.")

        last_t = -1.0
        for i, p in enumerate(pts):
            if "t" not in p or "positions" not in p:
                raise ValueError(f"points[{i}] must contain 't' and 'positions'.")
            t = float(p["t"])
            if t < 0 or t < last_t:
                raise ValueError("Each point.t must be non-negative and non-decreasing.")
            last_t = t

            if len(p["positions"]) != n:
                raise ValueError(f"points[{i}].positions length {len(p['positions'])} != joint_names length {n}")

    @staticmethod
    def _duration_from_seconds(sec_f: float) -> Duration:
        d = Duration()
        sec = int(sec_f)
        nanosec = int((sec_f - sec) * 1e9)
        d.sec = sec
        d.nanosec = nanosec
        return d

    @staticmethod
    def _make_joint_tolerances(joint_names: List[str], tol: float) -> List[JointTolerance]:
        out = []
        for jn in joint_names:
            jt = JointTolerance()
            jt.name = jn
            jt.position = float(tol)
            jt.velocity = 0.0
            jt.acceleration = 0.0
            out.append(jt)
        return out

    def _build_goal(
        self,
        joint_names: List[str],
        pts: List[dict],
        auto_start: bool,
        wait_joint_states: float,
        path_tol: Optional[float],
        goal_tol: Optional[float],
        goal_time_tol: Optional[float],
    ) -> FollowJointTrajectory.Goal:

        # auto-start: insert a safe first point at t=1.0 (and shift others by +1.0)
        if auto_start:
            cur = self._get_current_joint_positions(timeout_sec=wait_joint_states)
            missing = [jn for jn in joint_names if jn not in cur]
            if missing:
                raise RuntimeError(f"/joint_states missing joints: {missing}")

            start_positions = [cur[jn] for jn in joint_names]
            new_pts = [{"t": 1.0, "positions": start_positions}]
            for p in pts:
                cp = dict(p)
                cp["t"] = float(cp["t"]) + 1.0
                new_pts.append(cp)
            pts = new_pts

        self._validate_points(joint_names, pts)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        for p in pts:
            t = float(p["t"])
            jp = JointTrajectoryPoint()
            jp.positions = [float(x) for x in p["positions"]]

            if "velocities" in p:
                jp.velocities = [float(x) for x in p["velocities"]]
            if "accelerations" in p:
                jp.accelerations = [float(x) for x in p["accelerations"]]

            jp.time_from_start = self._duration_from_seconds(t)
            goal.trajectory.points.append(jp)

        # tolerances
        if path_tol is not None:
            goal.path_tolerance = self._make_joint_tolerances(joint_names, path_tol)
        if goal_tol is not None:
            goal.goal_tolerance = self._make_joint_tolerances(joint_names, goal_tol)
        if goal_time_tol is not None:
            goal.goal_time_tolerance = self._duration_from_seconds(goal_time_tol)

        return goal

    # -------- action send --------
    def send_goal_and_wait(self, goal: FollowJointTrajectory.Goal, yaml_path: str) -> None:
        self.get_logger().info(f"Waiting for action server: {self._client._action_name} ...")
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Action server not available. Check controller/action name.")

        time.sleep(0.1)

        self.get_logger().info(
            f"Sending {len(goal.trajectory.points)} points... yaml={yaml_path}"
        )
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None:
            raise RuntimeError("Failed to get goal_handle.")
        if not goal_handle.accepted:
            raise RuntimeError("Goal rejected by controller.")

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        wrapped = result_future.result()
        if wrapped is None or wrapped.result is None:
            raise RuntimeError("No result received (controller/driver may be stuck).")

        res = wrapped.result
        self.get_logger().info(f"Done. error_code={res.error_code}, error_string='{res.error_string}'")

    def _feedback_cb(self, fb_msg):
        fb = fb_msg.feedback
        t = fb.actual.time_from_start
        self.get_logger().info(f"feedback actual t={t.sec}.{t.nanosec:09d}")


def main():
    args = parse_args(sys.argv[1:])

    yaml_path = args.file if args.file else default_yaml_path()

    rclpy.init()
    node = TrajActionSender(args.action)

    try:
        if args.activate:
            node.try_activate_controller(args.controller)

        joint_names, pts = node._load_yaml(yaml_path)
        goal = node._build_goal(
            joint_names=joint_names,
            pts=pts,
            auto_start=args.auto_start,
            wait_joint_states=args.wait_joint_states,
            path_tol=args.path_tol,
            goal_tol=args.goal_tol,
            goal_time_tol=args.goal_time_tol,
        )
        node.send_goal_and_wait(goal, yaml_path)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
