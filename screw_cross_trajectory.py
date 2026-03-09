#!/usr/bin/env python3
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from pynput import keyboard


class RobotState(Enum):
    RUNNING = 1
    PAUSED = 2
    HOME = 3


@dataclass
class Waypoint:
    name: str
    xyz: np.ndarray
    dwell: float
    transition: float


def quintic_blend(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


class ScrewCrossTrajectory(Node):
    def __init__(self):
        super().__init__("screw_cross_trajectory")
        self.get_logger().error("### VERSION NUEVA CON PERTURBATION_ENABLE ###")

        self.robot_state = RobotState.RUNNING

        # ===================== EDITAR AQUÍ =====================
        self.desired_topic = str(self.declare_parameter("desired_topic", "/desired_position").value)

        self.feeder_dx = float(self.declare_parameter("feeder_dx", 0.08).value)
        self.feeder_dy = float(self.declare_parameter("feeder_dy", 0.00).value)
        self.cross_offset = float(self.declare_parameter("cross_offset", 0.05).value)

        self.z_high = float(self.declare_parameter("z_high", 0.26).value)
        self.z_cross = float(self.declare_parameter("z_cross", 0.18).value)
        self.z_dip = float(self.declare_parameter("z_dip", 0.165).value)

        self.dwell_high = float(self.declare_parameter("dwell_high", 1.0).value)
        self.dwell_cross = float(self.declare_parameter("dwell_cross", 1.0).value)
        self.dwell_dip = float(self.declare_parameter("dwell_dip", 1.8).value)

        self.trans_long = float(self.declare_parameter("trans_long", 1.5).value)
        self.trans_short = float(self.declare_parameter("trans_short", 0.8).value)
        self.trans_dip = float(self.declare_parameter("trans_dip", 0.9).value)

        self.loop_trajectory = bool(self.declare_parameter("loop_trajectory", False).value)
        self.frame_id = str(self.declare_parameter("frame_id", "link_base").value)

        home = self.declare_parameter("home_position", [0.227, 0.00, 0.468]).value
        self.home_position = np.array([float(home[0]), float(home[1]), float(home[2])], dtype=float)
        # =================== FIN EDITAR AQUÍ ===================

        self.desired_pub = self.create_publisher(PointStamped, self.desired_topic, 10)
        self.pert_enable_pub = self.create_publisher(Bool, "/perturbation_enable", 10)
        self.phase_pub = self.create_publisher(String, "/task_phase", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.center = None
        self.waypoints: List[Waypoint] = []

        self.segment_index = 0
        self.segment_start_time = None
        self.in_dwell = True
        self.finished = False

        self.last_tf_warn_time = self.get_clock().now()

        self._start_keyboard()
        self.timer = self.create_timer(0.02, self._loop)

        self.get_logger().info("✅ screw_cross_trajectory iniciado")
        self.get_logger().info("Keys: 'p' pause/resume, 'h' home")

    def _start_keyboard(self):
        def on_press(key):
            if hasattr(key, "char") and key.char == "p":
                if self.robot_state == RobotState.RUNNING:
                    self.robot_state = RobotState.PAUSED
                    self.get_logger().warn("Paused.")
                elif self.robot_state == RobotState.PAUSED:
                    self.robot_state = RobotState.RUNNING
                    self.segment_start_time = self.get_clock().now()
                    self.get_logger().info("Resumed.")

            if hasattr(key, "char") and key.char == "h":
                self.robot_state = RobotState.HOME
                self.get_logger().info("Going HOME...")

        self.keyboard_listener = keyboard.Listener(on_press=on_press)
        self.keyboard_listener.start()

    def _read_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform("link_base", "link_eef", rclpy.time.Time())
            return np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=float,
            )
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_tf_warn_time = now
            return None

    def _publish_desired(self, target_pos: np.ndarray):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.point.x = float(target_pos[0])
        msg.point.y = float(target_pos[1])
        msg.point.z = float(target_pos[2])
        self.desired_pub.publish(msg)

    def _publish_perturbation_flag(self, enabled: bool, phase: str):
        b = Bool()
        b.data = bool(enabled)
        self.pert_enable_pub.publish(b)

        s = String()
        s.data = phase
        self.phase_pub.publish(s)

    def _compute_perturbation_enable(self):
        if self.finished or not self.waypoints:
            return False, "finished"

        current_name = self.waypoints[self.segment_index].name

        if self.in_dwell:
            if "target_dip_place_" in current_name:
                return True, "place_dwell"
            return False, f"dwell:{current_name}"

        next_name = self.waypoints[self.segment_index + 1].name

        if ("target_cross_" in current_name) and ("target_dip_place_" in next_name):
            return True, "place_down"

        if ("target_dip_place_" in current_name) and ("target_cross_after_place_" in next_name):
            return True, "place_up"

        return False, f"transition:{current_name}->{next_name}"

    def _build_waypoints(self):
        cx, cy, _ = self.center

        center_high = np.array([cx, cy, self.z_high], dtype=float)
        feeder_cross = np.array([cx + self.feeder_dx, cy + self.feeder_dy, self.z_cross], dtype=float)
        feeder_dip = np.array([cx + self.feeder_dx, cy + self.feeder_dy, self.z_dip], dtype=float)

        targets = [
            ("+X",
             np.array([cx + self.cross_offset, cy, self.z_cross], dtype=float),
             np.array([cx + self.cross_offset, cy, self.z_dip], dtype=float)),
            ("+Y",
             np.array([cx, cy + self.cross_offset, self.z_cross], dtype=float),
             np.array([cx, cy + self.cross_offset, self.z_dip], dtype=float)),
            ("-X",
             np.array([cx - self.cross_offset, cy, self.z_cross], dtype=float),
             np.array([cx - self.cross_offset, cy, self.z_dip], dtype=float)),
            ("-Y",
             np.array([cx, cy - self.cross_offset, self.z_cross], dtype=float),
             np.array([cx, cy - self.cross_offset, self.z_dip], dtype=float)),
        ]

        wps: List[Waypoint] = []

        def add(name, xyz, dwell, transition):
            wps.append(Waypoint(name=name, xyz=xyz.copy(), dwell=dwell, transition=transition))

        add("start_center_high", center_high, self.dwell_high, self.trans_long)

        for label, target_cross, target_dip in targets:
            add(f"feeder_cross_before_pick_{label}", feeder_cross, self.dwell_cross, self.trans_dip)
            add(f"feeder_dip_pick_{label}", feeder_dip, self.dwell_dip, self.trans_dip)
            add(f"feeder_cross_after_pick_{label}", feeder_cross, self.dwell_cross, self.trans_long)

            add(f"center_high_after_pick_{label}", center_high, self.dwell_high, self.trans_long)

            add(f"target_cross_{label}", target_cross, self.dwell_cross, self.trans_dip)
            add(f"target_dip_place_{label}", target_dip, self.dwell_dip, self.trans_dip)
            add(f"target_cross_after_place_{label}", target_cross, self.dwell_cross, self.trans_long)

            add(f"center_high_after_place_{label}", center_high, self.dwell_high, self.trans_long)

        add("final_center_high", center_high, self.dwell_high, 0.0)

        self.waypoints = wps

        self.get_logger().info("✅ Waypoints cargados:")
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(
                f"{i+1:02d}. {wp.name}: {np.round(wp.xyz, 3)} | "
                f"dwell={wp.dwell:.2f}s transition={wp.transition:.2f}s"
            )

    def _loop(self):
        current = self._read_pose()
        if current is None:
            return

        if self.center is None:
            self.center = current.copy()
            self.center[2] = self.z_high
            self._build_waypoints()
            self.segment_index = 0
            self.segment_start_time = self.get_clock().now()
            self.in_dwell = True
            self.get_logger().info(f"✅ Centro inicial fijado en {np.round(self.center, 3)}")
            return

        if self.robot_state == RobotState.PAUSED:
            if 0 <= self.segment_index < len(self.waypoints):
                self._publish_desired(self.waypoints[self.segment_index].xyz)
            self._publish_perturbation_flag(False, "paused")
            return

        if self.robot_state == RobotState.HOME:
            self._publish_desired(self.home_position)
            self._publish_perturbation_flag(False, "home")
            return

        if self.finished:
            if self.loop_trajectory:
                self.segment_index = 0
                self.segment_start_time = self.get_clock().now()
                self.in_dwell = True
                self.finished = False
            else:
                self._publish_desired(self.waypoints[-1].xyz)
                self._publish_perturbation_flag(False, "finished")
                return

        enabled, phase = self._compute_perturbation_enable()
        self._publish_perturbation_flag(enabled, phase)

        now = self.get_clock().now()
        t = (now - self.segment_start_time).nanoseconds / 1e9

        current_wp = self.waypoints[self.segment_index]

        if self.in_dwell:
            self._publish_desired(current_wp.xyz)

            if t >= current_wp.dwell:
                if self.segment_index == len(self.waypoints) - 1:
                    self.get_logger().info("✅ Trayectoria completada.")
                    self.finished = True
                    return

                self.in_dwell = False
                self.segment_start_time = self.get_clock().now()
            return

        next_wp = self.waypoints[self.segment_index + 1]
        T = max(current_wp.transition, 1e-6)

        s = min(t / T, 1.0)
        b = quintic_blend(s)
        target = current_wp.xyz + (next_wp.xyz - current_wp.xyz) * b

        self._publish_desired(target)

        if t >= T:
            self.segment_index += 1
            self.segment_start_time = self.get_clock().now()
            self.in_dwell = True


def main(args=None):
    rclpy.init(args=args)
    node = ScrewCrossTrajectory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
