#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, TwistStamped, Vector3Stamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


class CTCLikeController(Node):

    def __init__(self):
        super().__init__("ctc_like_controller")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.target_topic = str(
            self.declare_parameter("target_topic", "/desired_position").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/controller/delta_twist_cmds").value
        )

        self.base_frame = str(
            self.declare_parameter("base_frame", "link_base").value
        )
        self.ee_frame = str(
            self.declare_parameter("ee_frame", "link_eef").value
        )

        # Controller gains (per axis)
        kp = self.declare_parameter("kp", [10.0, 10.0, 10.0]).value
        kd = self.declare_parameter("kd", [5.0, 5.0, 5.0]).value

        # Virtual mass diagonal (kg-ish virtual scaling, per axis)
        mv = self.declare_parameter("virtual_mass", [1.0, 1.0, 1.0]).value

        self.kp = np.array(kp, dtype=float)
        self.kd = np.array(kd, dtype=float)
        self.virtual_mass = np.array(mv, dtype=float)

        # Safety / smoothing
        self.max_speed = float(self.declare_parameter("max_speed", 0.18).value)
        self.max_accel = float(self.declare_parameter("max_accel", 0.50).value)
        self.deadband = float(self.declare_parameter("deadband", 0.003).value)

        # Velocity estimation smoothing
        self.vel_alpha = float(self.declare_parameter("vel_alpha", 0.20).value)

        # Control loop rate
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 50.0).value)

        # -----------------------------
        # ROS I/O
        # -----------------------------
        self.desired_sub = self.create_subscription(
            PointStamped, self.target_topic, self.desired_cb, 10
        )

        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_cb, 10
        )

        self.twist_pub = self.create_publisher(TwistStamped, self.output_topic, 10)
        self.actual_pub = self.create_publisher(PointStamped, "/actual_position", 10)
        self.error_pub = self.create_publisher(Vector3Stamped, "/position_error", 10)
        self.speed_pub = self.create_publisher(Vector3Stamped, "/commanded_speed", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.loop)

        # -----------------------------
        # State
        # -----------------------------
        self.latest_joint_state = None

        self.desired_pos = None
        self.desired_vel = np.zeros(3)
        self.desired_acc = np.zeros(3)

        self.prev_desired_pos = None
        self.prev_desired_vel = np.zeros(3)
        self.prev_desired_time = None

        self.prev_actual_pos = None
        self.actual_vel = np.zeros(3)
        self.prev_loop_time = None

        self.prev_cmd_vel = np.zeros(3)

        self.last_tf_warn_time = self.get_clock().now()
        self.last_info_time = self.get_clock().now()

        self.get_logger().info("✅ ctc_like_controller iniciado")
        self.get_logger().info(f"target_topic={self.target_topic}")
        self.get_logger().info(f"output_topic={self.output_topic}")
        self.get_logger().info(f"kp={self.kp.tolist()} kd={self.kd.tolist()} virtual_mass={self.virtual_mass.tolist()}")

    # --------------------------------
    # Callbacks
    # --------------------------------
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = msg

    def desired_cb(self, msg: PointStamped):
        now = self.get_clock().now()
        p = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)

        # Finite-difference desired derivatives
        if self.prev_desired_pos is not None and self.prev_desired_time is not None:
            dt = (now - self.prev_desired_time).nanoseconds / 1e9
            if dt > 1e-6:
                vel = (p - self.prev_desired_pos) / dt
                acc = (vel - self.prev_desired_vel) / dt
                self.desired_vel = vel
                self.desired_acc = acc
                self.prev_desired_vel = vel

        self.desired_pos = p
        self.prev_desired_pos = p
        self.prev_desired_time = now

    # --------------------------------
    # Helpers
    # --------------------------------
    def read_actual_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time()
            )
            p = np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ],
                dtype=float,
            )
            return p
        except Exception as e:
            now = self.get_clock().now()
            if (now - self.last_tf_warn_time).nanoseconds > 2e9:
                self.get_logger().warn(f"TF not ready: {e}")
                self.last_tf_warn_time = now
            return None

    def publish_actual_position(self, p):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.point.x = float(p[0])
        msg.point.y = float(p[1])
        msg.point.z = float(p[2])
        self.actual_pub.publish(msg)

    def publish_error(self, e):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.vector.x = float(e[0])
        msg.vector.y = float(e[1])
        msg.vector.z = float(e[2])
        self.error_pub.publish(msg)

    def publish_speed(self, v):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.vector.x = float(v[0])
        msg.vector.y = float(v[1])
        msg.vector.z = float(v[2])
        self.speed_pub.publish(msg)

    def publish_twist(self, v):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.twist.linear.x = float(v[0])
        msg.twist.linear.y = float(v[1])
        msg.twist.linear.z = float(v[2])
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

    # --------------------------------
    # Main loop
    # --------------------------------
    def loop(self):
        if self.desired_pos is None:
            return

        actual_pos = self.read_actual_position()
        if actual_pos is None:
            return

        now = self.get_clock().now()

        # dt
        if self.prev_loop_time is None:
            self.prev_loop_time = now
            self.prev_actual_pos = actual_pos.copy()
            self.publish_actual_position(actual_pos)
            self.publish_error(np.zeros(3))
            self.publish_speed(np.zeros(3))
            return

        dt = (now - self.prev_loop_time).nanoseconds / 1e9
        if dt <= 1e-6:
            dt = 1e-6

        # Actual velocity estimate with low-pass
        raw_actual_vel = (actual_pos - self.prev_actual_pos) / dt
        self.actual_vel = (
            self.vel_alpha * raw_actual_vel
            + (1.0 - self.vel_alpha) * self.actual_vel
        )

        # Errors
        e = self.desired_pos - actual_pos
        e = np.where(np.abs(e) < self.deadband, 0.0, e)
        e_dot = self.desired_vel - self.actual_vel

        # CTC-like computed acceleration in Cartesian space
        # a_cmd = pdd_des + Mv^{-1} (Kp*e + Kd*e_dot)
        fb = (self.kp * e + self.kd * e_dot) / self.virtual_mass
        a_cmd = self.desired_acc + fb

        # Integrate acceleration to velocity command
        v_cmd = self.desired_vel + a_cmd * dt

        # Acceleration clamp (limit change in command)
        dv = v_cmd - self.prev_cmd_vel
        dv_norm = np.linalg.norm(dv)
        max_dv = self.max_accel * dt
        if dv_norm > max_dv and dv_norm > 1e-9:
            dv = dv * (max_dv / dv_norm)
        v_cmd = self.prev_cmd_vel + dv

        # Speed clamp
        speed = np.linalg.norm(v_cmd)
        if speed > self.max_speed and speed > 1e-9:
            v_cmd = v_cmd * (self.max_speed / speed)

        # Publish
        self.publish_actual_position(actual_pos)
        self.publish_error(e)
        self.publish_speed(v_cmd)
        self.publish_twist(v_cmd)

        # 1 Hz debug
        if (now - self.last_info_time).nanoseconds > 1e9:
            self.get_logger().info(
                f"cur={np.round(actual_pos,3)} "
                f"des={np.round(self.desired_pos,3)} "
                f"e={np.round(e,3)} "
                f"v={np.round(v_cmd,3)}"
            )
            self.last_info_time = now

        # Update state
        self.prev_actual_pos = actual_pos.copy()
        self.prev_loop_time = now
        self.prev_cmd_vel = v_cmd.copy()


def main(args=None):
    rclpy.init(args=args)
    node = CTCLikeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por usuario.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
