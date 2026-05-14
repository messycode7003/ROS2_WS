#!/usr/bin/env python3

import math
import threading
from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def _as_float_list(value, name: str) -> List[float]:
    if isinstance(value, (float, int)):
        value = [value]
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite numbers")
    return result


def _as_string_list(value, name: str) -> List[str]:
    if isinstance(value, str):
        value = [value]
    result = [str(item).strip() for item in value if str(item).strip()]
    if not result:
        raise ValueError(f"{name} must not be empty")
    return result


def _clamp(value: float, low: float, high: float) -> float:
    return min(max(value, low), high)


def _smoothstep(u: float) -> float:
    u = _clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


class ThreeMotorPositionCli(Node):
    def __init__(self) -> None:
        super().__init__("three_motor_position_cli")

        self.declare_parameter("command_topic", "/forward_position_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("use_joint_states", True)

        self.declare_parameter("joint_names", ["joint_3", "joint_5", "joint_8"])
        self.declare_parameter("angle_unit", "deg")

        self.declare_parameter("section_length", 0.3)
        self.declare_parameter("tendon_radius", 0.02)
        self.declare_parameter("tendon_angles_deg", [0.0, 240.0, 120.0])

        self.declare_parameter("command_units_per_meter", 1048576000.0)
        self.declare_parameter("zero_positions", [0.0, 0.0, 0.0])
        self.declare_parameter("motor_signs", [1.0, 1.0, 1.0])

        self.declare_parameter("max_abs_bend_angle_deg", 90.0)
        self.declare_parameter("max_abs_command", 2147483647.0)

        self.declare_parameter("publish_period_sec", 0.02)
        self.declare_parameter("min_motion_duration_sec", 1.0)
        self.declare_parameter("max_command_velocity", 200000.0)
        self.declare_parameter("max_command_acceleration", 0.0)
        self.declare_parameter("target_tolerance", 1.0)

        self.declare_parameter("interactive", True)
        self.declare_parameter("print_each_point", False)

        self.command_topic = str(self.get_parameter("command_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self.use_joint_states = bool(self.get_parameter("use_joint_states").value)
        self.joint_names = _as_string_list(self.get_parameter("joint_names").value, "joint_names")
        self.angle_unit = str(self.get_parameter("angle_unit").value).lower()

        self.section_length = float(self.get_parameter("section_length").value)
        self.tendon_radius = float(self.get_parameter("tendon_radius").value)
        self.tendon_angles = [
            math.radians(angle)
            for angle in _as_float_list(
                self.get_parameter("tendon_angles_deg").value, "tendon_angles_deg"
            )
        ]

        self.command_units_per_meter = float(
            self.get_parameter("command_units_per_meter").value
        )
        self.zero_positions = _as_float_list(
            self.get_parameter("zero_positions").value, "zero_positions"
        )
        self.motor_signs = _as_float_list(
            self.get_parameter("motor_signs").value, "motor_signs"
        )

        self.max_abs_bend_angle = math.radians(
            abs(float(self.get_parameter("max_abs_bend_angle_deg").value))
        )
        self.max_abs_command = abs(float(self.get_parameter("max_abs_command").value))
        self.publish_period_sec = float(self.get_parameter("publish_period_sec").value)
        self.min_motion_duration_sec = float(
            self.get_parameter("min_motion_duration_sec").value
        )
        self.max_command_velocity = abs(float(self.get_parameter("max_command_velocity").value))
        self.max_command_acceleration = abs(
            float(self.get_parameter("max_command_acceleration").value)
        )
        self.target_tolerance = abs(float(self.get_parameter("target_tolerance").value))
        self.interactive = bool(self.get_parameter("interactive").value)
        self.print_each_point = bool(self.get_parameter("print_each_point").value)

        self._validate_parameters()

        self.command_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)
        self.motor_target_pub = self.create_publisher(Float64MultiArray, "/pcc_motor_targets", 10)
        self.interpolated_point_pub = self.create_publisher(
            Float64MultiArray, "/pcc_interpolated_points", 10
        )

        if self.use_joint_states:
            self.create_subscription(
                JointState, self.joint_states_topic, self._joint_states_callback, 10
            )

        self._lock = threading.Lock()
        self._current_positions = list(self.zero_positions)
        self._start_positions = list(self.zero_positions)
        self._target_positions = list(self.zero_positions)
        self._trajectory_elapsed = 0.0
        self._trajectory_duration = 0.0
        self._trajectory_active = False

        self.create_timer(self.publish_period_sec, self._timer_callback)

        self.get_logger().info(
            "Three motor CC CLI ready. Input: phi bend_angle [duration_sec]."
        )

    def _validate_parameters(self) -> None:
        if len(self.joint_names) != 3:
            raise ValueError("joint_names must contain exactly 3 names")
        if len(self.tendon_angles) != 3:
            raise ValueError("tendon_angles_deg must contain exactly 3 angles")
        if len(self.zero_positions) != 3:
            raise ValueError("zero_positions must contain exactly 3 values")
        if len(self.motor_signs) != 3:
            raise ValueError("motor_signs must contain exactly 3 values")
        if self.angle_unit not in {"rad", "radian", "radians", "deg", "degree", "degrees"}:
            raise ValueError("angle_unit must be 'rad' or 'deg'")
        if self.section_length <= 0.0:
            raise ValueError("section_length must be positive")
        if self.tendon_radius <= 0.0:
            raise ValueError("tendon_radius must be positive")
        if self.command_units_per_meter == 0.0:
            raise ValueError("command_units_per_meter must not be zero")
        if self.publish_period_sec <= 0.0:
            raise ValueError("publish_period_sec must be positive")
        if self.min_motion_duration_sec < self.publish_period_sec:
            raise ValueError("min_motion_duration_sec must be >= publish_period_sec")
        if self.max_command_velocity <= 0.0:
            raise ValueError("max_command_velocity must be positive")

    def cc_to_motor_targets(self, phi: float, bend_angle: float) -> List[float]:
        if self.angle_unit in {"deg", "degree", "degrees"}:
            phi = math.radians(phi)
            bend_angle = math.radians(bend_angle)

        bend_angle = _clamp(
            bend_angle,
            -self.max_abs_bend_angle,
            self.max_abs_bend_angle,
        )

        targets = []
        length_deltas_mm = []

        for i, tendon_angle in enumerate(self.tendon_angles):
            tendon_delta_m = -self.tendon_radius * bend_angle * math.cos(tendon_angle - phi)
            tendon_delta_mm = tendon_delta_m * 1000.0
            command_delta = tendon_delta_m * self.command_units_per_meter

            target = self.zero_positions[i] + self.motor_signs[i] * command_delta
            target = _clamp(target, -self.max_abs_command, self.max_abs_command)

            targets.append(target)
            length_deltas_mm.append(tendon_delta_mm)

        curvature = bend_angle / self.section_length
        self.get_logger().info(
            "phi=%.3f %s, bend=%.3f %s, curvature=%.6f 1/m, length_delta_mm=[%s]"
            % (
                math.degrees(phi) if self.angle_unit.startswith("deg") else phi,
                self.angle_unit,
                math.degrees(bend_angle) if self.angle_unit.startswith("deg") else bend_angle,
                self.angle_unit,
                curvature,
                ", ".join(f"{item:.6f}" for item in length_deltas_mm),
            )
        )

        return targets

    def accept_command(
        self, phi: float, bend_angle: float, duration_override: Optional[float] = None
    ) -> List[float]:
        targets = self.cc_to_motor_targets(phi, bend_angle)
        duration = self._compute_duration(targets, duration_override)

        with self._lock:
            self._start_positions = list(self._current_positions)
            self._target_positions = list(targets)
            self._trajectory_elapsed = 0.0
            self._trajectory_duration = duration
            self._trajectory_active = True

        self._publish_list(self.motor_target_pub, targets)
        self.get_logger().info(
            "targets %s -> [%s], duration %.3f s"
            % (
                ", ".join(self.joint_names),
                ", ".join(f"{item:.3f}" for item in targets),
                duration,
            )
        )
        return targets

    def _compute_duration(
        self, targets: List[float], duration_override: Optional[float] = None
    ) -> float:
        if duration_override is not None and duration_override > 0.0:
            return max(duration_override, self.publish_period_sec)

        with self._lock:
            start = list(self._current_positions)

        max_delta = max(abs(target - current) for target, current in zip(targets, start))
        if max_delta <= self.target_tolerance:
            return self.publish_period_sec

        duration = max(self.min_motion_duration_sec, 1.5 * max_delta / self.max_command_velocity)

        if self.max_command_acceleration > 0.0:
            duration = max(duration, math.sqrt(6.0 * max_delta / self.max_command_acceleration))

        steps = max(1, math.ceil(duration / self.publish_period_sec))
        return steps * self.publish_period_sec

    def _joint_states_callback(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in name_to_index for name in self.joint_names):
            return

        positions = [float(msg.position[name_to_index[name]]) for name in self.joint_names]
        if not all(math.isfinite(position) for position in positions):
            return

        with self._lock:
            if not self._trajectory_active:
                self._current_positions = positions

    def _timer_callback(self) -> None:
        with self._lock:
            if not self._trajectory_active:
                return

            self._trajectory_elapsed += self.publish_period_sec
            ratio = self._trajectory_elapsed / self._trajectory_duration
            blend = _smoothstep(ratio)

            point = [
                start + (target - start) * blend
                for start, target in zip(self._start_positions, self._target_positions)
            ]

            if ratio >= 1.0:
                point = list(self._target_positions)
                self._trajectory_active = False

            self._current_positions = list(point)

        self._publish_list(self.command_pub, point)
        self._publish_list(self.interpolated_point_pub, point)

        if self.print_each_point:
            self.get_logger().info("point [%s]" % ", ".join(f"{item:.3f}" for item in point))

    @staticmethod
    def _publish_list(publisher, values: List[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        publisher.publish(msg)

    def run_interactive(self) -> None:
        print("Input: phi bend_angle [duration_sec]. Type q to quit.")
        print("Example: 0 5")
        print("Order: joint_3=0 deg, joint_5=240 deg, joint_8=120 deg")
        print(f"Angle unit: {self.angle_unit}")

        while rclpy.ok():
            try:
                line = input("phi bend_angle > ").strip()
            except EOFError:
                break

            if line.lower() in {"q", "quit", "exit"}:
                break
            if not line:
                continue

            parts = line.replace(",", " ").split()
            if len(parts) not in {2, 3}:
                print("Please enter: phi bend_angle [duration_sec]")
                continue

            try:
                phi = float(parts[0])
                bend_angle = float(parts[1])
                duration = float(parts[2]) if len(parts) == 3 else None
                self.accept_command(phi, bend_angle, duration)
            except ValueError as exc:
                print(exc)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    spin_thread = None

    try:
        node = ThreeMotorPositionCli()
        if node.interactive:
            executor = MultiThreadedExecutor()
            executor.add_node(node)
            spin_thread = threading.Thread(target=executor.spin, daemon=True)
            spin_thread.start()
            node.run_interactive()
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if spin_thread is not None:
            spin_thread.join(timeout=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

