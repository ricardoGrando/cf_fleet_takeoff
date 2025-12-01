#!/usr/bin/env python3
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


class FleetTakeoffNode(Node):
    """
    Simple P controller per UAV:
      v_z = clamp(kp * (z_target - z_meas), -vz_max, vz_max)
    Publishes zero once UAV is within tolerance for `stable_time` seconds.
    """

    def __init__(self):
        super().__init__('cf_fleet_takeoff')

        # Tunables / parameters
        self.declare_parameter('uav_names', ['cf1', 'cf2'])
        self.declare_parameter('target_altitude', 1.0)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('vz_max', 0.7)              # m/s
        self.declare_parameter('stable_tolerance', 0.05)   # m
        self.declare_parameter('stable_time', 1.0)         # s
        self.declare_parameter('rate_hz', 20.0)
        if not self.has_parameter('use_sim_time'):
             self.declare_parameter('use_sim_time', True)

        self.uav_names: List[str] = [str(x) for x in self.get_parameter('uav_names').value]
        self.target_z: float = float(self.get_parameter('target_altitude').value)
        self.kp: float = float(self.get_parameter('kp').value)
        self.vz_max: float = float(self.get_parameter('vz_max').value)
        self.tol: float = float(self.get_parameter('stable_tolerance').value)
        self.stable_time: float = float(self.get_parameter('stable_time').value)
        self.rate_hz: float = float(self.get_parameter('rate_hz').value)

        # Per-UAV state
        self.odom_z: Dict[str, float] = {u: float('nan') for u in self.uav_names}
        self.stable_counts: Dict[str, int] = {u: 0 for u in self.uav_names}
        self.reached: Dict[str, bool] = {u: False for u in self.uav_names}

        # Publishers / Subscribers
        self.pub: Dict[str, rclpy.publisher.Publisher] = {}
        for u in self.uav_names:
            self.pub[u] = self.create_publisher(Twist, f'/{u}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{u}/odom',
                                     lambda msg, uu=u: self._odom_cb(uu, msg), 10)

        # Control loop timer
        self.timer = self.create_timer(1.0 / self.rate_hz, self._control_step)
        self.get_logger().info(f"Takeoff controller started for {self.uav_names}, target={self.target_z:.2f} m")

    def _odom_cb(self, uav: str, msg: Odometry):
        self.odom_z[uav] = msg.pose.pose.position.z

    def _control_step(self):
        for u in self.uav_names:
            z = self.odom_z[u]
            if math.isnan(z):
                # wait for odometry
                continue

            err = self.target_z - z
            within = abs(err) <= self.tol

            # simple stability detector
            if within:
                self.stable_counts[u] += 1
            else:
                self.stable_counts[u] = 0

            stable_needed = max(1, int(self.stable_time * self.rate_hz))
            reached = self.stable_counts[u] >= stable_needed

            if reached and not self.reached[u]:
                self.get_logger().info(f"{u} reached ~{self.target_z:.2f} m and is hovering.")
                self.reached[u] = True

            cmd = Twist()
            if reached:
                cmd.linear.z = 0.0
            else:
                cmd.linear.z = clamp(self.kp * err, -self.vz_max, self.vz_max)
            # keep XY/yaw zero
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0

            self.pub[u].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FleetTakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

