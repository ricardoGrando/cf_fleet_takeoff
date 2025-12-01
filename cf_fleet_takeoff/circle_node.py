#!/usr/bin/env python3
import math
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time as RosTime


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    # ZYX yaw from quaternion
    s = 2.0*(w*z + x*y)
    c = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(s, c)


class FleetCircleNode(Node):
    """
    Makes N UAVs move around a fixed circle (center, radius) at constant tangential speed,
    with a fixed arc-length spacing between neighbors. Uses a simple 'tangent + attraction'
    velocity field in the XY plane and a P controller on altitude.

      v_xy = v_tangent(theta_des) + k_e * (p_des - p_cur)   [clamped]
      v_z  = kp_z * (z_target - z)                           [clamped]

    You can publish in world frame (default) or convert to body frame using odom yaw.
    """

    def __init__(self):
        super().__init__('cf_fleet_circle')

        # -------- Parameters --------
        self.declare_parameter('uav_names', ['cf1', 'cf2'])
        self.declare_parameter('speed', 0.1)                  # m/s along the circle
        self.declare_parameter('spacing', 0.2)                # arc length between UAVs (m)
        self.declare_parameter('radius', 2.0)                 # meters
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('altitude', 1.0)               # target z (m)
        self.declare_parameter('rate_hz', 20.0)               # control frequency
        self.declare_parameter('k_error_xy', 0.5)             # attraction gain [m/s per m]
        self.declare_parameter('xy_max', 0.6)                 # clamp for planar speed (m/s)
        self.declare_parameter('kp_z', 1.2)                   # altitude P gain
        self.declare_parameter('vz_max', 0.7)                 # vertical speed clamp (m/s)
        self.declare_parameter('frame', 'world')              # 'world' or 'body'
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        self.uavs: List[str] = [str(u) for u in self.get_parameter('uav_names').value]
        self.v = float(self.get_parameter('speed').value)
        self.s = float(self.get_parameter('spacing').value)
        self.R = float(self.get_parameter('radius').value)
        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)
        self.z_ref = float(self.get_parameter('altitude').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.k_e = float(self.get_parameter('k_error_xy').value)
        self.xy_max = float(self.get_parameter('xy_max').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.vz_max = float(self.get_parameter('vz_max').value)
        self.frame = str(self.get_parameter('frame').value).lower()

        # Derived
        self.omega = self.v / max(self.R, 1e-6)           # rad/s
        self.delta_theta = self.s / max(self.R, 1e-6)     # rad offset between neighbors
        self.t0_ns = self.get_clock().now().nanoseconds

        # State
        self.odom_xy: Dict[str, Tuple[float, float]] = {u: (math.nan, math.nan) for u in self.uavs}
        self.odom_z: Dict[str, float] = {u: math.nan for u in self.uavs}
        self.odom_yaw: Dict[str, float] = {u: math.nan for u in self.uavs}

        # IO
        self.pub: Dict[str, rclpy.publisher.Publisher] = {}
        for u in self.uavs:
            self.pub[u] = self.create_publisher(Twist, f'/{u}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{u}/odom',
                                     lambda msg, uu=u: self._odom_cb(uu, msg), 20)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._step)
        self.get_logger().info(
            f"FleetCircleNode: {self.uavs}, R={self.R} m, v={self.v} m/s, spacing={self.s} m, "
            f"center=({self.cx},{self.cy}), altitude={self.z_ref} m, frame={self.frame}"
        )

    # --- Helpers ---
    def _now_s(self) -> float:
        return (self.get_clock().now().nanoseconds - self.t0_ns) * 1e-9

    def _odom_cb(self, u: str, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_xy[u] = (p.x, p.y)
        self.odom_z[u] = p.z
        self.odom_yaw[u] = yaw_from_quat(q.x, q.y, q.z, q.w)

    def _target_angle(self, idx: int, t: float) -> float:
        return (self.omega * t + idx * self.delta_theta) % (2.0 * math.pi)

    def _step(self):
        t = self._now_s()

        for idx, u in enumerate(self.uavs):
            x, y = self.odom_xy[u]
            z = self.odom_z[u]
            yaw = self.odom_yaw[u]

            # Wait for odom
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            # --- Desired point on circle & tangent (world frame) ---
            th = self._target_angle(idx, t)
            cpx = self.cx + self.R * math.cos(th)
            cpy = self.cy + self.R * math.sin(th)

            # Tangent unit (CCW): [-sin, cos]
            tx, ty = -math.sin(th), math.cos(th)
            vx_tan = self.v * tx
            vy_tan = self.v * ty

            # Attraction to desired phase point
            ex, ey = (cpx - x), (cpy - y)
            vx_attr = self.k_e * ex
            vy_attr = self.k_e * ey

            vx_w = vx_tan + vx_attr
            vy_w = vy_tan + vy_attr

            # Clamp planar speed
            speed_xy = math.hypot(vx_w, vy_w)
            if speed_xy > self.xy_max:
                scale = self.xy_max / max(speed_xy, 1e-6)
                vx_w *= scale
                vy_w *= scale

            # Altitude control (P)
            vz = clamp(self.kp_z * (self.z_ref - z), -self.vz_max, self.vz_max)

            # Frame conversion
            cmd = Twist()
            if self.frame == 'body':
                if math.isnan(yaw):
                    # No yaw yet: fall back to world
                    cmd.linear.x, cmd.linear.y = vx_w, vy_w
                else:
                    cyaw = math.cos(-yaw)
                    syaw = math.sin(-yaw)
                    # world -> body rotation (around Z)
                    cmd.linear.x = cyaw * vx_w - syaw * vy_w
                    cmd.linear.y = syaw * vx_w + cyaw * vy_w
            else:
                cmd.linear.x, cmd.linear.y = vx_w, vy_w

            cmd.linear.z = vz
            cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0

            self.pub[u].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCircleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

