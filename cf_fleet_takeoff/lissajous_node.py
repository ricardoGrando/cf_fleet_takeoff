#!/usr/bin/env python3
import math
import random
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# add imports near the top
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    s = 2.0 * (w*z + x*y)
    c = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(s, c)

class FleetLissajousNode(Node):
    """
    Each UAV tracks a fixed 2D Lissajous curve, forever:
      x = cx + Ax * sin(a * u + phi_x)
      y = cy + Ay * sin(b * u + phi_y)

    We advance u per-UAV so that tangential speed ≈ v_i (random in [vmin, vmax]),
    and command velocity = v_tangent + k_path*(p_des - p_cur) (clamped).
    Altitude is held with a small P controller at z_i = z_base + i*dz.
    """

    def __init__(self):
        super().__init__('cf_fleet_lissajous')

        # ---------- Parameters ----------
        self.declare_parameter(
	    'uav_names',
	    ['cf1', 'cf2', 'cf3'],  # non-empty default -> still explicit string array
	    ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
	)
        self.declare_parameter('count', 3)
        self.declare_parameter('uav_prefix', 'cf')

        # Scenario (centered at 0,0; amplitudes must fit inside ±2.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('amp_x', 1.8)               # ≤ 2.0 to stay in 4×4
        self.declare_parameter('amp_y', 1.8)

        # Lissajous integer pairs (a,b) chosen per UAV (random from these sets)
        self.declare_parameter('a_choices', [1, 2, 3])
        self.declare_parameter('b_choices', [2, 3, 4])

        # Speed band (m/s)
        self.declare_parameter('speed_min', 0.1)
        self.declare_parameter('speed_max', 0.25)

        # Altitude profile
        self.declare_parameter('altitude_base', 1.0)
        self.declare_parameter('altitude_step', 0.1)

        # Control
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('k_path', 0.6)              # attraction gain [m/s per m]
        self.declare_parameter('xy_max', 0.6)              # planar speed clamp
        self.declare_parameter('kp_z', 1.2)                # altitude P gain
        self.declare_parameter('vz_max', 0.5)              # vertical speed clamp
        self.declare_parameter('frame', 'world')           # 'world' or 'body'
        self.declare_parameter('seed', 42)

        if not self.has_parameter('use_sim_time'):
    	    self.declare_parameter('use_sim_time', True)

        # ---------- Read params ----------
        names_param = self.get_parameter('uav_names').value
        if names_param:
            self.uavs: List[str] = [str(n) for n in names_param]
        else:
            count = int(self.get_parameter('count').value)
            prefix = str(self.get_parameter('uav_prefix').value)
            self.uavs = [f"{prefix}{i}" for i in range(1, count + 1)]

        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)
        self.Ax = float(self.get_parameter('amp_x').value)
        self.Ay = float(self.get_parameter('amp_y').value)
        self.a_choices = [int(x) for x in self.get_parameter('a_choices').value]
        self.b_choices = [int(x) for x in self.get_parameter('b_choices').value]
        self.vmin = float(self.get_parameter('speed_min').value)
        self.vmax = float(self.get_parameter('speed_max').value)
        self.z_base = float(self.get_parameter('altitude_base').value)
        self.z_step = float(self.get_parameter('altitude_step').value)

        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.k_path = float(self.get_parameter('k_path').value)
        self.xy_max = float(self.get_parameter('xy_max').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.vz_max = float(self.get_parameter('vz_max').value)
        self.frame = str(self.get_parameter('frame').value).lower()
        seed = int(self.get_parameter('seed').value)
        rng = random.Random(seed)

        # ---------- Per-UAV path params ----------
        # Each UAV gets (a,b), phases, speed v_i, and an evolving path parameter u_i
        self.path_params = {}
        for idx, u in enumerate(self.uavs):
            a = rng.choice(self.a_choices)
            b = rng.choice(self.b_choices)
            # avoid (a==b==1) for richer traces
            if a == 1 and b == 1:
                b = 2
            phi_x = rng.uniform(0.0, 2.0 * math.pi)
            phi_y = rng.uniform(0.0, 2.0 * math.pi)
            v_i = rng.uniform(self.vmin, self.vmax)
            u_i = rng.uniform(0.0, 2.0 * math.pi)  # start at random phase
            z_ref = self.z_base + idx * self.z_step
            self.path_params[u] = dict(a=a, b=b, phi_x=phi_x, phi_y=phi_y, v=v_i, u=u_i, z=z_ref)

        # ---------- State ----------
        self.odom_xy: Dict[str, Tuple[float, float]] = {u: (math.nan, math.nan) for u in self.uavs}
        self.odom_z: Dict[str, float] = {u: math.nan for u in self.uavs}
        self.odom_yaw: Dict[str, float] = {u: math.nan for u in self.uavs}

        # ---------- IO ----------
        self.pub: Dict[str, rclpy.publisher.Publisher] = {}
        for u in self.uavs:
            self.pub[u] = self.create_publisher(Twist, f'/{u}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{u}/odom',
                                     lambda msg, uu=u: self._odom_cb(uu, msg), 20)

        self.timer = self.create_timer(1.0 / self.rate_hz, self._step)
        self.get_logger().info(
            "FleetLissajousNode: " +
            ", ".join([f"{u}(a={p['a']},b={p['b']},v={p['v']:.2f},z={p['z']:.2f})"
                       for u, p in self.path_params.items()])
        )

    # ----- Callbacks -----
    def _odom_cb(self, u: str, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_xy[u] = (p.x, p.y)
        self.odom_z[u] = p.z
        self.odom_yaw[u] = yaw_from_quat(q.x, q.y, q.z, q.w)

    # ----- Path math -----
    def _path_point(self, a: int, b: int, u: float, phi_x: float, phi_y: float):
        x = self.cx + self.Ax * math.sin(a * u + phi_x)
        y = self.cy + self.Ay * math.sin(b * u + phi_y)
        return x, y

    def _path_deriv(self, a: int, b: int, u: float, phi_x: float, phi_y: float):
        # derivative wrt parameter u (not time)
        dxdu = self.Ax * a * math.cos(a * u + phi_x)
        dydu = self.Ay * b * math.cos(b * u + phi_y)
        return dxdu, dydu

    # ----- Control step -----
    def _step(self):
        dt = 1.0 / self.rate_hz
        for u in self.uavs:
            x, y = self.odom_xy[u]
            z = self.odom_z[u]
            yaw = self.odom_yaw[u]
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            p = self.path_params[u]
            a, b, phi_x, phi_y = p['a'], p['b'], p['phi_x'], p['phi_y']
            u_param = p['u']
            v_des = p['v']
            z_ref = p['z']

            # Advance path parameter so that speed ≈ v_des:
            dxdu, dydu = self._path_deriv(a, b, u_param, phi_x, phi_y)
            du_norm = math.hypot(dxdu, dydu)
            du_dt = v_des / max(du_norm, 1e-4)  # maintain desired tangential speed
            u_param = (u_param + du_dt * dt) % (2.0 * math.pi)  # wrap around
            p['u'] = u_param

            # Desired point & tangent (world frame)
            xd, yd = self._path_point(a, b, u_param, phi_x, phi_y)
            dxdu, dydu = self._path_deriv(a, b, u_param, phi_x, phi_y)
            # tangent unit
            tnx, tny = dxdu, dydu
            tnorm = math.hypot(tnx, tny)
            if tnorm < 1e-6:
                tnx, tny = 1.0, 0.0
                tnorm = 1.0
            tnx /= tnorm
            tny /= tnorm

            vx_tan = v_des * tnx
            vy_tan = v_des * tny

            # attraction towards desired point on path
            ex, ey = (xd - x), (yd - y)
            vx_attr = self.k_path * ex
            vy_attr = self.k_path * ey

            vx_w = vx_tan + vx_attr
            vy_w = vy_tan + vy_attr

            # Clamp planar speed
            speed_xy = math.hypot(vx_w, vy_w)
            if speed_xy > self.xy_max:
                scale = self.xy_max / speed_xy
                vx_w *= scale
                vy_w *= scale

            # Altitude P control
            vz = clamp(self.kp_z * (z_ref - z), -self.vz_max, self.vz_max)

            # Frame conversion
            cmd = Twist()
            if self.frame == 'body' and not math.isnan(yaw):
                cyaw = math.cos(-yaw)
                syaw = math.sin(-yaw)
                cmd.linear.x = cyaw * vx_w - syaw * vy_w
                cmd.linear.y = syaw * vx_w + cyaw * vy_w
            else:
                cmd.linear.x, cmd.linear.y = vx_w, vy_w

            cmd.linear.z = vz
            cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0

            self.pub[u].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FleetLissajousNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

