#!/usr/bin/env python3
import math
import random
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v

def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    s = 2.0*(w*z + x*y); c = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(s, c)

def quat_rotate(qx, qy, qz, qw, vx, vy, vz, world_to_body: bool) -> Tuple[float,float,float]:
    # Rotation matrix R (body->world)
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    R00 = 1 - 2*(yy+zz); R01 = 2*(xy - wz);   R02 = 2*(xz + wy)
    R10 = 2*(xy + wz);   R11 = 1 - 2*(xx+zz); R12 = 2*(yz - wx)
    R20 = 2*(xz - wy);   R21 = 2*(yz + wx);   R22 = 1 - 2*(xx+yy)
    if world_to_body:
        # body <- world = R^T
        return (R00*vx + R10*vy + R20*vz,
                R01*vx + R11*vy + R21*vz,
                R02*vx + R12*vy + R22*vz)
    else:
        return (R00*vx + R01*vy + R02*vz,
                R10*vx + R11*vy + R12*vz,
                R20*vx + R21*vy + R22*vz)

class FleetLissajousTakeoffNode(Node):
    """
    Staged operation per UAV:
      WAIT (t < go_time) -> TAKEOFF (to z_takeoff) -> BLEND (ramp XY to path) -> CRUISE (Lissajous)
    XY is held at 0 during TAKEOFF to prevent tip-overs. World->Body conversion is full-quaternion.
    """

    PH_WAIT, PH_TAKEOFF, PH_BLEND, PH_CRUISE = 0, 1, 2, 3

    def __init__(self):
        super().__init__('cf_fleet_lissajous_takeoff')

        # ---------- Core params ----------
        self.declare_parameter(
            'uav_names',
            ['cf1','cf2','cf3'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('frame', 'body')  # 'body' recommended for this plugin
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        # Scenario bounds (stay inside 4x4 → amps <= 2.0)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('amp_x', 1.8)
        self.declare_parameter('amp_y', 1.8)

        # Lissajous (random per UAV)
        self.declare_parameter('a_choices', [1,2,3])
        self.declare_parameter('b_choices', [2,3,4])
        self.declare_parameter('speed_min', 0.1)
        self.declare_parameter('speed_max', 0.25)
        self.declare_parameter('seed', 42)

        # Altitudes
        self.declare_parameter('altitude_base', 1.0)
        self.declare_parameter('altitude_step', 0.1)

        # Takeoff & staggering
        self.declare_parameter('takeoff_altitude', 0.5)   # climb to this first
        self.declare_parameter('start_delay_s', 3.0)      # spacing between UAV takeoffs
        self.declare_parameter('takeoff_tolerance', 0.05)
        self.declare_parameter('takeoff_stable_time', 0.7)  # seconds
        self.declare_parameter('blend_time', 1.5)           # seconds to ramp XY after takeoff

        # Controllers / limits
        self.declare_parameter('k_path', 0.6)
        self.declare_parameter('xy_max', 0.6)
        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('vz_max', 1.0)

        # ----------- Read params -----------
        self.uavs: List[str] = [str(u) for u in self.get_parameter('uav_names').value]
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.frame = str(self.get_parameter('frame').value).lower()

        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)
        self.Ax = float(self.get_parameter('amp_x').value)
        self.Ay = float(self.get_parameter('amp_y').value)

        self.a_choices = [int(x) for x in self.get_parameter('a_choices').value]
        self.b_choices = [int(x) for x in self.get_parameter('b_choices').value]
        self.vmin = float(self.get_parameter('speed_min').value)
        self.vmax = float(self.get_parameter('speed_max').value)
        seed = int(self.get_parameter('seed').value)
        rng = random.Random(seed)

        self.z_base = float(self.get_parameter('altitude_base').value)
        self.z_step = float(self.get_parameter('altitude_step').value)

        self.z_takeoff = float(self.get_parameter('takeoff_altitude').value)
        self.start_delay = float(self.get_parameter('start_delay_s').value)
        self.tol_z = float(self.get_parameter('takeoff_tolerance').value)
        self.stable_take_s = float(self.get_parameter('takeoff_stable_time').value)
        self.blend_time = float(self.get_parameter('blend_time').value)

        self.k_path = float(self.get_parameter('k_path').value)
        self.xy_max = float(self.get_parameter('xy_max').value)
        self.kp_z = float(self.get_parameter('kp_z').value)
        self.vz_max = float(self.get_parameter('vz_max').value)

        # ---------- Per-UAV path params ----------
        self.path_params = {}
        for idx, u in enumerate(self.uavs):
            a = rng.choice(self.a_choices)
            b = rng.choice(self.b_choices)
            if a == 1 and b == 1: b = 2
            phi_x = rng.uniform(0.0, 2*math.pi)
            phi_y = rng.uniform(0.0, 2*math.pi)
            v_i  = rng.uniform(self.vmin, self.vmax)
            u_i  = rng.uniform(0.0, 2*math.pi)
            z_i  = self.z_base + idx*self.z_step
            self.path_params[u] = dict(a=a,b=b,phi_x=phi_x,phi_y=phi_y,v=v_i,u=u_i,z=z_i)

        # ---------- State ----------
        self.odom_xy: Dict[str, Tuple[float, float]] = {u:(math.nan,math.nan) for u in self.uavs}
        self.odom_z:  Dict[str, float] = {u: math.nan for u in self.uavs}
        self.odom_q:  Dict[str, Tuple[float,float,float,float]] = {u:(math.nan,)*4 for u in self.uavs}
        self.phase:   Dict[str, int] = {u: self.PH_WAIT for u in self.uavs}
        self.stable_cnt: Dict[str, int] = {u: 0 for u in self.uavs}
        self.go_time: Dict[str, float] = {}     # when TAKEOFF starts
        self.blend_t0: Dict[str, float] = {}    # when BLEND starts

        t0 = self.get_clock().now().nanoseconds*1e-9
        for i,u in enumerate(self.uavs):
            self.go_time[u] = t0 + i*self.start_delay  # staggered start

        # ---------- IO ----------
        self.pub: Dict[str, rclpy.publisher.Publisher] = {}
        for u in self.uavs:
            self.pub[u] = self.create_publisher(Twist, f'/{u}/cmd_vel', 10)
            self.create_subscription(Odometry, f'/{u}/odom', lambda msg, uu=u: self._odom_cb(uu, msg), 20)

        self.timer = self.create_timer(1.0/self.rate_hz, self._step)
        self.get_logger().info(
            "Staged Lissajous with takeoff: " +
            ", ".join([f"{u}(a={p['a']},b={p['b']},v={p['v']:.2f},z={p['z']:.2f})" for u,p in self.path_params.items()])
            + f" | takeoff={self.z_takeoff} m, delay={self.start_delay}s, blend={self.blend_time}s"
        )

    # ---------- Callbacks ----------
    def _odom_cb(self, u: str, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.odom_xy[u] = (p.x, p.y)
        self.odom_z[u]  = p.z
        self.odom_q[u]  = (q.x, q.y, q.z, q.w)

    # ---------- Path helpers ----------
    def _path_point(self, a, b, u, phi_x, phi_y):
        x = self.cx + self.Ax * math.sin(a*u + phi_x)
        y = self.cy + self.Ay * math.sin(b*u + phi_y)
        return x, y

    def _path_deriv(self, a, b, u, phi_x, phi_y):
        dxdu = self.Ax * a * math.cos(a*u + phi_x)
        dydu = self.Ay * b * math.cos(b*u + phi_y)
        return dxdu, dydu

    # ---------- Control loop ----------
    def _step(self):
        now = self.get_clock().now().nanoseconds*1e-9
        dt = 1.0/self.rate_hz
        stable_needed = max(1, int(self.stable_take_s*self.rate_hz))

        for u in self.uavs:
            x, y = self.odom_xy[u]
            z     = self.odom_z[u]
            qx,qy,qz,qw = self.odom_q[u]
            if math.isnan(z):
                continue  # wait for odom

            cmd = Twist()

            # Phase transitions
            if self.phase[u] == self.PH_WAIT:
                # Hold still until it's this UAV's turn
                if now >= self.go_time[u]:
                    self.phase[u] = self.PH_TAKEOFF
                # publish zeros
                self.pub[u].publish(cmd)
                continue

            if self.phase[u] == self.PH_TAKEOFF:
                # Only climb vertically to z_takeoff
                vz = clamp(self.kp_z*(self.z_takeoff - z), -self.vz_max, self.vz_max)
                cmd.linear.z = vz
                # XY zero during takeoff
                cmd.linear.x = cmd.linear.y = 0.0
                self.pub[u].publish(cmd)

                # Check stability
                if abs(self.z_takeoff - z) <= self.tol_z:
                    self.stable_cnt[u] += 1
                else:
                    self.stable_cnt[u] = 0
                if self.stable_cnt[u] >= stable_needed:
                    self.phase[u] = self.PH_BLEND
                    self.blend_t0[u] = now
                continue

            # From here: compute Lissajous XY world velocities and altitude to final z
            p = self.path_params[u]
            a,b,phi_x,phi_y = p['a'], p['b'], p['phi_x'], p['phi_y']
            u_param = p['u']; v_des = p['v']; z_ref = p['z']

            # advance parameter u to maintain desired tangential speed
            dxdu, dydu = self._path_deriv(a,b,u_param,phi_x,phi_y)
            du_norm = max(1e-4, math.hypot(dxdu,dydu))
            u_param = (u_param + (v_des/du_norm)*dt) % (2.0*math.pi)
            p['u'] = u_param

            xd, yd = self._path_point(a,b,u_param,phi_x,phi_y)
            dxdu, dydu = self._path_deriv(a,b,u_param,phi_x,phi_y)
            tnx, tny = dxdu, dydu
            tnorm = math.hypot(tnx,tny)
            if tnorm < 1e-6: tnx,tny,tnorm = 1.0,0.0,1.0
            tnx /= tnorm; tny /= tnorm

            vx_tan = v_des * tnx
            vy_tan = v_des * tny
            ex, ey = (xd - x), (yd - y)
            vx_attr = self.k_path * ex
            vy_attr = self.k_path * ey

            vx_w = vx_tan + vx_attr
            vy_w = vy_tan + vy_attr
            sp = math.hypot(vx_w, vy_w)
            if sp > self.xy_max:
                s = self.xy_max / sp
                vx_w *= s; vy_w *= s

            vz_w = clamp(self.kp_z*(z_ref - z), -self.vz_max, self.vz_max)

            if self.phase[u] == self.PH_BLEND:
                # Gradually ramp XY from 0 to full over blend_time
                alpha = min(1.0, (now - self.blend_t0[u]) / max(1e-3, self.blend_time))
                vx_w *= alpha
                vy_w *= alpha
                if alpha >= 1.0:
                    self.phase[u] = self.PH_CRUISE

            # Frame conversion
            if self.frame == 'body' and not any(math.isnan(k) for k in (qx,qy,qz,qw)):
                vx_b, vy_b, vz_b = quat_rotate(qx,qy,qz,qw, vx_w,vy_w,vz_w, world_to_body=True)
                cmd.linear.x, cmd.linear.y, cmd.linear.z = vx_b, vy_b, vz_b
            else:
                cmd.linear.x, cmd.linear.y, cmd.linear.z = vx_w, vy_w, vz_w

            self.pub[u].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = FleetLissajousTakeoffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

