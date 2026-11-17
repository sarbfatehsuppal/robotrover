"""The physical rover, simulated: motors, suspension, sonar, bumping into things.

This replaces the hardware below the STM32. It takes the six motor duties
and two steering angles the body controller produces and works out what
the rover actually does on terrain made of boxes:

* each wheel's speed comes from the DC motor model under the load the
  suspension puts on it (``statics.solve_side``), so climbing slows it down
  and a step that's too much stalls it
* each side's rocker-bogie is solved along its own wheel track
  (``suspension.SideSolver``), and the differential bar averages the two
  rockers into the body pitch
* the HC-SR04 is a cone of rays from where it's mounted on the body, so
  when the nose pitches up on a climb the sensor looks up too
* anything the body or the suspension can't get over is a wall

No ROS in here; ``sim_node.py`` wraps it. That keeps it testable on any
machine with plain Python.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from .params import RoverParams
from .statics import gripping_torque, solve_side
from .suspension import SideGeometry, SideSolver, _solve3
from .terrain import CPath, World

# Wheel order everywhere: FL, ML, RL, FR, MR, RR (same as rover_core).
LEFT, RIGHT = +1, -1
SIDE_WHEELS = {LEFT: (0, 1, 2), RIGHT: (3, 4, 5)}
U_MIN, U_MAX = -0.32, 0.42  # track window around the rocker pivot, metres
# Past ~40 degrees nose-up the body's centre of mass is close to behind the
# rear wheels; the real thing would go over backwards. We just stop it there.
TIP_PITCH = math.radians(40)
# Echoes only come back from surfaces within ~50 degrees of face-on.
SONAR_MIN_COS = math.cos(math.radians(50))


@dataclass
class SideState:
    """Shape of one rocker-bogie: wheel centres relative to the pivot (u) and absolute height (z)."""

    rel_u: Tuple[float, float, float]
    z: Tuple[float, float, float]
    pivot_z: float
    rocker_pitch: float = 0.0  # nose-up positive (side view)
    bogie_angle: float = 0.0   # relative to the rocker
    torque: float = 0.0        # motor torque this side's wheels need right now
    ok: bool = True
    reason: str = ''


@dataclass
class PlantState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0  # ROS convention: positive = nose down
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    wheel_speed: List[float] = field(default_factory=lambda: [0.0] * 6)
    wheel_angle: List[float] = field(default_factory=lambda: [0.0] * 6)
    steer: List[float] = field(default_factory=lambda: [0.0, 0.0])
    rocker_joint: Dict[int, float] = field(default_factory=lambda: {LEFT: 0.0, RIGHT: 0.0})
    bogie_joint: Dict[int, float] = field(default_factory=lambda: {LEFT: 0.0, RIGHT: 0.0})
    bumped: bool = False
    stuck: str = ''


def rpy_matrix(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr))


def _mul(R, v):
    return (R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
            R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
            R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2])


class Plant:
    def __init__(self, p: RoverParams, world: World, x=0.0, y=0.0, yaw=0.0, seed: int = 1):
        self.p = p
        self.world = world
        self.geo = SideGeometry.from_params(p)
        self.solver = SideSolver(self.geo)
        self.s = PlantState(x=x, y=y, yaw=yaw, z=p.pivot_height)
        self.duty = [0.0] * 6
        self.steer_cmd = [0.0, 0.0]
        self.rng = random.Random(seed)
        g = self.geo
        self.wheel_xy = [(g.F[0], p.track_width / 2), (g.M[0], p.track_width / 2), (g.R[0], p.track_width / 2),
                         (g.F[0], -p.track_width / 2), (g.M[0], -p.track_width / 2), (g.R[0], -p.track_width / 2)]
        self.sides: Dict[int, SideState] = {}
        for side in (LEFT, RIGHT):
            self.sides[side] = self._settle(side)
        self._update_body()

    # ------------------------------------------------------------ inputs
    def command(self, duties, steer) -> None:
        self.duty = [max(-1.0, min(1.0, float(d))) for d in duties[:6]]
        lim = self.p.steer_limit
        self.steer_cmd = [max(-lim, min(lim, float(a))) for a in steer[:2]]

    # ------------------------------------------------------------ geometry helpers
    def _side_origin(self, side: int, x: float, y: float, yaw: float) -> Tuple[float, float]:
        h = side * self.p.track_width / 2
        return x - h * math.sin(yaw), y + h * math.cos(yaw)

    def _path(self, side: int, x: float, y: float, yaw: float) -> CPath:
        ox, oy = self._side_origin(side, x, y, yaw)
        blocks = self.world.profile(ox, oy, yaw, U_MIN - 0.05, U_MAX + 0.05)
        return CPath(blocks, self.p.wheel_radius, U_MIN, U_MAX, resolution=0.002)

    def _settle(self, side: int) -> SideState:
        """Shape of a side standing still at the current pose (used at start-up)."""
        path = self._path(side, self.s.x, self.s.y, self.s.yaw)
        sol = self.solver.solve(path, self.solver.initial(path, 0.0),
                                sum(self.solver.initial(path, 0.0)) / 3)
        sol = sol or self.solver.initial(path, 0.0)
        return self._state_from(path, sol)

    def _state_from(self, path: CPath, sol) -> SideState:
        pose = self.solver.pose(path, sol)
        pu = pose.P[0]
        return SideState(rel_u=(pose.F[0] - pu, pose.M[0] - pu, pose.R[0] - pu),
                         z=(pose.F[1], pose.M[1], pose.R[1]), pivot_z=pose.P[1],
                         rocker_pitch=pose.rocker_pitch, bogie_angle=pose.bogie_angle)

    # ------------------------------------------------------------ main step
    def step(self, dt: float) -> PlantState:
        p, st = self.p, self.s
        st.bumped = False
        st.stuck = ''

        # Steering servos slew toward their targets.
        for k in range(2):
            err = self.steer_cmd[k] - st.steer[k]
            st.steer[k] += max(-p.steer_rate * dt, min(p.steer_rate * dt, err))

        # Per side: where are we on the terrain, what does it take to move?
        progress: Dict[int, float] = {}
        new_sides: Dict[int, SideState] = {}
        for side in (LEFT, RIGHT):
            idx = SIDE_WHEELS[side]
            path = self._path(side, st.x, st.y, st.yaw)
            prev = self.sides[side]
            guess = tuple(path.project(prev.rel_u[i], prev.z[i]) for i in range(3))
            sol = self.solver.solve(path, guess, sum(guess) / 3) or self.solver.initial(path, 0.0)
            pose0 = self.solver.pose(path, sol)
            direction = sum(self.duty[i] for i in idx)
            forces = solve_side(path, pose0, p, 1.0 if direction >= 0 else -1.0)
            tau = gripping_torque(forces) if forces.ok else 0.0

            # Motor model, with a little lag for inertia.
            travel = 0.0
            for i in idx:
                target = p.wheel_speed(self.duty[i], tau)
                st.wheel_speed[i] += (target - st.wheel_speed[i]) * min(1.0, dt / p.motor_time_constant)
                st.wheel_angle[i] = math.remainder(st.wheel_angle[i] + st.wheel_speed[i] * dt, 2 * math.pi)
                travel += st.wheel_speed[i] * p.wheel_radius * dt / 3.0

            state = self._state_from(path, sol)
            state.torque, state.ok, state.reason = tau, forces.ok, forces.reason
            if not forces.ok and travel > 0:
                # Spinning wheels, no progress.
                progress[side] = 0.0
                st.stuck = forces.reason or 'slipping'
                new_sides[side] = state
                continue
            moved = self.solver.advance(path, sol, travel) if abs(travel) > 1e-7 else sol
            if moved is None:
                progress[side] = 0.0
                st.stuck = 'suspension at its limit'
                new_sides[side] = state
                continue
            pose1 = self.solver.pose(path, moved)
            if abs(pose1.bogie_angle) > p.bogie_limit:
                progress[side] = 0.0
                st.stuck = 'bogie at its stop'
                new_sides[side] = state
                continue
            if abs(pose1.rocker_pitch) > TIP_PITCH and abs(pose1.rocker_pitch) > abs(pose0.rocker_pitch):
                progress[side] = 0.0
                st.stuck = 'too steep, it would flip'
                new_sides[side] = state
                continue
            du = pose1.P[0] - pose0.P[0]
            progress[side] = du / travel if abs(travel) > 1e-6 else 1.0
            new_sides[side] = self._state_from(path, moved)
            new_sides[side].torque, new_sides[side].ok = tau, forces.ok

        # Body velocity from the six wheels (least squares; lateral slip allowed).
        A = [[0.0] * 3 for _ in range(3)]
        b = [0.0, 0.0, 0.0]
        for i, (wx, wy) in enumerate(self.wheel_xy):
            delta = st.steer[0] if i == 0 else (st.steer[1] if i == 3 else 0.0)
            side = LEFT if i < 3 else RIGHT
            roll_speed = st.wheel_speed[i] * p.wheel_radius * max(-0.5, min(1.5, progress[side]))
            c, s_ = math.cos(delta), math.sin(delta)
            for (hx, hy), target, weight in (((c, s_), roll_speed, 1.0), ((-s_, c), 0.0, 0.4)):
                row = (hx, hy, -hx * wy + hy * wx)  # d(h . v_i)/d(vx, vy, wz)
                for r in range(3):
                    b[r] += weight * row[r] * target
                    for k in range(3):
                        A[r][k] += weight * row[r] * row[k]
        try:
            vx, vy, wz = _solve3(A, b)
        except ZeroDivisionError:
            vx = vy = wz = 0.0

        # Integrate, then check the body didn't end up inside something.
        old = (st.x, st.y, st.yaw)
        c, s_ = math.cos(st.yaw + 0.5 * wz * dt), math.sin(st.yaw + 0.5 * wz * dt)
        st.x += (vx * c - vy * s_) * dt
        st.y += (vx * s_ + vy * c) * dt
        st.yaw = math.remainder(st.yaw + wz * dt, 2 * math.pi)
        old_sides = self.sides
        self.sides = new_sides
        self._update_body()
        if self._body_hits_something():
            st.x, st.y, st.yaw = old
            self.sides = old_sides
            self._update_body()
            st.bumped = True
            vx = vy = wz = 0.0
        st.vx, st.vy, st.wz = vx, vy, wz
        return st

    # ------------------------------------------------------------ body pose
    def _update_body(self) -> None:
        st = self.s
        L, R = self.sides[LEFT], self.sides[RIGHT]
        body_up = 0.5 * (L.rocker_pitch + R.rocker_pitch)  # differential bar averages the rockers
        st.pitch = -body_up
        st.z = 0.5 * (L.pivot_z + R.pivot_z)
        st.roll = math.atan2(L.pivot_z - R.pivot_z, self.p.track_width)
        # URDF joints rotate about +y, which is nose-down; the side model is nose-up.
        st.rocker_joint = {LEFT: -(L.rocker_pitch - body_up), RIGHT: -(R.rocker_pitch - body_up)}
        st.bogie_joint = {LEFT: -L.bogie_angle, RIGHT: -R.bogie_angle}

    def to_world(self, v):
        st = self.s
        R = rpy_matrix(st.roll, st.pitch, st.yaw)
        w = _mul(R, v)
        return st.x + w[0], st.y + w[1], st.z + w[2]

    def _body_hits_something(self) -> bool:
        p = self.p
        hx, hy, hz = p.body_length / 2, p.body_width / 2, p.body_height / 2
        cx, cz = p.body_center_x, p.body_center_z
        pts = []
        for fx in (-1.0, -0.5, 0.0, 0.5, 1.0):
            for fy in (-1.0, 0.0, 1.0):
                pts.append((cx + fx * hx, fy * hy, cz - hz))  # underside
        for fy in (-1.0, 0.0, 1.0):
            for fz in (-1.0, 0.0, 1.0):
                pts.append((cx + hx, fy * hy, cz + fz * hz))  # nose
                pts.append((cx - hx, fy * hy, cz + fz * hz))  # tail
        for q in pts:
            x, y, z = self.to_world(q)
            if self.world.height_at(x, y) > z:
                return True
        return False

    # ------------------------------------------------------------ sensors
    def sonar(self, noise: float = 0.003) -> float:
        """HC-SR04 reading in metres, or math.inf for no echo."""
        p = self.p
        origin = self.to_world((p.sonar_x, 0.0, p.sonar_z))
        R = rpy_matrix(self.s.roll, self.s.pitch, self.s.yaw)
        best = math.inf
        half = p.sonar_half_angle
        for az_k in (-1.0, -0.5, 0.0, 0.5, 1.0):
            for el_k in (-1.0, -0.5, 0.0, 0.5, 1.0):
                if az_k * az_k + el_k * el_k > 1.0001:
                    continue  # stay inside the cone
                az, el = az_k * half, el_k * half + p.sonar_tilt
                d_body = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
                d = _mul(R, d_body)
                t = self._ray(origin, d, p.sonar_max_range)
                best = min(best, t)
        if best >= p.sonar_max_range:
            return math.inf
        return max(p.sonar_min_range, best + self.rng.gauss(0.0, noise))

    def _ray(self, o, d, max_range: float) -> float:
        """One ray of the sonar cone. Ultrasound bounces off flat surfaces like
        light off a mirror, so a face hit at a shallow angle sends the ping
        away and nothing comes back. That's why the floor doesn't show up
        when the nose dips going down a step, and why the sensor can miss a
        wall it approaches at a steep angle."""
        best, best_cos = max_range, 1.0
        if d[2] < -1e-6:
            t = -o[2] / d[2]
            if t < best:
                best, best_cos = t, -d[2]
        for b in self.world.boxes:
            hit = b.ray_hit_cos(o, d)
            if hit is not None and hit[0] < best:
                best, best_cos = hit
        if best < max_range and best_cos < SONAR_MIN_COS:
            return max_range
        return best
