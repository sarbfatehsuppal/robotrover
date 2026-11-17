"""Planar (side view) model of one rocker-bogie.

Each side of the rover is two rigid parts joined by a free pin:

* the rocker: rocker pivot P (on the body), front wheel axle F, bogie pin B
* the bogie: bogie pin B, middle wheel axle M, rear wheel axle R

All three wheel centres sit on the terrain's wheel-centre path (``CPath``),
so the state of one side is just three arc lengths (s_F, s_M, s_R) along
that path. Two rigid-link constraints tie them together:

    |M - R| = bogie wheel spacing
    |F - B| = rocker length from the front axle to the bogie pin

That leaves one degree of freedom: how far the rover has driven. We use
S = (s_F + s_M + s_R) / 3, the average distance the wheels have rolled,
as the driving variable. Unlike the body's x position it always increases
when the wheels turn forward, even while the front wheel is creeping up a
wall and the body is actually being pushed back a couple of millimetres.

When both sides see the same terrain (a step across the whole width) the
differential bar keeps the rockers locked to the body, so the body pitches
with the rocker. That is the case this module models exactly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Tuple

from .params import RoverParams
from .terrain import CPath

Vec = Tuple[float, float]


def _rot(v: Vec, a: float) -> Vec:
    c, s = math.cos(a), math.sin(a)
    return c * v[0] - s * v[1], s * v[0] + c * v[1]


def _sub(a: Vec, b: Vec) -> Vec:
    return a[0] - b[0], a[1] - b[1]


def _add(a: Vec, b: Vec) -> Vec:
    return a[0] + b[0], a[1] + b[1]


def _norm(v: Vec) -> float:
    return math.hypot(v[0], v[1])


def _ang(v: Vec) -> float:
    return math.atan2(v[1], v[0])


@dataclass
class SideGeometry:
    """Rest positions of the side-view points, relative to the rocker pivot."""

    P: Vec
    F: Vec
    B: Vec
    M: Vec
    R: Vec

    @classmethod
    def from_params(cls, p: RoverParams) -> 'SideGeometry':
        return cls(P=(0.0, 0.0),
                   F=(p.front_axle_x, p.front_axle_z),
                   B=(p.bogie_pivot_x, p.bogie_pivot_z),
                   M=(p.middle_axle_x, p.middle_axle_z),
                   R=(p.rear_axle_x, p.rear_axle_z))

    @property
    def bogie_span(self) -> float:
        return _norm(_sub(self.M, self.R))

    @property
    def rocker_span(self) -> float:
        return _norm(_sub(self.F, self.B))


@dataclass
class SidePose:
    """Solved pose of one side, in path coordinates (u forward, z up)."""

    s: Tuple[float, float, float]  # arc lengths of F, M, R on the path
    F: Vec
    M: Vec
    R: Vec
    B: Vec
    P: Vec
    rocker_pitch: float            # world pitch of the rocker, nose-up positive
    bogie_angle: float             # bogie angle relative to the rocker, rear-up positive

    def body_point(self, rel: Vec) -> Vec:
        """World position of a point fixed to the rocker/body (relative to P)."""
        return _add(self.P, _rot(rel, self.rocker_pitch))

    def bogie_point(self, geo: SideGeometry, rel_from_pivot: Vec) -> Vec:
        """Point fixed to the bogie, given relative to the rocker pivot at rest."""
        rest = _sub(rel_from_pivot, geo.B)
        return _add(self.B, _rot(rest, self.rocker_pitch + self.bogie_angle))


class SideSolver:
    def __init__(self, geo: SideGeometry):
        self.geo = geo
        self._bogie_rest = _ang(_sub(geo.M, geo.R))
        self._rocker_rest = _ang(_sub(geo.F, geo.B))
        self._B_from_R = _sub(geo.B, geo.R)
        self._P_from_B = _sub(geo.P, geo.B)
        self.Lb = geo.bogie_span
        self.Lr = geo.rocker_span

    # -- forward kinematics from three path positions ----------------------
    def pose(self, path: CPath, s: Tuple[float, float, float]) -> SidePose:
        F, M, R = path.point(s[0]), path.point(s[1]), path.point(s[2])
        bogie_world = _ang(_sub(M, R)) - self._bogie_rest
        B = _add(R, _rot(self._B_from_R, bogie_world))
        rocker_world = _ang(_sub(F, B)) - self._rocker_rest
        P = _add(B, _rot(self._P_from_B, rocker_world))
        return SidePose(s=s, F=F, M=M, R=R, B=B, P=P,
                        rocker_pitch=rocker_world,
                        bogie_angle=bogie_world - rocker_world)

    def _residual(self, path: CPath, s, S3: float):
        F, M, R = path.point(s[0]), path.point(s[1]), path.point(s[2])
        bogie_world = _ang(_sub(M, R)) - self._bogie_rest
        B = _add(R, _rot(self._B_from_R, bogie_world))
        return (_norm(_sub(M, R)) - self.Lb,
                _norm(_sub(F, B)) - self.Lr,
                s[0] + s[1] + s[2] - S3)

    def initial(self, path: CPath, u_pivot: float) -> Tuple[float, float, float]:
        """Path positions for the rover standing on flat ground at u_pivot."""
        g = self.geo
        return (path.s_at_u(u_pivot + g.F[0]),
                path.s_at_u(u_pivot + g.M[0]),
                path.s_at_u(u_pivot + g.R[0]))

    def solve(self, path: CPath, guess: Tuple[float, float, float], S: float,
              tol: float = 1e-9, max_iter: int = 40) -> Tuple[float, float, float] | None:
        """Find (s_F, s_M, s_R) with average S, starting from a nearby guess."""
        s = list(guess)
        S3 = 3.0 * S
        h = 1e-6
        for _ in range(max_iter):
            g = self._residual(path, s, S3)
            if max(abs(x) for x in g) < tol:
                return tuple(s)  # type: ignore[return-value]
            J = [[0.0] * 3 for _ in range(3)]
            for j in range(3):
                sp = list(s)
                sp[j] += h
                gp = self._residual(path, sp, S3)
                for i in range(3):
                    J[i][j] = (gp[i] - g[i]) / h
            try:
                step = _solve3(J, [-x for x in g])
            except ZeroDivisionError:
                return None
            # Keep Newton steps short; the path has kinks at wall corners.
            big = max(abs(x) for x in step)
            if big > 0.01:
                step = [x * 0.01 / big for x in step]
            s = [min(max(s[i] + step[i], 0.0), path.length) for i in range(3)]
        g = self._residual(path, s, S3)
        if max(abs(x) for x in g) < 1e-6:
            return tuple(s)  # type: ignore[return-value]
        return None

    def advance(self, path: CPath, s0: Tuple[float, float, float], dS: float,
                max_sub: float = 0.002) -> Tuple[float, float, float] | None:
        """March the solution forward (or back) by dS in small steps."""
        S = sum(s0) / 3.0
        n = max(1, int(math.ceil(abs(dS) / max_sub)))
        s = s0
        prev = s0
        for k in range(1, n + 1):
            target = S + dS * k / n
            # Predict with the last step's direction, then correct.
            guess = tuple(2 * a - b for a, b in zip(s, prev)) if k > 1 else s
            sol = self.solve(path, guess, target) or self.solve(path, s, target)
            if sol is None:
                return None
            prev, s = s, sol
        return s


def _solve3(A, b):
    """Solve a 3x3 linear system with partial pivoting."""
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(3):
        p = max(range(c, 3), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-14:
            raise ZeroDivisionError
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, 3):
            f = M[r][c] / M[c][c]
            for k in range(c, 4):
                M[r][k] -= f * M[c][k]
    x = [0.0, 0.0, 0.0]
    for r in (2, 1, 0):
        x[r] = (M[r][3] - sum(M[r][k] * x[k] for k in range(r + 1, 3))) / M[r][r]
    return x


def segment_clearance(path: CPath, a: Vec, b: Vec, step: float = 0.004) -> float:
    """Smallest height of segment a-b above the actual ground."""
    n = max(2, int(_norm(_sub(b, a)) / step) + 1)
    best = math.inf
    for i in range(n):
        t = i / (n - 1)
        u = a[0] + t * (b[0] - a[0])
        z = a[1] + t * (b[1] - a[1])
        best = min(best, z - path.ground(u))
    return best


def clearances(path: CPath, pose: SidePose, p: RoverParams) -> dict:
    """How close the body and the suspension links get to the ground."""
    hx, hz = p.body_length / 2, p.body_height / 2
    cx, cz = p.body_center_x, p.body_center_z
    bl = pose.body_point((cx - hx, cz - hz))
    br = pose.body_point((cx + hx, cz - hz))
    fl = pose.body_point((cx + hx, cz + hz))
    body = min(segment_clearance(path, bl, br), segment_clearance(path, br, fl))
    links = min(segment_clearance(path, pose.P, pose.F),
                segment_clearance(path, pose.P, pose.B),
                segment_clearance(path, pose.B, pose.M),
                segment_clearance(path, pose.B, pose.R))
    return {'body': body, 'links': links}


def body_outline(pose: SidePose, p: RoverParams) -> List[Vec]:
    hx, hz = p.body_length / 2, p.body_height / 2
    cx, cz = p.body_center_x, p.body_center_z
    return [pose.body_point(q) for q in ((cx - hx, cz - hz), (cx + hx, cz - hz),
                                         (cx + hx, cz + hz), (cx - hx, cz + hz))]
