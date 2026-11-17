"""Quasi-static forces on one side of the rocker-bogie.

The rover crawls slowly, so we ignore inertia and ask: in this pose, what
ground forces and wheel torques keep it in balance?

Unknowns per side: normal force N and traction T at each of the three
wheel contacts. Equations:

1-2. forces balance in x and z
3.   moments balance for the whole side (about the rocker pivot)
4.   moments balance for the bogie on its own (it pivots freely on its pin)
5-6. all three motors make the same torque

The last two come from the firmware: every wheel gets the same PWM duty and
they turn at nearly the same speed, so a DC motor gives nearly the same
torque. If a wheel would need more grip than the tyre has, it slips: its
traction is capped at mu*N and the other wheels share the work. If even
that doesn't balance, the rover can't make progress here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from .params import RoverParams
from .suspension import SideGeometry, SidePose
from .terrain import CPath

G = 9.81


@dataclass
class SideForces:
    N: Tuple[float, float, float]
    T: Tuple[float, float, float]
    torque: Tuple[float, float, float]   # motor torque each wheel needs, N*m
    mu_required: Tuple[float, float, float]
    slipping: Tuple[bool, bool, bool]
    ok: bool
    reason: str = ''
    contact_normals: List[Tuple[float, float]] = field(default_factory=list)

    @property
    def max_torque(self) -> float:
        return max(abs(t) for t in self.torque)


def _cross(a, b) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _solve(A: List[List[float]], b: List[float]) -> List[float]:
    n = len(b)
    M = [A[i][:] + [b[i]] for i in range(n)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            raise ZeroDivisionError('singular force system')
        M[c], M[p] = M[p], M[c]
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            if f:
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def side_masses(pose: SidePose, p: RoverParams):
    """(mass, position) pairs: whole side, and the bogie sub-assembly."""
    geo = SideGeometry.from_params(p)
    rocker_cg = ((geo.P[0] + geo.F[0] + geo.B[0]) / 3, (geo.P[1] + geo.F[1] + geo.B[1]) / 3)
    rocker_side = [
        (p.mass_body / 2, pose.body_point((p.body_cg_x, p.body_cg_z))),
        (p.mass_rocker, pose.body_point(rocker_cg)),
        (p.mass_wheel_motor, pose.F),
    ]
    bogie_side = [
        (p.mass_bogie, pose.B),
        (p.mass_wheel_motor, pose.M),
        (p.mass_wheel_motor, pose.R),
    ]
    return rocker_side + bogie_side, bogie_side


def solve_side(path: CPath, pose: SidePose, p: RoverParams, direction: float = 1.0) -> SideForces:
    r = p.wheel_radius
    mu = p.mu_ground
    crr = p.rolling_resistance * (1.0 if direction >= 0 else -1.0)
    centres = (pose.F, pose.M, pose.R)
    normals = [path.normal(s) for s in pose.s]
    tangents = [path.tangent(s) for s in pose.s]
    contacts = [(c[0] - r * n[0], c[1] - r * n[1]) for c, n in zip(centres, normals)]

    every, bogie = side_masses(pose, p)
    W = sum(m for m, _ in every) * G
    mom_P = sum((q[0] - pose.P[0]) * m * G for m, q in every)
    mom_B = sum((q[0] - pose.B[0]) * m * G for m, q in bogie)

    def base_rows():
        rows, rhs = [], []
        # x and z force balance: columns are N_F, N_M, N_R, T_F, T_M, T_R
        rows.append([normals[i][0] for i in range(3)] + [tangents[i][0] for i in range(3)])
        rhs.append(0.0)
        rows.append([normals[i][1] for i in range(3)] + [tangents[i][1] for i in range(3)])
        rhs.append(W)
        arm = [(c[0] - pose.P[0], c[1] - pose.P[1]) for c in contacts]
        rows.append([_cross(arm[i], normals[i]) for i in range(3)]
                    + [_cross(arm[i], tangents[i]) for i in range(3)])
        rhs.append(mom_P)
        armB = [(c[0] - pose.B[0], c[1] - pose.B[1]) for c in contacts]
        row = [0.0] * 6
        for i in (1, 2):
            row[i] = _cross(armB[i], normals[i])
            row[3 + i] = _cross(armB[i], tangents[i])
        rows.append(row)
        rhs.append(mom_B)
        return rows, rhs

    saturated: dict[int, float] = {}  # wheel index -> sign of its traction
    for _ in range(4):
        rows, rhs = base_rows()
        free = [i for i in range(3) if i not in saturated]
        if not free:
            break
        for a, b in zip(free, free[1:]):  # equal motor torque among gripping wheels
            row = [0.0] * 6
            row[3 + a], row[3 + b] = 1.0, -1.0
            row[a], row[b] = crr, -crr
            rows.append(row)
            rhs.append(0.0)
        for i, sgn in saturated.items():  # slipping wheels: T = +-mu*N
            row = [0.0] * 6
            row[3 + i] = 1.0
            row[i] = -mu * sgn
            rows.append(row)
            rhs.append(0.0)
        try:
            x = _solve(rows, rhs)
        except ZeroDivisionError:
            return SideForces((0, 0, 0), (0, 0, 0), (0, 0, 0), (math.inf,) * 3,
                              (True,) * 3, False, 'singular', normals)
        N, T = x[:3], x[3:]
        mu_req = [abs(T[i]) / N[i] if N[i] > 1e-9 else math.inf for i in range(3)]
        worst = max(free, key=lambda i: mu_req[i])
        if mu_req[worst] > mu + 1e-9 and N[worst] > 0:
            saturated[worst] = 1.0 if T[worst] >= 0 else -1.0
            continue
        torque = tuple(r * (T[i] + crr * N[i]) for i in range(3))
        lifted = [i for i in range(3) if N[i] < 0]
        reason = ''
        if lifted:
            reason = 'wheel lifts off (' + ','.join('FMR'[i] for i in lifted) + ')'
        return SideForces(tuple(N), tuple(T), torque, tuple(mu_req),
                          tuple(i in saturated for i in range(3)),
                          not lifted, reason, normals)
    return SideForces((0, 0, 0), (0, 0, 0), (0, 0, 0), (math.inf,) * 3,
                      (True,) * 3, False, 'all wheels slip', normals)


def gripping_torque(f: SideForces) -> float:
    """Torque the gripping (non-slipping) motors have to make."""
    vals = [f.torque[i] for i in range(3) if not f.slipping[i]]
    return max(vals, key=abs) if vals else 0.0
