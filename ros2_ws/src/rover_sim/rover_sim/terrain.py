"""Terrain made of boxes, and the path a wheel centre follows over it.

The world is flat ground (z = 0) with box-shaped obstacles on it: rocks,
curbs, books, a step. For the suspension model we look at the terrain along
one line (the track of the left or right wheels) which turns it into a 1-D
height profile made of flat pieces.

A wheel of radius r rolling over that profile keeps its centre on a curve
that sits r above the ground, wraps around convex corners with radius r, and
goes straight up wherever the profile has a wall taller than r. That curve is
called the configuration-space path (``CPath`` here). Every wheel's centre
lies on it, which turns the suspension into a simple geometry problem.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class Box:
    """An obstacle: rectangle in plan view, extruded up to ``height``."""

    cx: float
    cy: float
    length: float  # along x before rotation
    width: float   # along y before rotation
    height: float
    yaw: float = 0.0
    name: str = ''

    def contains_xy(self, x: float, y: float) -> bool:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = x - self.cx, y - self.cy
        lx = c * dx + s * dy
        ly = -s * dx + c * dy
        return abs(lx) <= self.length / 2 and abs(ly) <= self.width / 2

    def line_interval(self, px: float, py: float, dx: float, dy: float) -> Tuple[float, float] | None:
        """Where the line p + u*d crosses this box in plan view (slab test)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = px - self.cx, py - self.cy
        lpx, lpy = c * ox + s * oy, -s * ox + c * oy
        ldx, ldy = c * dx + s * dy, -s * dx + c * dy
        lo, hi = -math.inf, math.inf
        for p, d, half in ((lpx, ldx, self.length / 2), (lpy, ldy, self.width / 2)):
            if abs(d) < 1e-12:
                if abs(p) > half:
                    return None
                continue
            t1, t2 = (-half - p) / d, (half - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            lo, hi = max(lo, t1), min(hi, t2)
            if lo > hi:
                return None
        return lo, hi

    def ray_hit(self, o: Sequence[float], d: Sequence[float]) -> float | None:
        """Distance along a 3-D ray to the first hit on this box, if any."""
        hit = self.ray_hit_cos(o, d)
        return None if hit is None else hit[0]

    def ray_hit_cos(self, o: Sequence[float], d: Sequence[float]) -> Tuple[float, float] | None:
        """(distance, |cos| of the angle between the ray and the face it hits)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        ox, oy = o[0] - self.cx, o[1] - self.cy
        lo_ = (c * ox + s * oy, -s * ox + c * oy, o[2] - self.height / 2)
        ld = (c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2])
        half = (self.length / 2, self.width / 2, self.height / 2)
        t_near, t_far, axis = -math.inf, math.inf, 0
        for k, (p, dd, h) in enumerate(zip(lo_, ld, half)):
            if abs(dd) < 1e-12:
                if abs(p) > h:
                    return None
                continue
            t1, t2 = (-h - p) / dd, (h - p) / dd
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > t_near:
                t_near, axis = t1, k
            t_far = min(t_far, t2)
            if t_near > t_far:
                return None
        if t_far < 0:
            return None
        norm = math.sqrt(sum(x * x for x in ld)) or 1.0
        return (t_near if t_near >= 0 else 0.0), abs(ld[axis]) / norm


class World:
    def __init__(self, boxes: Iterable[Box] = ()):
        self.boxes: List[Box] = list(boxes)

    def height_at(self, x: float, y: float) -> float:
        h = 0.0
        for b in self.boxes:
            if b.height > h and b.contains_xy(x, y):
                h = b.height
        return h

    def profile(self, px: float, py: float, heading: float,
                u_min: float, u_max: float) -> List[Tuple[float, float, float]]:
        """Blocks (u_start, u_end, height) met along a line through (px, py)."""
        dx, dy = math.cos(heading), math.sin(heading)
        blocks = []
        for b in self.boxes:
            hit = b.line_interval(px, py, dx, dy)
            if hit is None:
                continue
            a, e = max(hit[0], u_min), min(hit[1], u_max)
            if e > a:
                blocks.append((a, e, b.height))
        return blocks

    def raycast(self, origin: Sequence[float], direction: Sequence[float],
                max_range: float) -> float:
        best = max_range
        if direction[2] < -1e-9:  # the ground plane
            t = -origin[2] / direction[2]
            if 0 <= t < best:
                best = t
        for b in self.boxes:
            t = b.ray_hit(origin, direction)
            if t is not None and t < best:
                best = t
        return best


class CPath:
    """Polyline traced by a wheel centre over a 1-D block profile.

    Coordinates are (u, z): u along the travel line, z up. The polyline is
    monotone in u, with exactly vertical pieces where the wheel has to climb
    or drop down a wall.
    """

    def __init__(self, blocks: Sequence[Tuple[float, float, float]], radius: float,
                 u_min: float, u_max: float, resolution: float = 0.001):
        self.blocks = [b for b in blocks if b[2] > 0.0]
        self.r = radius
        self.u_min, self.u_max = u_min, u_max
        r = radius

        breaks = set()
        for a, e, _ in self.blocks:
            for u in (a - r, a, e, e + r):
                if u_min < u < u_max:
                    breaks.add(u)
        n = max(2, int(math.ceil((u_max - u_min) / resolution)) + 1)
        grid = [u_min + (u_max - u_min) * i / (n - 1) for i in range(n)]
        us = sorted(set(grid) | breaks)

        eps = 1e-7
        pts: List[Tuple[float, float]] = []
        for u in us:
            if u in breaks:
                zl, zr = self.height(u - eps), self.height(u + eps)
                z0 = self.height(u)
                for z in (zl, z0, zr):
                    if not pts or abs(pts[-1][1] - z) > 1e-9 or abs(pts[-1][0] - u) > 1e-12:
                        pts.append((u, z))
            else:
                pts.append((u, self.height(u)))
        # Drop zero-length pieces so tangents are always defined.
        clean = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-9:
                clean.append(p)
        self.u = [p[0] for p in clean]
        self.z = [p[1] for p in clean]
        self.s = [0.0]
        for i in range(1, len(clean)):
            self.s.append(self.s[-1] + math.hypot(self.u[i] - self.u[i - 1], self.z[i] - self.z[i - 1]))

    # Height of the wheel centre when it sits at u (upper envelope).
    def height(self, u: float) -> float:
        r = self.r
        best = r
        for a, e, h in self.blocks:
            d = a - u if u < a else (u - e if u > e else 0.0)
            if d <= r:
                z = h + math.sqrt(max(0.0, r * r - d * d))
                if z > best:
                    best = z
        return best

    def ground(self, u: float) -> float:
        """Actual terrain height (not the wheel-centre path) at u."""
        h = 0.0
        for a, e, hb in self.blocks:
            if a <= u <= e and hb > h:
                h = hb
        return h

    @property
    def length(self) -> float:
        return self.s[-1]

    def _seg(self, s: float) -> int:
        i = bisect.bisect_right(self.s, s) - 1
        return min(max(i, 0), len(self.s) - 2)

    def point(self, s: float) -> Tuple[float, float]:
        i = self._seg(s)
        ds = self.s[i + 1] - self.s[i]
        t = (s - self.s[i]) / ds
        return (self.u[i] + t * (self.u[i + 1] - self.u[i]),
                self.z[i] + t * (self.z[i + 1] - self.z[i]))

    def tangent(self, s: float) -> Tuple[float, float]:
        i = self._seg(s)
        du, dz = self.u[i + 1] - self.u[i], self.z[i + 1] - self.z[i]
        n = math.hypot(du, dz)
        return du / n, dz / n

    def normal(self, s: float) -> Tuple[float, float]:
        tu, tz = self.tangent(s)
        return -tz, tu  # left of the travel direction = away from the ground

    def s_at_u(self, u: float) -> float:
        """Arc length at the first point of the path with this u."""
        i = bisect.bisect_left(self.u, u)
        i = min(max(i, 1), len(self.u) - 1)
        du = self.u[i] - self.u[i - 1]
        if du <= 0:
            return self.s[i - 1]
        t = (u - self.u[i - 1]) / du
        return self.s[i - 1] + t * (self.s[i] - self.s[i - 1])

    def project(self, u: float, z: float, window: float = 0.15) -> float:
        """Arc length of the path point closest to (u, z)."""
        lo = max(0, bisect.bisect_left(self.u, u - window) - 1)
        hi = min(len(self.u) - 1, bisect.bisect_right(self.u, u + window) + 1)
        best_s, best_d = self.s[lo], math.inf
        for i in range(lo, max(lo + 1, hi)):
            if i + 1 >= len(self.u):
                break
            au, az = self.u[i], self.z[i]
            bu, bz = self.u[i + 1], self.z[i + 1]
            vu, vz = bu - au, bz - az
            L2 = vu * vu + vz * vz
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((u - au) * vu + (z - az) * vz) / L2))
            pu, pz = au + t * vu, az + t * vz
            d = (pu - u) ** 2 + (pz - z) ** 2
            if d < best_d:
                best_d, best_s = d, self.s[i] + t * (self.s[i + 1] - self.s[i])
        return best_s
