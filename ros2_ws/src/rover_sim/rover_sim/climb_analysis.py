"""Can the rover climb a step as tall as its wheels? Let's find out.

Drives the planar rocker-bogie model over a step, solves the forces at
every millimetre of travel, and reports what the motors and the tyres have
to deliver. It also runs a time-domain version at a fixed PWM duty, which
is how the firmware's climb mode actually drives the wheels.

    python3 -m rover_sim.climb_analysis                  # 65 mm step, writes docs/img/*.svg
    python3 -m rover_sim.climb_analysis --height 0.08    # try something taller
    ros2 run rover_sim climb_analysis --mu 0.6           # slippery floor

Only the standard library is needed.
"""

from __future__ import annotations

import argparse
import dataclasses
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence

from .params import RoverParams, load_params
from .statics import SideForces, gripping_torque, solve_side
from .suspension import SideGeometry, SidePose, SideSolver, body_outline, clearances
from .svg import Axes, Svg
from .terrain import CPath

STEP_AT = 0.30  # where the step starts, rover pivot starts at u = 0
PHASE_NAMES = {'...': 'rolling', 'F..': 'front wheel climbing', '.M.': 'middle wheel climbing',
               '..R': 'rear wheel climbing'}


@dataclass
class Sample:
    S: float
    rolled: float
    pose: SidePose
    forces: SideForces
    clearance: dict
    phase: str

    @property
    def torque(self) -> float:
        return gripping_torque(self.forces)


def step_path(p: RoverParams, height: float) -> CPath:
    return CPath([(STEP_AT, STEP_AT + 3.0, height)], p.wheel_radius, -0.45, STEP_AT + 1.2)


def phase_of(path: CPath, s: Sequence[float]) -> str:
    return ''.join('FMR'[i] if abs(path.tangent(x)[1]) > 0.05 else '.' for i, x in enumerate(s))


def run_quasi_static(p: RoverParams, height: float, ds: float = 0.001) -> List[Sample]:
    path = step_path(p, height)
    solver = SideSolver(SideGeometry.from_params(p))
    s = solver.initial(path, 0.0)
    S0 = S = sum(s) / 3
    # Stop once the rear wheel is a wheel radius past the edge.
    end_u = STEP_AT + 2 * p.wheel_radius
    out: List[Sample] = []
    while True:
        pose = solver.pose(path, s)
        out.append(Sample(S, S - S0, pose, solve_side(path, pose, p), clearances(path, pose, p),
                          phase_of(path, s)))
        if pose.R[0] > end_u or S - S0 > 1.5:
            break
        nxt = solver.advance(path, s, ds)
        if nxt is None:
            raise RuntimeError(f'suspension solver lost track at S={S:.3f}')
        s, S = nxt, S + ds
    return out


def bogie_ok(pose: SidePose, p: RoverParams) -> bool:
    return abs(pose.bogie_angle) <= p.bogie_limit


def feasible(samples: Sequence[Sample], p: RoverParams | None = None) -> bool:
    if p is not None and not all(bogie_ok(x.pose, p) for x in samples):
        return False
    return all(x.forces.ok for x in samples) and min_clearance(samples) > 0.0


def min_clearance(samples: Sequence[Sample]) -> float:
    return min(min(x.clearance.values()) for x in samples)


def mu_needed(p: RoverParams, height: float, lo=0.2, hi=2.0, iters=10) -> float:
    """Smallest tyre friction coefficient that gets the rover up the step."""
    def ok(mu):
        q = dataclasses.replace(p, mu_ground=mu)
        return feasible(run_quasi_static(q, height, ds=0.002), q)
    if not ok(hi):
        return math.inf
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


def time_climb(p: RoverParams, height: float, duty: float, dt: float = 0.005, t_max: float = 40.0) -> dict:
    """Drive over the step at a fixed duty on all six wheels."""
    path = step_path(p, height)
    solver = SideSolver(SideGeometry.from_params(p))
    s = solver.initial(path, 0.0)
    end_u = STEP_AT + 2 * p.wheel_radius
    t, log = 0.0, []
    w = p.no_load_speed(duty)
    while t < t_max:
        pose = solver.pose(path, s)
        if pose.R[0] > end_u:
            return {'done': True, 'time': t, 'log': log}
        f = solve_side(path, pose, p)
        if not f.ok:
            return {'done': False, 'time': t, 'log': log, 'why': f.reason or 'slips', 'phase': phase_of(path, s)}
        if min(clearances(path, pose, p).values()) < 0:
            return {'done': False, 'time': t, 'log': log, 'why': 'scrapes the step', 'phase': phase_of(path, s)}
        if not bogie_ok(pose, p):
            return {'done': False, 'time': t, 'log': log, 'why': 'bogie hits its stop', 'phase': phase_of(path, s)}
        tau = gripping_torque(f)
        w_target = p.wheel_speed(duty, tau)
        # First-order lag for wheel + rover inertia.
        w += (w_target - w) * min(1.0, dt / p.motor_time_constant)
        log.append((t, sum(s) / 3, pose.P[0], w, tau))
        if w <= 1e-4 and w_target <= 1e-4:
            return {'done': False, 'time': t, 'log': log, 'why': 'motors stall', 'phase': phase_of(path, s),
                    'torque': tau}
        nxt = solver.advance(path, s, w * p.wheel_radius * dt)
        if nxt is None:
            return {'done': False, 'time': t, 'log': log, 'why': 'solver lost track'}
        s = nxt
        t += dt
    return {'done': False, 'time': t, 'log': log, 'why': 'too slow'}


def min_duty(p: RoverParams, height: float) -> float:
    """Lowest constant duty that still gets over the step."""
    if not time_climb(p, height, 1.0)['done']:
        return math.inf
    lo, hi = 0.2, 1.0
    for _ in range(9):
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if time_climb(p, height, mid, dt=0.01)['done'] else (mid, hi)
    return hi


# ---------------------------------------------------------------- figures
COL = {'F': '#1f77b4', 'M': '#ff7f0e', 'R': '#2ca02c', 'rocker': '#3b5b92', 'bogie': '#c0621a'}
PHASE_FILL = {'F..': '#e7f0fa', '.M.': '#fdf0e3', '..R': '#e9f6e9'}


def _draw_rover(svg: Svg, ax: Axes, pose: SidePose, p: RoverParams, opacity=1.0):
    body = [ax.px(*q) for q in body_outline(pose, p)]
    svg.polygon(body, fill='#9aa7b8', stroke='#56606e', width=1.2, opacity=0.55 * opacity)
    k = ax.w / (ax.xlim[1] - ax.xlim[0])
    for a, b, c in ((pose.P, pose.F, COL['rocker']), (pose.P, pose.B, COL['rocker']),
                    (pose.B, pose.M, COL['bogie']), (pose.B, pose.R, COL['bogie'])):
        (x1, y1), (x2, y2) = ax.px(*a), ax.px(*b)
        svg.line(x1, y1, x2, y2, stroke=c, width=4, opacity=opacity)
    for c in (pose.F, pose.M, pose.R):
        x, y = ax.px(*c)
        svg.circle(x, y, p.wheel_radius * k, fill='#2b2b2b', stroke='#111', opacity=0.85 * opacity)
        svg.circle(x, y, p.wheel_radius * k * 0.45, fill='#f2c200', stroke='none', opacity=opacity)
    for q in (pose.P, pose.B):
        x, y = ax.px(*q)
        svg.circle(x, y, 3.2, fill='#fff', stroke='#222', width=1.2)


def _draw_ground(svg: Svg, ax: Axes, height: float):
    x0, y0 = ax.px(ax.xlim[0], 0)
    xs, _ = ax.px(STEP_AT, 0)
    _, yt = ax.px(STEP_AT, height)
    x1, _ = ax.px(ax.xlim[1], 0)
    yb = ax.y + ax.h
    svg.polygon([(x0, y0), (xs, y0), (xs, yt), (x1, yt), (x1, yb), (x0, yb)], fill='#d9c7a7', stroke='#8a7350')


def draw_filmstrip(samples: List[Sample], p: RoverParams, height: float, out: Path):
    picks = []
    first_touch = next(i for i, x in enumerate(samples) if x.phase != '...')
    picks.append((max(0, first_touch - 1), 'front wheel meets the step'))
    runs, cur = [], None
    for i, x in enumerate(samples):
        if x.phase != (cur[0] if cur else None):
            cur = [x.phase, i, i]
            runs.append(cur)
        cur[2] = i
    peaks = {}
    for ph, a, b in runs:
        if ph in PHASE_NAMES and ph != '...':
            # Draw the wheel halfway up the face, label with the worst torque of the phase.
            k = 'FMR'.index(ph.strip('.'))
            target = p.wheel_radius + 0.5 * height

            def wheel_z(i, k=k):
                pose = samples[i].pose
                return (pose.F, pose.M, pose.R)[k][1]
            i = min(range(a, b + 1), key=lambda j: abs(wheel_z(j) - target))
            ok = [j for j in range(a, b + 1) if samples[j].forces.ok]
            peaks[i] = max(samples[j].torque for j in ok) if ok else None
            picks.append((i, PHASE_NAMES[ph].replace('climbing', 'halfway up')))
    picks.append((len(samples) - 1, 'all six wheels up'))
    picks = picks[:6]
    cols, pw, ph_ = 3, 330, 190
    rows = (len(picks) + cols - 1) // cols
    svg = Svg(cols * pw + 20, rows * ph_ + 50)
    svg.text(14, 26, f'Climbing a {height * 1000:.0f} mm step (wheel diameter {2000 * p.wheel_radius:.0f} mm), '
             f'side view, mu = {p.mu_ground:.2f}', size=15, weight='bold')
    stall = p.stall_torque()
    for n, (i, label) in enumerate(picks):
        x = samples[i]
        r, c = divmod(n, cols)
        ox, oy = 10 + c * pw, 40 + r * ph_
        u_mid = x.pose.P[0]
        span = 0.62
        ax = Axes(svg, ox + 6, oy + 24, pw - 16, ph_ - 34,
                  (u_mid - span / 2, u_mid + span / 2), (-0.03, -0.03 + span * (ph_ - 34) / (pw - 16)))
        svg.rect(ax.x, ax.y, ax.w, ax.h, fill='#f4f8fc', stroke='#ccd')
        _draw_ground(svg, ax, height)
        _draw_rover(svg, ax, x.pose, p)
        svg.text(ox + 8, oy + 16, f'{n + 1}. {label}', size=12, weight='bold')
        if i in peaks:
            need = (f'worst moment of this phase: {peaks[i] / stall:.0%} of stall torque'
                    if peaks[i] is not None else 'slips')
        else:
            need = f'{x.torque / stall:.0%} of stall torque' if x.forces.ok else (x.forces.reason or 'slips')
        svg.text(ax.x + ax.w - 6, ax.y + 14, need, size=10, anchor='end', fill='#444')
    svg.save(out)


def draw_curves(samples: List[Sample], p: RoverParams, height: float, out: Path):
    svg = Svg(860, 535)
    xs = [x.rolled * 100 for x in samples]
    xmax = math.ceil(max(xs) / 5) * 5
    stall = p.stall_torque() * 1000
    top = Axes(svg, 70, 50, 760, 190, (0, xmax), (0, math.ceil(stall * 1.25 / 20) * 20))
    bot = Axes(svg, 70, 300, 760, 170, (0, xmax), (0, 1.2))
    for ax in (top, bot):
        a = None
        for i, x in enumerate(samples + [None]):
            ph = x.phase if x else None
            if a is not None and ph != samples[a].phase:
                fill = PHASE_FILL.get(samples[a].phase)
                if fill:
                    x0, _ = ax.px(xs[a], 0)
                    x1, _ = ax.px(xs[i - 1], 0)
                    svg.rect(x0, ax.y, max(1, x1 - x0), ax.h, fill=fill)
                a = i
            if a is None:
                a = i
    top.frame(ylabel='motor torque  [mN*m]', xticks=range(0, xmax + 1, 5),
              yticks=range(0, int(top.ylim[1]) + 1, 20),
              title='What each motor has to deliver (all six get the same PWM)')
    top.plot(xs, [x.torque * 1000 if x.forces.ok else None for x in samples], stroke='#222', width=2.2)
    top.hline(stall, label=f'stall torque at {p.motor_bus_voltage:.0f} V, 100% duty')
    top.hline(p.stall_torque(0.7) * 1000, stroke='#e08a00', label='stall torque at 70% duty')
    bot.frame(xlabel='distance the wheels have rolled  [cm]', ylabel='grip needed  (|T| / N)',
              xticks=range(0, xmax + 1, 5), yticks=[0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2], fmt='{:g}',
              title='How much tyre grip each wheel needs')
    for k, name in enumerate('FMR'):
        bot.plot(xs, [min(x.forces.mu_required[k], 5) if x.forces.ok else None for x in samples],
                 stroke=COL[name], width=1.8)
    bot.hline(p.mu_ground, label=f'grip available (mu = {p.mu_ground:.2f})')
    bot.legend([('front wheel', COL['F']), ('middle wheel', COL['M']), ('rear wheel', COL['R'])],
               x=bot.x + 12, y=bot.y + 16)
    svg.text(70, 524, 'Shaded bands: blue = front wheel on the step face, orange = middle, green = rear.',
             size=10, fill='#666')
    svg.save(out)


def draw_sweep(rows, p: RoverParams, out: Path):
    svg = Svg(860, 300)
    hs = [r['height'] * 1000 for r in rows]
    left = Axes(svg, 70, 40, 340, 200, (min(hs), max(hs)), (0, 1.6))
    right = Axes(svg, 500, 40, 340, 200, (min(hs), max(hs)), (0, 120))
    left.frame(xlabel='step height [mm]', ylabel='grip needed (mu)', xticks=hs[::2],
               yticks=[0, 0.4, 0.8, 1.2, 1.6], title='Tyre grip needed to get up')
    right.frame(xlabel='step height [mm]', ylabel='% of stall torque', xticks=hs[::2],
                yticks=[0, 20, 40, 60, 80, 100, 120], title='Peak motor torque on the way up')
    left.plot(hs, [r['mu'] if r['mu'] < 9 else None for r in rows], stroke='#1f77b4', width=2.2)
    left.hline(p.mu_ground, label=f'mu = {p.mu_ground:.2f}')
    right.plot(hs, [r['torque_pct'] if r['torque_pct'] == r['torque_pct'] else None for r in rows],
               stroke='#222', width=2.2)
    right.hline(100, label='stall at 5 V')
    wh = 2000 * p.wheel_radius
    for ax in (left, right):
        x, _ = ax.px(wh, 0)
        svg.line(x, ax.y, x, ax.y + ax.h, stroke='#888', dash='3,3')
        svg.text(x + 3, ax.y + 12, 'wheel diameter', size=9, fill='#666')
    for r, h in zip(rows, hs):
        colour = '#2ca02c' if r['climbs'] else '#d62728'
        if r['mu'] < 9:
            x, y = left.px(h, min(r['mu'], 1.6))
            svg.circle(x, y, 3.5, fill=colour, stroke='none')
    svg.text(70, 290, 'Green dots: the model says it climbs with the tyres and motors we have. Red: it does not.',
             size=10, fill='#666')
    svg.save(out)


# ---------------------------------------------------------------- report
def describe(p: RoverParams, height: float, duty: float) -> dict:
    samples = run_quasi_static(p, height)
    ok_samples = [x for x in samples if x.forces.ok]
    peak = max(ok_samples, key=lambda x: x.torque) if ok_samples else None
    run = time_climb(p, height, duty)
    return {'samples': samples, 'peak': peak, 'feasible': feasible(samples, p),
            'clearance': min_clearance(samples), 'run': run}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--height', type=float, default=0.065, help='step height in metres (default 0.065)')
    ap.add_argument('--mu', type=float, default=None, help='tyre friction coefficient (default from yaml)')
    ap.add_argument('--duty', type=float, default=1.0, help='PWM duty used in climb mode (default 1.0)')
    ap.add_argument('--out', type=Path, default=None, help='folder for SVG figures (default docs/img)')
    ap.add_argument('--no-figures', action='store_true')
    ap.add_argument('--quick', action='store_true', help='skip the height sweep and what-ifs')
    ap.add_argument('--geometry', type=Path, default=None, help='path to rover_geometry.yaml')
    a = ap.parse_args(argv)

    p = load_params(a.geometry)
    if a.mu is not None:
        p = dataclasses.replace(p, mu_ground=a.mu)
    H = a.height

    print(f'Rover: {p.total_mass:.2f} kg, wheels {2000 * p.wheel_radius:.0f} mm, '
          f'motors {p.motor_bus_voltage:.1f} V bus')
    print(f'  one motor at 100% duty: stall {p.stall_torque() * 1000:.0f} mN*m '
          f'({p.stall_torque() / 0.0981:.2f} kgf*cm), no-load {p.no_load_speed() * p.wheel_radius:.2f} m/s')

    d = describe(p, H, a.duty)
    peak = d['peak']
    print(f'\nStep {H * 1000:.0f} mm, mu = {p.mu_ground:.2f}')
    if peak:
        print(f'  peak motor torque  {peak.torque * 1000:5.1f} mN*m = {peak.torque / p.stall_torque():.0%} of stall '
              f'({PHASE_NAMES.get(peak.phase, peak.phase)})')
    print(f'  closest scrape     {d["clearance"] * 1000:5.1f} mm (body or suspension arm to the step)')
    mu = mu_needed(p, H)
    print(f'  grip needed        mu >= {mu:.2f}' if mu < 9 else '  grip needed        more than any tyre has')
    run = d['run']
    if run['done']:
        log = run['log']
        print(f'  at {a.duty:.0%} duty it gets over in {run["time"]:.1f} s; slowest wheel speed '
              f'{min(x[3] for x in log) * p.wheel_radius * 100:.1f} cm/s')
    else:
        print(f'  at {a.duty:.0%} duty it gets stuck: {run.get("why")} '
              f'({PHASE_NAMES.get(run.get("phase", ""), run.get("phase", ""))})')
    md = min_duty(p, H)
    print(f'  lowest duty that still makes it: {md:.0%}' if md < 9 else '  no duty is enough')
    print(f'  verdict: {"CLIMBS" if d["feasible"] and run["done"] else "DOES NOT CLIMB"}')

    sweep = []
    if not a.quick:
        print('\nHeight sweep (same rover, same tyres):')
        print('  height   grip needed   peak torque   result')
        for mm in range(20, 111, 5):
            h = mm / 1000
            q = run_quasi_static(p, h, ds=0.002)
            okq = [x for x in q if x.forces.ok]
            pct = max(x.torque for x in okq) / p.stall_torque() * 100 if okq else float('nan')
            m = mu_needed(p, h, iters=8)
            climbs = feasible(q, p) and time_climb(p, h, 1.0, dt=0.01)['done']
            sweep.append({'height': h, 'mu': m if m < 9 else 99, 'torque_pct': pct, 'climbs': climbs})
            print(f'  {mm:4d} mm   {m:9.2f}     {pct:8.0f} %     {"climbs" if climbs else "no"}')

        print('\nWhat moves the needle on the 65 mm step:')
        variants = [
            ('as designed', p),
            ('one 8xAA pack instead of two (-0.21 kg)', dataclasses.replace(p, mass_body=p.mass_body - 0.21)),
            ('motors on 6 V instead of 5 V', dataclasses.replace(p, motor_bus_voltage=6.0)),
            ('slippery floor, mu = 0.6', dataclasses.replace(p, mu_ground=0.6)),
            ('grippy surface, mu = 1.0', dataclasses.replace(p, mu_ground=1.0)),
            ('batteries 2 cm further forward', dataclasses.replace(p, body_cg_x=p.body_cg_x + 0.02)),
            ('batteries 2 cm further back', dataclasses.replace(p, body_cg_x=p.body_cg_x - 0.02)),
        ]
        for name, v in variants:
            q = run_quasi_static(v, 0.065, ds=0.002)
            okq = [x for x in q if x.forces.ok]
            pct = max(x.torque for x in okq) / v.stall_torque() if okq else float('nan')
            md_v = min_duty(v, 0.065)
            res = 'climbs' if feasible(q, v) and md_v < 9 else 'stuck'
            print(f'  {name:44s} peak {pct:4.0%} of stall, min duty '
                  f'{(f"{md_v:.0%}" if md_v < 9 else "  - "):>4}  {res}')

    if not a.no_figures:
        out = a.out
        if out is None:
            here = Path(__file__).resolve()
            out = next((q / 'docs' / 'img' for q in here.parents if (q / 'docs').is_dir()), Path('.'))
        out.mkdir(parents=True, exist_ok=True)
        tag = f'{H * 1000:.0f}mm'
        draw_filmstrip(d['samples'], p, H, out / f'climb_{tag}_filmstrip.svg')
        draw_curves(d['samples'], p, H, out / f'climb_{tag}_forces.svg')
        if sweep:
            draw_sweep(sweep, p, out / 'climb_height_sweep.svg')
        print(f'\nFigures written to {out}')


if __name__ == '__main__':
    main()
