"""Just enough SVG to draw the climb figures without matplotlib."""

from __future__ import annotations

from typing import Iterable, List, Sequence, Tuple
from xml.sax.saxutils import escape

FONT = 'font-family="Helvetica, Arial, sans-serif"'


class Svg:
    def __init__(self, width: int, height: int, background: str = '#ffffff'):
        self.w, self.h = width, height
        self.parts: List[str] = [f'<rect width="{width}" height="{height}" fill="{background}"/>']

    def add(self, s: str) -> None:
        self.parts.append(s)

    def line(self, x1, y1, x2, y2, stroke='#333', width=1.0, dash: str | None = None, opacity=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ''
        self.add(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{stroke}" '
                 f'stroke-width="{width}" stroke-opacity="{opacity}"{d}/>')

    def polyline(self, pts: Iterable[Tuple[float, float]], stroke='#333', width=1.5,
                 fill='none', opacity=1.0, dash: str | None = None):
        d = f' stroke-dasharray="{dash}"' if dash else ''
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.add(f'<polyline points="{p}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" '
                 f'stroke-opacity="{opacity}" stroke-linejoin="round"{d}/>')

    def polygon(self, pts: Iterable[Tuple[float, float]], fill='#ccc', stroke='none', width=1.0, opacity=1.0):
        p = ' '.join(f'{x:.1f},{y:.1f}' for x, y in pts)
        self.add(f'<polygon points="{p}" fill="{fill}" stroke="{stroke}" stroke-width="{width}" '
                 f'fill-opacity="{opacity}"/>')

    def circle(self, cx, cy, r, fill='none', stroke='#333', width=1.0, opacity=1.0):
        self.add(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" '
                 f'stroke-width="{width}" fill-opacity="{opacity}" stroke-opacity="{opacity}"/>')

    def rect(self, x, y, w, h, fill='#eee', stroke='none', width=1.0, rx=0.0, opacity=1.0):
        self.add(f'<rect x="{x:.1f}" y="{y:.1f}" width="{w:.1f}" height="{h:.1f}" rx="{rx}" fill="{fill}" '
                 f'stroke="{stroke}" stroke-width="{width}" fill-opacity="{opacity}"/>')

    def text(self, x, y, s, size=12, fill='#222', anchor='start', weight='normal', rotate: float | None = None):
        rot = f' transform="rotate({rotate:.0f} {x:.1f} {y:.1f})"' if rotate is not None else ''
        self.add(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" fill="{fill}" text-anchor="{anchor}" '
                 f'font-weight="{weight}" {FONT}{rot}>{escape(s)}</text>')

    def save(self, path) -> None:
        body = '\n'.join(self.parts)
        with open(path, 'w') as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                    f'viewBox="0 0 {self.w} {self.h}">\n{body}\n</svg>\n')


class Axes:
    """A rectangular plot area mapping data coordinates to pixels."""

    def __init__(self, svg: Svg, x: float, y: float, w: float, h: float,
                 xlim: Sequence[float], ylim: Sequence[float]):
        self.svg, self.x, self.y, self.w, self.h = svg, x, y, w, h
        self.xlim, self.ylim = xlim, ylim

    def px(self, xv: float, yv: float) -> Tuple[float, float]:
        fx = (xv - self.xlim[0]) / (self.xlim[1] - self.xlim[0])
        fy = (yv - self.ylim[0]) / (self.ylim[1] - self.ylim[0])
        return self.x + fx * self.w, self.y + self.h - fy * self.h

    def frame(self, xlabel='', ylabel='', xticks: Sequence[float] = (), yticks: Sequence[float] = (),
              fmt='{:g}', title=''):
        s = self.svg
        s.rect(self.x, self.y, self.w, self.h, fill='#fafafa', stroke='#bbb')
        for t in xticks:
            x, _ = self.px(t, self.ylim[0])
            s.line(x, self.y, x, self.y + self.h, stroke='#e3e3e3')
            s.text(x, self.y + self.h + 14, fmt.format(t), size=10, anchor='middle', fill='#555')
        for t in yticks:
            _, y = self.px(self.xlim[0], t)
            s.line(self.x, y, self.x + self.w, y, stroke='#e3e3e3')
            s.text(self.x - 6, y + 3.5, fmt.format(t), size=10, anchor='end', fill='#555')
        if xlabel:
            s.text(self.x + self.w / 2, self.y + self.h + 30, xlabel, size=11, anchor='middle', fill='#333')
        if ylabel:
            s.text(self.x - 38, self.y + self.h / 2, ylabel, size=11, anchor='middle', fill='#333', rotate=-90)
        if title:
            s.text(self.x, self.y - 8, title, size=13, weight='bold')

    def plot(self, xs, ys, stroke='#1f77b4', width=1.8, dash=None):
        pts = []
        for xv, yv in zip(xs, ys):
            if yv is None or yv != yv or abs(yv) == float('inf'):
                if len(pts) > 1:
                    self.svg.polyline(pts, stroke=stroke, width=width, dash=dash)
                pts = []
                continue
            yv = min(max(yv, self.ylim[0]), self.ylim[1])
            pts.append(self.px(xv, yv))
        if len(pts) > 1:
            self.svg.polyline(pts, stroke=stroke, width=width, dash=dash)

    def hline(self, yv, stroke='#d62728', dash='5,4', label='', width=1.2):
        x1, y = self.px(self.xlim[0], yv)
        x2, _ = self.px(self.xlim[1], yv)
        self.svg.line(x1, y, x2, y, stroke=stroke, width=width, dash=dash)
        if label:
            self.svg.text(x2 - 4, y - 4, label, size=10, anchor='end', fill=stroke)

    def legend(self, items: Sequence[Tuple[str, str]], x=None, y=None):
        x = self.x + 10 if x is None else x
        y = self.y + 14 if y is None else y
        for i, (label, colour) in enumerate(items):
            yy = y + i * 15
            self.svg.line(x, yy - 4, x + 18, yy - 4, stroke=colour, width=2.5)
            self.svg.text(x + 24, yy, label, size=10, fill='#333')
