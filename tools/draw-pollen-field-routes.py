#!/usr/bin/env python3
"""Redraw docs/POLLEN_FIELD_ROUTES.svg from the nominal presets. Optional: needs matplotlib.
Usage: python3 tools/draw-pollen-field-routes.py [output.svg]
Geometry matches tools/check-pollen-field-routes.py; that script is the check, this is a picture."""
import math, sys
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'  # keep text as text: small, diffable SVG
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

R = 9; W = 70.585; hx = -12.74164098; IN = 2.54
start = (hx, W - R); off_wall = (hx, W - R - 10 / IN)
corner = (off_wall[0] - 25, off_wall[1]); primary = (-56, -26)
flower_x = -20.058; ss = (flower_x / 2, -W + R); sc = (ss[0], ss[1] + 8); support = (-56, -47)

def square(p):
    return [(p[0] - R, p[1] - R), (p[0] + R, p[1] - R), (p[0] + R, p[1] + R), (p[0] - R, p[1] + R)]

def leg(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1]) * IN

fig, (ax, tx) = plt.subplots(1, 2, figsize=(14, 9), gridspec_kw={'width_ratios': [1.05, 1]})
fig.patch.set_facecolor('#f7f8fa')
ax.set_facecolor('#ffffff')
ax.add_patch(Rectangle((-W, -W), W, 2 * W, color='#fbeeee', zorder=0))
ax.add_patch(Rectangle((0, -W), W, 2 * W, color='#edf2fa', zorder=0))
for k in range(-2, 3):
    ax.axvline(k * 24, color='#e3e7ed', lw=.7, zorder=1)
    ax.axhline(k * 24, color='#e3e7ed', lw=.7, zorder=1)
ax.axvline(0, color='#888', lw=1, ls='--', zorder=1)
ax.add_patch(Rectangle((-23.354, -25.071), 46.708, 50.142, fc='#d9d9d9', ec='#777', lw=.8, alpha=.6, zorder=2))
ax.add_patch(Rectangle((-24.729, -19.476), 49.471, 38.952, fill=False, ec='#444', lw=1.2, zorder=3))
ax.add_patch(Rectangle((-24.729, -19.472), 1.979, 38.944, fc='#444', zorder=3))
for x0, x1, z0, z1 in [(-71.794, -65.630, 20.058, 26.727), (65.630, 71.794, -26.727, -20.058),
                       (20.058, 26.727, 65.630, 71.794), (-26.727, flower_x, -71.794, -65.630)]:
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc='#f2b134', ec='#8a5a00', zorder=4))
ax.add_patch(Rectangle((-70.101, -46.599), 10.516, 22.692, fc='none', ec='#c0392b', lw=1.5, ls='-', zorder=3,
                       hatch='///'))
ax.text(-67.6, -35, 'park zone', color='#c0392b', ha='center', va='center', fontsize=7, rotation=90, zorder=8)

def route(points, color, label):
    xs, zs = zip(*points)
    ax.plot(xs, zs, color=color, lw=2.2, marker='o', ms=3, zorder=7, label=label)
    for p, alpha in [(points[0], .35), (points[-1], .55)]:
        ax.add_patch(Polygon(square(p), closed=True, fc=color, ec=color, alpha=alpha, zorder=5))

route([start, off_wall, corner, primary], '#1f6fb2', 'B Square: shoot, 10 cm off wall, left, diagonal')
route([ss, sc, support], '#2e8b57', 'C: shoot flush, forward clear of flower, right-forward')
ax.annotate('B start', start, (start[0] + 14, start[1] - 2), fontsize=8, arrowprops=dict(arrowstyle='-', lw=.6))
ax.annotate('C start (flush, even gaps)', ss, (ss[0] + 12, ss[1] + 14), fontsize=8,
            arrowprops=dict(arrowstyle='-', lw=.6))
ax.set_xlim(-W - 2, W + 2); ax.set_ylim(-W - 2, W + 2); ax.set_aspect('equal')
ax.invert_yaxis()  # match-view convention: red left, audience (up cell) at the bottom, loading zones at the top
ax.set_xlabel('X inches (+ = blue side)'); ax.set_ylabel('Z inches (+ = audience, at bottom)')
ax.set_title('Red alliance nominal presets (blue: rotate 180 degrees)', fontsize=11)
ax.legend(loc='upper right', fontsize=8, framealpha=.95)

tx.axis('off')
lines = [
    ('B Square field preset (Option B)', True),
    ('Start: audience wall, flush + square, centered on hive', False),
    ('1. Spin up 3 s, then shoot (1250 ticks/s, 10 s feed)', False),
    ('2. Leg 1: 10.0 cm @ 0 deg (off wall)', False),
    ('3. Leg 2: 63.5 cm @ +90 deg (left, clear frame)', False),
    ('4. Leg 3 plan: %.2f cm @ %+.2f deg to (-56, -26)' % (leg(corner, primary),
        math.degrees(math.atan2(corner[0] - primary[0], corner[1] - primary[1]))), False),
    ('   INIT default (field-tuned): 172.47 cm @ +17 deg', False),
    ('Closest obstacle ~8.52 in (side-wall flower)', False),
    ('', False),
    ('C supporting preset (Option C)', True),
    ('Start: scoring wall, flush + square; no aim yaw', False),
    ('Center X = %.3f in: %.2f cm gaps to flower and midline' % (ss[0], (ss[0] - R - flower_x) * IN), False),
    ('Robot centerline %.2f cm toward midline from hive center' % ((ss[0] - hx) * IN), False),
    ('1. Wait 12 s, spin up 3 s, shoot 7.5 s (15-22.5 s)', False),
    ('2. Leg 1: 20.32 cm @ 0 deg (clear flower)', False),
    ('3. Leg 2: 0 cm (optional; default -90 deg)', False),
    ('4. Leg 3 plan: %.2f cm @ %+.2f deg to (-56, -47)' % (leg(sc, support),
        -math.degrees(math.atan2(sc[0] - support[0], support[1] - sc[1]))), False),
    ('   INIT default (field-tuned): 97.96 cm @ -81.85 deg', False),
    ('', False),
    ('All launcher speeds, leg distances and angles are', False),
    ('INIT controller rows. Translation only; no turns.', False),
    ('Nominal CAD (STEP v26-27.2), 18 in filled square.', False),
    ('Not a physical measurement or a tested route.', False),
]
y = .97
for text, bold in lines:
    tx.text(0, y, text, fontsize=10.5, weight='bold' if bold else 'normal', va='top', family='DejaVu Sans')
    y -= .045
fig.tight_layout()
out = sys.argv[1] if len(sys.argv) > 1 else 'docs/POLLEN_FIELD_ROUTES.svg'
fig.savefig(out, metadata={'Date': None})
if out.endswith('.svg'):  # matplotlib leaves trailing spaces in path data; keep git diff --check clean
    with open(out) as f:
        text = '\n'.join(line.rstrip() for line in f.read().splitlines()) + '\n'
    with open(out, 'w') as f:
        f.write(text)
