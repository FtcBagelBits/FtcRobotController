#!/usr/bin/env python3
"""Draw the alliance-coordination pit sheet (docs/POLLEN_PIT_SHEET.svg). Optional: needs matplotlib.
Usage: python3 tools/draw-pollen-pit-sheet.py [output.svg|.pdf|.png]
Uses the same nominal CAD numbers as tools/check-pollen-field-routes.py; run that script for the check.
Print the PDF/SVG landscape on letter paper; the engineering plot is docs/POLLEN_FIELD_ROUTES.svg."""
import sys
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'  # keep text as text: small, diffable SVG
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle

R = 9; W = 70.585; hx = -12.74164098; IN = 2.54
SEAM = 23.502                                  # CAD tile pitch (soft-tile centers 11.751, 35.253, 58.755 in)
ZONE = (-70.101, -59.585, -46.599, -23.907)    # red loading zone: x0, x1, z0, z1 (conservative inboard edge)
flower_x = -20.058
# Red nominal presets (inches, field center origin). Blue is the same picture rotated 180 degrees.
P = [(hx, W - R), (hx, W - R - 10 / IN), (hx - 25, W - R - 10 / IN), (-56, -26)]       # B: shoot first
S = [(flower_x / 2, -W + R), (flower_x / 2, -W + R + 8), (-56, -47)]                   # C: shoot second
FLOWERS = [(-71.794, -65.630, 20.058, 26.727), (65.630, 71.794, -26.727, -20.058),
           (20.058, 26.727, 65.630, 71.794), (-26.727, flower_x, -71.794, -65.630)]
RED, BLUE, P_COL, S_COL = '#c0392b', '#1f5fa8', '#1f6fb2', '#2e8b57'

def rot(p, blue):
    return (-p[0], -p[1]) if blue else p

def rect(ax, x0, x1, z0, z1, blue=False, **kw):
    if blue:
        x0, x1, z0, z1 = -x1, -x0, -z1, -z0
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, **kw))

def robot(ax, p, color, alpha, blue, label=None):
    x, z = rot(p, blue)
    ax.add_patch(Polygon([(x - R, z - R), (x + R, z - R), (x + R, z + R), (x - R, z + R)],
                         closed=True, fc=color, ec=color, alpha=alpha, zorder=5))
    if label:
        ax.text(x, z, label, ha='center', va='center', fontsize=8, weight='bold', color='white', zorder=9)

def badge(ax, xy, text, color):
    ax.text(xy[0], xy[1], text, fontsize=7.2, ha='center', va='center', zorder=10, color='#1a1a1a',
            bbox=dict(boxstyle='round,pad=.25', fc='white', ec=color, lw=1.2))

fig = plt.figure(figsize=(11, 8.5))
fig.patch.set_facecolor('white')
ax = fig.add_axes([.02, .06, .58, .86])
tx = fig.add_axes([.62, .04, .37, .9]); tx.axis('off')

# Field: halves, tile seams, perimeter, hive, flowers.
ax.add_patch(Rectangle((-W, -W), W, 2 * W, color='#fdf1f0', zorder=0))
ax.add_patch(Rectangle((0, -W), W, 2 * W, color='#eef3fb', zorder=0))
for k in (-2, -1, 0, 1, 2):  # tile seams, kept inside the perimeter
    ax.plot([k * SEAM] * 2, [-W, W], color='#c9ced6', lw=.8, zorder=1)
    ax.plot([-W, W], [k * SEAM] * 2, color='#c9ced6', lw=.8, zorder=1)
ax.plot([0, 0], [-W, W], color='#555', lw=1.6, ls='--', zorder=2)
ax.add_patch(Rectangle((-W, -W), 2 * W, 2 * W, fill=False, ec='#222', lw=2, zorder=8))
ax.add_patch(Rectangle((-23.354, -25.071), 46.708, 50.142, fc='#dcdcdc', ec='#777', lw=.8, zorder=2))
ax.add_patch(Rectangle((-24.729, -19.476), 49.471, 38.952, fill=False, ec='#444', lw=1.2, zorder=3))
ax.text(0, 0, 'HIVE\n(frame foot blocks\na direct diagonal)', ha='center', va='center', fontsize=8, color='#444', zorder=4)
for f in FLOWERS:
    rect(ax, *f, fc='#f2b134', ec='#8a5a00', zorder=4)

for blue in (False, True):
    col = BLUE if blue else RED
    x0, x1, z0, z1 = ZONE
    rect(ax, x0, x1, z0, z1, blue, fc='none', ec=col, lw=1.8, hatch='///', zorder=3)
    # No-wall-contact strip: from the wall to the planned robot edge (5.6 in nominal).
    rect(ax, -W, -65, z0, z1, blue, fc='#fff200', ec='#b5a800', lw=.8, alpha=.85, zorder=4)
    sx, sz = rot((-67.8, (z0 + z1) / 2), blue)
    ax.text(sx, sz, 'NO WALL\nCONTACT', rotation=90, ha='center', va='center', fontsize=5.5, weight='bold', zorder=9)
    for pts, color, tag in ((P, P_COL, 'P'), (S, S_COL, 'S')):
        xs, zs = zip(*[rot(p, blue) for p in pts])
        ax.plot(xs, zs, color=color, lw=2.4, marker='o', ms=3, zorder=7)
        robot(ax, pts[0], color, .35, blue, tag)
        robot(ax, pts[-1], color, .75, blue, 'IN' if tag == 'P' else 'OUT')

# Timeline badges on the red half; the blue half is identical, rotated.
badge(ax, (-5, 44), 'P: 0-14 s shoot\n~10-24 s: 3 legs\nto INNER slot', P_COL)
badge(ax, (8, -40), 'S: 0-15 s wait\n15-22.5 s shoot\n~23-25.5 s: 2 legs\nto OUTER slot', S_COL)
ax.text(-W + 2, W + 2.5, 'RED', color=RED, fontsize=12, weight='bold', va='bottom')
ax.text(W - 2, W + 2.5, 'BLUE', color=BLUE, fontsize=12, weight='bold', va='bottom', ha='right')
ax.text(0, W + 2.5, 'AUDIENCE END', fontsize=8, ha='center', va='bottom')
ax.text(0, -W - 2.5, 'SCORING END', fontsize=8, ha='center', va='top')
ax.text(1.5, 28, 'alliance\nmidline\n(tile seam)', fontsize=6.5, va='bottom', color='#333')
ax.set_xlim(-W - 3, W + 3); ax.set_ylim(-W - 6, W + 6); ax.set_aspect('equal'); ax.axis('off')
ax.set_title('BIOBUZZ autonomous lanes - team 31452 (nominal, untested)', fontsize=11, pad=14)

lines = [
    ('Lane assignments', 'h'),
    ('P = shoots first.  S = shoots second (waits for the tip).', ''),
    ('Each alliance: P parks in the INNER slot (zone end toward', ''),
    ('midfield), S in the OUTER slot (zone end toward its wall).', ''),
    ('Red P starts at the audience wall; blue P at the scoring wall.', ''),
    ('', ''),
    ('Auto-SWARM: 16 points by end of AUTO', 'h'),
    ('Each robot: LEAVE 3 + AUTO PARK 5. Both robots = 16 =', ''),
    ('SWARM RP at events using the default threshold (Table 10-3).', ''),
    ('Judged at the END of AUTO: a robot touching the perimeter', ''),
    ('wall then gets no LEAVE. PARK = partly in the loading zone.', ''),
    ('Our slots stop 5.6 in off the wall and 5.4 in into the tape.', ''),
    ('', ''),
    ('Setup rules (G304)', 'h'),
    ('Touch a wall, own half, not in the loading zone, not', ''),
    ('touching a flower, exactly 4 pollen preloaded.', ''),
    ('P: square to wall; midline-side edge 3.7 in from center seam.', ''),
    ('S: square to wall; 1.0 in gap to flower AND to center seam.', ''),
    ('Spacer gauges are allowed (G302); remove before the match (G303).', ''),
    ('', ''),
    ('Partner timing', 'h'),
    ('Our S feeds from t=15 s: partner P must finish shooting by', ''),
    ('~14 s and clear its lane. S delay is an INIT row if needed.', ''),
    ('Park-only partner: take the opposite lane and slot; end', ''),
    ('partly in the zone and OFF the wall. From mid audience', ''),
    ('wall, do not drive the straight diagonal: it hits the frame.', ''),
    ('', ''),
    ('Drawn from FTC field CAD v26-27.2 with an 18 in robot.', 's'),
    ('Times are planned, not measured. Verify on a real field.', 's'),
]
y = 1.0
for text, kind in lines:
    tx.text(0, y, text, fontsize=11.5 if kind == 'h' else (8 if kind == 's' else 9.2),
            weight='bold' if kind == 'h' else 'normal', va='top', color='#555' if kind == 's' else '#111')
    y -= .037 if text else .02
out = sys.argv[1] if len(sys.argv) > 1 else 'docs/POLLEN_PIT_SHEET.svg'
fig.savefig(out, metadata={'Date': None} if out.endswith(('.svg', '.pdf')) else None)
if out.endswith('.svg'):  # matplotlib leaves trailing spaces in path data; keep git diff --check clean
    with open(out) as f:
        text = '\n'.join(line.rstrip() for line in f.read().splitlines()) + '\n'
    with open(out, 'w') as f:
        f.write(text)
