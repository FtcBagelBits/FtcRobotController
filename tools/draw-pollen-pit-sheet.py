#!/usr/bin/env python3
"""Draw the alliance-partner sheet (docs/POLLEN_PIT_SHEET.svg): our two Match autos. Optional: needs matplotlib.
Usage: python3 tools/draw-pollen-pit-sheet.py [output.svg|.pdf|.png]
Routes are the pollen-hive-v44 defaults (unchanged since v38) (tools/pollen_field_drawing.py HIVE_*); times are the
2026-11-19 practice traces in runs/ (Primary v35-v38, 7 runs; Secondary v35, 2 runs).
Print the PDF/SVG landscape on letter paper; the engineering plot is docs/POLLEN_FIELD_ROUTES.svg."""
import os
import sys
sys.dont_write_bytecode = True  # no __pycache__ in tools/
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'  # keep text as text: small, diffable SVG
matplotlib.rcParams['svg.hashsalt'] = 'pollen-pit-sheet'  # stable element ids: regenerating gives no diff
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from pollen_field_drawing import *  # geometry, colours and helpers shared with the drive-team pictures


def lane(ax, pts, color, blue, dashed=False):
    xs, zs = zip(*[rot(p, blue) for p in pts])
    ax.plot(xs, zs, color=color, lw=2.6 if not dashed else 2.2, ls='--' if dashed else '-',
            marker='o', ms=3, zorder=7)
    (x0, z0), (x1, z1) = rot(pts[-2], blue), rot(pts[-1], blue)
    ax.annotate('', xy=(x1, z1), xytext=((x0 + x1) / 2, (z0 + z1) / 2),
                arrowprops=dict(arrowstyle='-|>', color=color, lw=2, mutation_scale=14), zorder=8)


fig = plt.figure(figsize=(11, 8.5))
fig.patch.set_facecolor('white')
ax = fig.add_axes([.01, .06, .55, .86])
tx = fig.add_axes([.575, .03, .415, .93]); tx.axis('off')

draw_field(ax)
for blue in (False, True):
    lane(ax, HIVE_P_OUT, P_COL, blue)
    lane(ax, HIVE_P_PARK, P_COL, blue, dashed=True)
    lane(ax, HIVE_S_OUT, S_COL, blue)
    lane(ax, HIVE_S_PARK, S_COL, blue, dashed=True)
    for b in GARDEN_BALLS:
        bx, bz = rot(b, blue)
        ax.add_patch(Circle((bx, bz), 1.5, fc='#f5d76e', ec='#8a5a00', lw=.8, zorder=6))
    robot(ax, HIVE_P_OUT[0], P_COL, .45, blue, 'P')
    robot(ax, HIVE_P_PARK[-1], P_COL, .8, blue, 'IN')
    robot(ax, HIVE_S_OUT[0], S_COL, .45, blue, 'S')
    robot(ax, HIVE_S_PARK[-1], S_COL, .8, blue, 'OUT')

# Timeline badges on the red half; the blue half is identical, rotated.
badge(ax, (-13, 37), 'P (Primary + Garden)\n1-4 s shoot 4\n4-15 s garden and back\n15-22 s shoot if our cell UP\n23.5 s go: INNER slot', P_COL)
badge(ax, (-35, -6), 'S (Secondary + Flower)\nwait for our cell UP, shoot 4\nflower, back, shoot again\n25.5 s go: OUTER slot', S_COL)
ax.text(-W + 2, -W - 2.5, 'RED', color=RED, fontsize=12, weight='bold', va='bottom')
ax.text(W - 2, -W - 2.5, 'BLUE', color=BLUE, fontsize=12, weight='bold', va='bottom', ha='right')
ax.text(0, W + 2.5, 'AUDIENCE END   (dashed line = alliance midline, a tile seam)', fontsize=8, ha='center', va='top')
ax.text(0, -W - 2.5, 'SCORING END', fontsize=8, ha='center', va='bottom')
ax.set_xlim(-W - 3, W + 3); ax.set_ylim(-W - 6, W + 6); ax.set_aspect('equal'); ax.axis('off')
ax.invert_yaxis()  # match-view convention: red left, audience (up cell) at the bottom, loading zones at the top
ax.set_title('Team 31452 autonomous: our two options   (solid = trip and return, dashed = park)', fontsize=10.5, pad=14)

lines = [
    ('Pick one with your partner', 'h'),
    ('P = shoots first.  S = shoots second, after the first tip.', ''),
    ('Both start with the FRONT on the end wall, launcher toward the hive.', ''),
    ('', ''),
    ('P  Primary Shooter + Garden  (we shoot first)', 'hp'),
    ('Start: red at the audience wall, blue at the scoring wall, 1 in from the', ''),
    ('center seam. Shoots 4 preloads at about 1-4 s into the cell that starts UP.', ''),
    ('4-15 s: drives to the garden corner, intake first, and back.', ''),
    ('15-22 s: shoots again only if that cell is UP again.', ''),
    ('Leaves at 23.5 s, parked in the INNER slot by about 28.5 s.', ''),
    ('From you: shoot the other cell after our tip; stay out of our', 'b'),
    ('garden lane until 16 s; park in the OUTER slot.', 'b'),
    ('', ''),
    ('S  Secondary Shooter + Flower  (you shoot first)', 'hs'),
    ('Start: red at the scoring wall, blue at the audience wall, between', ''),
    ('the flower and the center seam (1 in gaps).', ''),
    ('Waits with its launcher spinning until ITS cell is UP (your tip),', ''),
    ('then shoots 4. Drains the flower beside it, comes back, and shoots', ''),
    ('again right away (its cell needs 8 balls to tip, so it is still UP).', ''),
    ('Leaves at 25.5 s for the OUTER slot.', ''),
    ('From you: tip the starting UP cell as early as you can; stay off our', 'b'),
    ('flower lane; park in the INNER slot. No tip = we never shoot in auto.', 'b'),
    ('', ''),
    ('SWARM: 16 points', 'h'),
    ('Each robot LEAVE 3 + AUTO PARK 5; both robots = 16. End-of-match', ''),
    ('parks count too (5 each). PARK = partly in our loading zone. Touching', ''),
    ('the perimeter wall at the end of AUTO = no LEAVE.', ''),
    ('', ''),
    ('Tip math (field-check counts, not a promise)', 'h'),
    ('The starting UP cell holds 3 staged nectar: about 3 more pollen tips it.', ''),
    ('An empty cell holds 7 pollen; the 8th tips it. So after the first tip,', ''),
    ('4 preloads alone will not tip the other cell.', ''),
    ('', ''),
    ('Times measured in our Nov 19 practice (P 7 runs, S 2 runs), build v35-v38.', 's'),
    ('Routes: pollen-hive-v44 defaults, field CAD v26-27.2, 18 in robot.', 's'),
]
y = 1.0
colors = {'hp': P_COL, 'hs': S_COL}
for text, kind in lines:
    big = kind in ('h', 'hp', 'hs')
    tx.text(0, y, text, fontsize=11.5 if big else (7.8 if kind == 's' else 9.2),
            weight='bold' if big or kind == 'b' else 'normal', va='top',
            color=colors.get(kind, '#555' if kind == 's' else '#111'))
    y -= .031 if text else .016
out = sys.argv[1] if len(sys.argv) > 1 else 'docs/POLLEN_PIT_SHEET.svg'
fig.savefig(out, metadata={'Date': None} if out.endswith(('.svg', '.pdf')) else None)
if out.endswith('.svg'):  # matplotlib leaves trailing spaces in path data; keep git diff --check clean
    with open(out) as f:
        text = '\n'.join(line.rstrip() for line in f.read().splitlines()) + '\n'
    with open(out, 'w') as f:
        f.write(text)
