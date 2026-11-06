#!/usr/bin/env python3
"""Draw the alliance-coordination pit sheet (docs/POLLEN_PIT_SHEET.svg). Optional: needs matplotlib.
Usage: python3 tools/draw-pollen-pit-sheet.py [output.svg|.pdf|.png]
Uses the same nominal CAD numbers as tools/check-pollen-field-routes.py; run that script for the check.
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
from matplotlib.patches import Polygon, Rectangle

from pollen_field_drawing import *  # geometry, colours and helpers shared with the drive-team pictures

fig = plt.figure(figsize=(11, 8.5))
fig.patch.set_facecolor('white')
ax = fig.add_axes([.02, .06, .58, .86])
tx = fig.add_axes([.62, .04, .37, .9]); tx.axis('off')

draw_field(ax)
for blue in (False, True):
    for pts, color, tag in ((P, P_COL, 'P'), (S, S_COL, 'S')):
        xs, zs = zip(*[rot(p, blue) for p in pts])
        ax.plot(xs, zs, color=color, lw=2.4, marker='o', ms=3, zorder=7)
        robot(ax, pts[0], color, .35, blue, tag)
        robot(ax, pts[-1], color, .75, blue, 'IN' if tag == 'P' else 'OUT')

# Timeline badges on the red half; the blue half is identical, rotated.
badge(ax, (-5, 44), 'P: 0-0.5 s squares on wall\n0.5-3.5 s spin-up\n3.5-13.5 s shoot\n~13.5-19 s: 3 legs\nto INNER slot', P_COL)
badge(ax, (8, -40), 'S: 0-12 s wait\n12-15 s spin-up\n15-22.5 s shoot\n~22.5-26 s: 2 legs\nto OUTER slot', S_COL)
ax.text(-W + 2, -W - 2.5, 'RED', color=RED, fontsize=12, weight='bold', va='bottom')
ax.text(W - 2, -W - 2.5, 'BLUE', color=BLUE, fontsize=12, weight='bold', va='bottom', ha='right')
ax.text(0, W + 2.5, 'AUDIENCE END   (dashed line = alliance midline, a tile seam)', fontsize=8, ha='center', va='top')
ax.text(0, -W - 2.5, 'SCORING END', fontsize=8, ha='center', va='bottom')
ax.set_xlim(-W - 3, W + 3); ax.set_ylim(-W - 6, W + 6); ax.set_aspect('equal'); ax.axis('off')
ax.invert_yaxis()  # match-view convention: red left, audience (up cell) at the bottom, loading zones at the top
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
    ('Spacer gauges allowed (G302); remove before the match (G303).', ''),
    ('', ''),
    ('Partner timing', 'h'),
    ('Our S feeds from t=15 s: partner P must finish shooting by', ''),
    ('~14 s and clear its lane. S delay is an INIT row if needed.', ''),
    ('Our robots first back into their wall ~0.5 s to square up.', ''),
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
