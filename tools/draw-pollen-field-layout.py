#!/usr/bin/env python3
"""Draw docs/POLLEN_FIELD_LAYOUT.svg: permanent BIOBUZZ field features in field coordinates.
Usage: python3 tools/draw-pollen-field-layout.py [output.svg|.pdf|.png]   (needs matplotlib)
Numbers are the official STEP v26-27.2 extraction tabulated in docs/POLLEN_FIELD_LAYOUT.md.
Match view: audience at the bottom, red on the left. Initial (pre-tip) hive state."""
import sys
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'  # keep text as text: small, diffable SVG
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyArrowPatch

W = 70.585                       # inner top-rail plane
SEAMS = (-47.0, -23.5, 0.0, 23.5, 47.0)
RED, BLUE, INK, QUIET = '#c0392b', '#1f5fa8', '#111111', '#666666'
TAPE = [  # name, colour, x0, x1, z0, z1 (outer tape edges)
    ('red loading zone', RED, -70.101, -59.101, -46.599, -23.907),
    ('blue loading zone', BLUE, 59.101, 70.101, 23.907, 46.599),
    ('red garden', RED, -70.101, -47.409, 68.101, 70.101),
    ('blue garden', BLUE, 47.409, 70.101, -70.101, -68.101)]
FLOWERS = [(-23.44, -68.01, -26.727, -20.058, -71.794, -65.630), (-68.01, 23.44, -71.794, -65.630, 20.058, 26.727),
           (23.44, 68.01, 20.058, 26.727, 65.630, 71.794), (68.01, -23.44, 65.630, 71.794, -26.727, -20.058)]
CELLS = [  # x0, x1, z0, z1, colour, state, tag ids, label z (clear of the staged nectar)
    (-23.27, -2.21, 1.67, 20.06, RED, 'UP', '34-37', 16), (-23.27, -2.21, -25.07, -2.72, RED, 'down', 'Fig 9-17', -14),
    (2.16, 23.35, 2.72, 25.07, BLUE, 'down', '38-41', 14), (2.16, 23.35, -20.06, -1.67, BLUE, 'UP', 'Fig 9-17', -16)]

fig, ax = plt.subplots(figsize=(10, 10.6))
ax.add_patch(Rectangle((-W, -W), W, 2 * W, color='#fdf1f0', zorder=0))
ax.add_patch(Rectangle((0, -W), W, 2 * W, color='#eef3fb', zorder=0))
for s in SEAMS:
    ax.plot([s, s], [-W, W], color='#c3c8d0', lw=.8, zorder=1)
    ax.plot([-W, W], [s, s], color='#c3c8d0', lw=.8, zorder=1)
ax.plot([0, 0], [-W, W], color='#555', lw=1.4, ls='--', zorder=2)
ax.add_patch(Rectangle((-W, -W), 2 * W, 2 * W, fill=False, ec='#222', lw=2.2, zorder=9))
# Tile names: columns A-F red to blue, rows 1-6 audience to scoring (Field Acceptance 3.5/3.6).
cols = [(-58.79, 'A'), (-35.25, 'B'), (-11.75, 'C'), (11.75, 'D'), (35.25, 'E'), (58.79, 'F')]
rows = [(58.79, '1'), (35.25, '2'), (11.75, '3'), (-11.75, '4'), (-35.25, '5'), (-58.79, '6')]
for x, c in cols:
    for z, r in rows:
        ax.text(x + 9.5, z + 9.5, c + r, fontsize=8, color='#9aa1ab', ha='right', va='bottom', zorder=2)
for name, col, x0, x1, z0, z1 in TAPE:
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=col, alpha=.18, ec=col, lw=1.6, hatch='///', zorder=3))
# Frame (projection), foot bars, hive cells in the CAD's initial state.
ax.add_patch(Rectangle((-24.729, -19.476), 49.471, 38.952, fill=False, ec='#444', lw=1.2, ls=':', zorder=4))
for x0 in (-24.73, 22.75):
    ax.add_patch(Rectangle((x0, -19.47), 1.98, 38.94, fc='#444', zorder=5))
for x0, x1, z0, z1, col, state, tags, lz in CELLS:
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=col, alpha=.28 if state == 'UP' else .1,
                           ec=col, lw=1.6 if state == 'UP' else 1, zorder=4))
    ax.text((x0 + x1) / 2, lz, f'{state} cell, tags {tags}', ha='center', va='center', fontsize=8, color=INK, zorder=8,
            bbox=dict(boxstyle='round,pad=.15', fc='white', ec='none', alpha=.8))
for x, z, x0, x1, z0, z1 in FLOWERS:
    ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc='#f2b134', ec='#8a5a00', zorder=6))
    ax.add_patch(Circle((x, z), 1.4, fc='white', ec='#8a5a00', lw=.8, zorder=7))
# Starting game pieces (CAD): nectar staged in each UP cell; pollen in gardens.
for x in (-20.94, -17.32, -13.7):
    ax.add_patch(Circle((x, 9.47), 1.8, fc=RED, ec='white', lw=.5, zorder=7))
for x in (13.81, 17.43, 21.05):
    ax.add_patch(Circle((x, -9.57), 1.8, fc=BLUE, ec='white', lw=.5, zorder=7))
for x in (-69.31, -66.42, -63.53, -60.64):
    ax.add_patch(Circle((x, 69.3), 1.4, fc='#f7d417', ec='#8a7a00', lw=.5, zorder=7))
    ax.add_patch(Circle((-x, -69.3), 1.4, fc='#f7d417', ec='#8a7a00', lw=.5, zorder=7))
# Labels.
lab = dict(fontsize=8.5, ha='center', va='center', zorder=10, color=INK,
           bbox=dict(boxstyle='round,pad=.2', fc='white', ec='#aaa', lw=.6))
ax.text(-52, -35.25, 'red loading zone\nX -70.10..-59.10\nZ -46.60..-23.91', **lab)
ax.text(52, 35.25, 'blue loading zone\nX 59.10..70.10\nZ 23.91..46.60', **lab)
ax.text(-58.8, 61.5, 'red garden  Z 68.10..70.10', **lab)
ax.text(58.8, -61.5, 'blue garden  Z -70.10..-68.10', **lab)
ax.text(-23.44, -59.5, 'flower (-23.44, -68.01)', **lab)
ax.text(-54.5, 23.44, 'flower\n(-68.01, 23.44)', **lab)
ax.text(23.44, 59.5, 'flower (23.44, 68.01)', **lab)
ax.text(54.5, -23.44, 'flower\n(68.01, -23.44)', **lab)
ax.text(-34.5, 0, 'foot bar\nX -24.73..-22.75\n0-2.15 in tall', **lab)
ax.text(35.5, 0, 'foot bar\nX 22.75..24.73', **lab)
ax.text(0, 22.6, 'frame projection X -24.73..24.74, Z +/-19.48', fontsize=7.5, ha='center', color=QUIET, zorder=10)
ax.text(-12.74, -29, 'red hive center X -12.742', fontsize=8, ha='center', color=RED, zorder=10)
ax.text(12.76, 29, 'blue hive center X +12.758', fontsize=8, ha='center', color=BLUE, zorder=10)
# Axes, sides, ends.
ax.add_patch(FancyArrowPatch((0, 0), (14, 0), arrowstyle='-|>', mutation_scale=14, color=INK, lw=1.4, zorder=11))
ax.add_patch(FancyArrowPatch((0, 0), (0, 14), arrowstyle='-|>', mutation_scale=14, color=INK, lw=1.4, zorder=11))
ax.text(15, 0, '+X', fontsize=10, weight='bold', va='center', zorder=11)
ax.text(0, 16.5, '+Z', fontsize=10, weight='bold', ha='center', va='center', zorder=11)
ax.text(0, W + 3.5, 'AUDIENCE (+Z)', fontsize=11, weight='bold', ha='center', va='top')
ax.text(0, -W - 3.5, 'scoring end (-Z)', fontsize=10, ha='center', va='bottom', color=QUIET)
ax.text(-W - 2, 0, 'RED alliance area (outside wall)', rotation=90, fontsize=10, ha='right', va='center', color=RED, weight='bold')
ax.text(W + 2, 0, 'BLUE alliance area (outside wall)', rotation=270, fontsize=10, ha='left', va='center', color=BLUE, weight='bold')
for s in SEAMS:
    ax.text(s, W + 1, f'{s:+.1f}' if s else '0', fontsize=7.5, ha='center', va='top', color=QUIET)
    ax.text(W + 1, s, f'{s:+.1f}' if s else '0', fontsize=7.5, ha='left', va='center', color=QUIET)
ax.text(-W, -W - 3.5, 'inner rail +/-70.585 in', fontsize=8, va='bottom', color=QUIET)
ax.set_xlim(-W - 8, W + 8); ax.set_ylim(-W - 8, W + 8); ax.set_aspect('equal'); ax.axis('off')
ax.invert_yaxis()  # match view: audience (+Z) at the bottom
ax.set_title('BIOBUZZ field layout, inches from field center (STEP v26-27.2, initial hive state)', fontsize=11, pad=8)
fig.text(.5, .015, 'Tile names A-F (red to blue) and 1-6 (audience to scoring) per Field Acceptance 3.5/3.6. '
         'Hive cells swap UP/down when tipped; tags move with them.', ha='center', fontsize=8.5, color=QUIET)
fig.tight_layout(rect=(0, .03, 1, 1))
out = sys.argv[1] if len(sys.argv) > 1 else 'docs/POLLEN_FIELD_LAYOUT.svg'
fig.savefig(out, metadata={'Date': None} if out.endswith(('.svg', '.pdf')) else None)
if out.endswith('.svg'):  # matplotlib leaves trailing spaces in path data; keep git diff --check clean
    with open(out) as f:
        text = '\n'.join(line.rstrip() for line in f.read().splitlines()) + '\n'
    with open(out, 'w') as f:
        f.write(text)
