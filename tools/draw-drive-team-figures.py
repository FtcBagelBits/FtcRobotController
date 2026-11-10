#!/usr/bin/env python3
"""Draw the pictures in docs/drive-team/img/ (needs matplotlib).
Usage: python3 tools/draw-drive-team-figures.py
Field numbers come from tools/pollen_field_drawing.py (the same ones as the pit sheet);
tools/check-pollen-field-routes.py checks those routes against the field CAD."""
import os
import sys
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'
matplotlib.rcParams['svg.hashsalt'] = 'pollen-drive-team'
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Circle, Rectangle

sys.dont_write_bytecode = True  # no __pycache__ in tools/
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pollen_field_drawing import *  # noqa: F401,F403  R, W, hx, P, S, FLOWERS, colours, rot, rect, robot, draw_field

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'drive-team', 'img')
LEG_NAMES = {'P': ['1  off the wall', '2  left past the hive', '3  to the slot'],
             'S': ['1  off the wall', '2  right to the slot']}


def save(fig, name):
    path = os.path.join(OUT, name)
    fig.savefig(path, metadata={'Date': None})
    with open(path) as f:
        text = '\n'.join(line.rstrip() for line in f.read().splitlines()) + '\n'
    with open(path, 'w') as f:
        f.write(text)
    plt.close(fig)


def route_picture(tag, name, title):
    """Whole field, both alliances, only this mode's lane."""
    pts, color = (P, P_COL) if tag == 'P' else (S, S_COL)
    fig = plt.figure(figsize=(6.4, 6.6))
    ax = fig.add_axes([.01, .01, .98, .98])
    draw_field(ax, hive_label='HIVE')
    for blue in (False, True):
        xs, zs = zip(*[rot(p, blue) for p in pts])
        ax.plot(xs, zs, color=color, lw=3, marker='o', ms=4, zorder=7)
        robot(ax, pts[0], color, .4, blue)
        robot(ax, pts[-1], color, .8, blue)
        for p, word in ((pts[0], 'START'), (pts[-1], 'PARK')):
            px, pz = rot(p, blue)
            ax.text(px, pz, word, ha='center', va='center', fontsize=13, weight='bold', color='white', zorder=9)
    # Number the legs on the red side only (blue is the same, turned around).
    for i, label in enumerate(LEG_NAMES[tag]):
        (x0, z0), (x1, z1) = pts[i], pts[i + 1]
        mx, mz = (x0 + x1) / 2, (z0 + z1) / 2
        dx = {('P', 0): 11, ('P', 1): 0, ('P', 2): 0, ('S', 0): 10.5, ('S', 1): 0.001}[(tag, i)]
        dz = {('P', 0): 0, ('P', 1): -5, ('P', 2): 0, ('S', 0): 0, ('S', 1): 7}[(tag, i)]
        if (tag, i) == ('P', 2):  # long diagonal: label in the open space left of it
            mx, dx = -W + 1.5, 0.001
        if (tag, i) == ('S', 1):  # start the label clear of the PARK box
            mx = -44
        ax.text(mx + dx, mz + dz, label, fontsize=14, weight='bold', color='#111',
                ha='left' if dx > 0 else ('right' if dx < 0 else 'center'),
                va='center', zorder=11, bbox=dict(boxstyle='round,pad=.2', fc='white', ec=color, lw=1))
    ax.text(-W + 2, -W - 2.5, 'RED', color=RED, fontsize=18, weight='bold', va='bottom')
    ax.text(W - 2, -W - 2.5, 'BLUE', color=BLUE, fontsize=18, weight='bold', va='bottom', ha='right')
    ax.text(0, W + 2.5, 'AUDIENCE', fontsize=14, weight='bold', ha='center', va='top')
    ax.text(0, -W - 2.5, 'SCORING END', fontsize=14, weight='bold', ha='center', va='bottom')
    ax.set_xlim(-W - 3, W + 3); ax.set_ylim(-W - 9, W + 9); ax.set_aspect('equal'); ax.axis('off')
    ax.invert_yaxis()  # team view: red left, audience at the bottom
    save(fig, name)


def gap(ax, x0, x1, z, text, label_xy, align):
    """A small red gap marker between x0 and x1 at depth z, with its label off to the side."""
    ax.annotate('', xy=(x0, z), xytext=(x1, z), arrowprops=dict(arrowstyle='<->', lw=1.6, color='#c0392b'), zorder=9)
    ax.annotate(text, xy=((x0 + x1) / 2, z), xytext=label_xy, ha=align, va='center', fontsize=15, weight='bold',
                color='#c0392b', zorder=10, bbox=dict(boxstyle='round,pad=.25', fc='white', ec='#c0392b', lw=1),
                arrowprops=dict(arrowstyle='-', color='#c0392b', lw=1))


def placement_picture(tag, name, title):
    """Close-up of the red start; blue is the same at the other end of the field.
    'away' is the direction from the wall toward the hive (+1 or -1 in field z)."""
    fig = plt.figure(figsize=(6.4, 5.4))
    ax = fig.add_axes([.01, .01, .98, .98])
    if tag == 'P':
        (x, z), wall, away = P[0], W, -1
    else:
        (x, z), wall, away = S[0], -W, 1
    xmin, xmax = -36, 12
    near, far = wall - away * 5, wall + away * 40          # view: a little behind the wall to 40 in out
    zlo, zhi = min(near, far), max(near, far)
    ax.add_patch(Rectangle((xmin, zlo), -xmin, zhi - zlo, color='#fdf1f0', zorder=0))
    ax.add_patch(Rectangle((0, zlo), xmax, zhi - zlo, color='#eef3fb', zorder=0))
    ax.plot([0, 0], [zlo, zhi], color='#555', lw=1.6, ls='--', zorder=2)
    ax.text(.8, wall + away * 36, 'center seam\n(alliance midline)', fontsize=13.6, color='#333', va='center')
    ax.plot([xmin, xmax], [wall, wall], color='#222', lw=6, zorder=8)
    ax.text(xmin + 1, wall - away * 2.6, 'FIELD WALL (the robot faces it)', fontsize=14.4, weight='bold', va='center')
    for f in FLOWERS:
        if f[1] < xmin or f[0] > xmax or max(f[2], f[3]) < zlo or min(f[2], f[3]) > zhi:
            continue
        rect(ax, *f, fc='#f2b134', ec='#8a5a00', zorder=4)
        ax.text((f[0] + f[1]) / 2, wall + away * 9.5, 'flower', fontsize=13.6, ha='center', va='center', weight='bold', color='#8a5a00')
    ax.plot([hx, hx], [zlo, zhi], color='#888', lw=1, ls=':', zorder=2)
    ax.text(hx, wall + away * 38, 'hive center', fontsize=13.6, color='#555', ha='center', va='center',
            bbox=dict(fc='white', ec='none', pad=1))
    col = P_COL if tag == 'P' else S_COL
    ax.add_patch(Rectangle((x - R, z - R), 2 * R, 2 * R, fc=col, ec='#111', lw=1.5, alpha=.9, zorder=6))
    ax.text(x, z, 'ROBOT\nFRONT flat\non the wall', ha='center', va='center', fontsize=14.4, weight='bold', color='white', zorder=7)
    back = z + away * R  # the robot's back (launcher) is the side toward the hive
    ax.annotate('', xy=(x, back + away * 9), xytext=(x, back + away * .5),
                arrowprops=dict(arrowstyle='-|>', lw=2.5, color='#111', mutation_scale=18), zorder=7)
    ax.text(x + 1.2, back + away * 5, 'BACK\n(launcher)\nto the hive', ha='left', va='center', fontsize=14.4, weight='bold')
    if tag == 'P':
        gap(ax, x + R, 0, z, '3.7 in\n(9.5 cm)', (2.0, wall + away * 20), 'left')
        ax.text(xmin + 1, wall + away * 30, 'Center the robot on the hive.', fontsize=15.2, weight='bold')
    else:
        gap(ax, -20.058, x - R, wall + away * 4, 'about 1 in', (xmin + 1, wall + away * 16), 'left')
        gap(ax, x + R, 0, wall + away * 4, 'about 1 in', (2.0, wall + away * 16), 'left')
        ax.text(xmin + 1, wall + away * 32, 'Equal gaps, about 1 in (2.6 cm).\nIt is tight: use a spacer gauge,\nthen take it away.', fontsize=14.4, weight='bold', va='center')
    ax.set_xlim(xmin, xmax); ax.set_ylim(zlo, zhi); ax.set_aspect('equal'); ax.axis('off')
    ax.invert_yaxis()
    save(fig, name)


def controller_picture(name, title, labels):
    """A plain, generic game controller with labels; labels maps a control to text ('' hides it)."""
    fig = plt.figure(figsize=(9, 5.2))
    ax = fig.add_axes([0, 0, 1, .9])
    ax.add_patch(FancyBboxPatch((-4.2, -1.6), 8.4, 3.4, boxstyle='round,pad=0.4,rounding_size=1.4',
                                fc='#e8eaee', ec='#333', lw=2))
    ax.add_patch(FancyBboxPatch((-3.9, 2.05), 1.7, .45, boxstyle='round,pad=0.05,rounding_size=.2', fc='#c9ccd3', ec='#333'))
    ax.add_patch(FancyBboxPatch((2.2, 2.05), 1.7, .45, boxstyle='round,pad=0.05,rounding_size=.2', fc='#c9ccd3', ec='#333'))
    ax.text(-3.05, 2.27, 'L1', ha='center', va='center', fontsize=10, weight='bold')
    ax.text(3.05, 2.27, 'R1', ha='center', va='center', fontsize=10, weight='bold')
    for tx in (-3.05, 3.05):  # triggers sit above (behind) the bumpers
        ax.add_patch(FancyBboxPatch((tx - .55, 2.75), 1.1, .5, boxstyle='round,pad=0.05,rounding_size=.25', fc='#b5b9c2', ec='#333'))
    ax.text(-3.05, 3.0, 'left trigger', ha='center', va='center', fontsize=8)
    ax.text(3.05, 3.0, 'right trigger', ha='center', va='center', fontsize=8)
    for cx, cy in ((-1.6, -.6), (1.6, -.6)):  # sticks
        ax.add_patch(Circle((cx, cy), .62, fc='#555', ec='#222'))
        ax.add_patch(Circle((cx, cy), .38, fc='#777', ec='#222'))
    dx, dy = -2.95, .8  # d-pad
    for ox, oy, a in ((0, .45, 'UP'), (0, -.45, 'DN'), (-.45, 0, 'L'), (.45, 0, 'R')):
        ax.add_patch(Rectangle((dx + ox - .2, dy + oy - .2), .4, .4, fc='#555', ec='#222'))
    ax.add_patch(Rectangle((dx - .2, dy - .2), .4, .4, fc='#555', ec='#222'))
    bx, by = 2.95, .8  # face buttons
    for ox, oy, sym in ((0, .5, 'triangle'), (-.5, 0, 'square'), (.5, 0, 'circle'), (0, -.5, 'cross')):
        ax.add_patch(Circle((bx + ox, by + oy), .24, fc='#333', ec='#111'))
        sx, sy = bx + ox, by + oy
        if sym == 'triangle':
            ax.plot([sx - .11, sx + .11, sx, sx - .11], [sy - .08, sy - .08, sy + .11, sy - .08], color='#7fd1c4', lw=1.6)
        elif sym == 'square':
            ax.add_patch(Rectangle((sx - .09, sy - .09), .18, .18, fill=False, ec='#e6a3d3', lw=1.6))
        elif sym == 'circle':
            ax.add_patch(Circle((sx, sy), .1, fill=False, ec='#ef7a7a', lw=1.6))
        else:
            ax.plot([sx - .09, sx + .09], [sy - .09, sy + .09], color='#8fb3f0', lw=1.6)
            ax.plot([sx - .09, sx + .09], [sy + .09, sy - .09], color='#8fb3f0', lw=1.6)
    anchors = {  # control: (point on controller, label position, alignment)
        'L1': ((-3.9, 2.27), (-6.2, 1.55), 'left'), 'R1': ((3.9, 2.27), (6.2, 2.0), 'right'),
        'LT': ((-3.6, 3.15), (-6.2, 3.55), 'left'), 'RT': ((3.6, 3.15), (6.2, 3.55), 'right'),
        'DPAD': ((dx - .7, dy), (-6.2, .55), 'left'), 'LSTICK': ((-1.6, -1.2), (-6.2, -2.4), 'left'),
        'RSTICK': ((1.6, -1.2), (6.2, -2.4), 'right'), 'TRIANGLE': ((bx + .24, by + .5), (6.2, 1.5), 'right'),
        'CIRCLE': ((bx + .74, by), (6.2, -.35), 'right'), 'SQUARE': ((bx - .5, by - .24), (6.2, -1.2), 'right', (bx - .5, -1.2)),
        'CROSS': ((bx, by - .74), (6.2, -1.05), 'right'),
    }
    for key, text in labels.items():
        if not text:
            continue
        (px, py), (lx, ly), align, *via = anchors[key]
        if via:  # bend the leader so it does not cross other labels: button -> via, then via -> label
            ax.plot([px, via[0][0]], [py, via[0][1]], color='#666', lw=.9, zorder=3)
            px, py = via[0]
        ax.annotate(text, xy=(px, py), xytext=(lx, ly), ha=align, va='center', fontsize=10.5,
                    bbox=dict(boxstyle='round,pad=.3', fc='white', ec='#666', lw=.8),
                    arrowprops=dict(arrowstyle='-', color='#666', lw=.9, shrinkA=0, shrinkB=2))
    ax.set_xlim(-6.4, 6.4); ax.set_ylim(-3.2, 4.1); ax.set_aspect('equal'); ax.axis('off')
    fig.suptitle(title, fontsize=13, weight='bold', y=.97)
    save(fig, name)


def init_buttons_picture(name, title, circle_text):
    """Just the buttons that matter during autonomous INIT, drawn large enough to read at half a page."""
    fig = plt.figure(figsize=(5.0, 2.15))
    ax = fig.add_axes([0, 0, 1, .86])
    bx, by, gap = 0.0, 0.0, .62
    for ox, oy, sym, text, align in ((0, gap, 'triangle', 'Triangle: defaults', 'left'),
                                     (gap, 0, 'circle', circle_text, 'left'),
                                     (-gap, 0, 'square', 'Square: does nothing', 'right'),
                                     (0, -gap, 'cross', '', 'left')):
        sx, sy = bx + ox, by + oy
        ax.add_patch(Circle((sx, sy), .27, fc='#333', ec='#111'))
        if sym == 'triangle':
            ax.plot([sx - .13, sx + .13, sx, sx - .13], [sy - .09, sy - .09, sy + .13, sy - .09], color='#7fd1c4', lw=2)
        elif sym == 'square':
            ax.add_patch(Rectangle((sx - .1, sy - .1), .2, .2, fill=False, ec='#e6a3d3', lw=2))
        elif sym == 'circle':
            ax.add_patch(Circle((sx, sy), .12, fill=False, ec='#ef7a7a', lw=2))
        else:
            ax.plot([sx - .1, sx + .1], [sy - .1, sy + .1], color='#8fb3f0', lw=2)
            ax.plot([sx - .1, sx + .1], [sy + .1, sy - .1], color='#8fb3f0', lw=2)
        if text:
            tx = sx + (.42 if align == 'left' else -.42)
            ax.text(tx, sy, text, ha=align, va='center', fontsize=10.5, weight='bold')
    ax.text(-.2, -1.2, 'D-pad: pick a row (UP/DOWN) and change it (LEFT/RIGHT),\nonly if the coach says.',
            ha='center', va='center', fontsize=9)
    ax.set_xlim(-3.2, 3.6); ax.set_ylim(-1.5, .95); ax.set_aspect('equal'); ax.axis('off')
    fig.suptitle(title, fontsize=11, weight='bold', y=.97)
    save(fig, name)


if __name__ == '__main__':
    os.makedirs(OUT, exist_ok=True)
    route_picture('P', 'shoot-and-park-route.svg', 'Odo ROBOT Shoot + Park: shoot, then park in the INNER slot')
    placement_picture('P', 'shoot-and-park-placement.svg', 'Where to put the robot (red: audience wall)')
    route_picture('S', 'supporting-shooter-route.svg', 'Odo Supporting Shooter: wait, shoot, park in the OUTER slot')
    placement_picture('S', 'supporting-shooter-placement.svg', 'Where to put the robot (red: scoring-end wall)')
    controller_picture('teleop-controller.svg', 'PollenBot TeleOp: during the match', {
        'LSTICK': 'Left stick: drive and strafe', 'RSTICK': 'Right stick: turn',
        'R1': 'R1 (hold): shoot', 'L1': 'L1 (hold): launcher backwards\nto clear a jam',
        'RT': 'Right trigger: intake in', 'LT': 'Left trigger: intake out',
        'DPAD': 'D-pad: INIT menu only\n(UP/DOWN pick a row,\nLEFT/RIGHT change it)', 'TRIANGLE': 'Triangle: INIT menu defaults',
    })
    init_buttons_picture('shoot-and-park-buttons.svg', 'Buttons during INIT (before START)',
                         'Circle: does nothing')
    init_buttons_picture('supporting-shooter-buttons.svg', 'Buttons during INIT (before START)',
                         'Circle: does nothing\nin this mode')
    print('wrote', ', '.join(sorted(os.listdir(OUT))))
