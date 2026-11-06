"""Shared BIOBUZZ field drawing for the pit sheet and the drive-team pictures.
Nominal CAD numbers in inches, field-center origin, red alliance; blue is the same picture rotated
180 degrees. tools/check-pollen-field-routes.py checks these routes; keep the numbers in step with it."""
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


def draw_field(ax, hive_label='HIVE\n(frame foot blocks\na direct diagonal)'):
    """Halves, tile seams, midline, perimeter, hive, flowers, both loading zones and no-wall strips."""
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
    ax.text(0, 0, hive_label, ha='center', va='center', fontsize=8, color='#444', zorder=4)
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
