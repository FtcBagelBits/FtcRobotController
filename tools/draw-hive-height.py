#!/usr/bin/env python3
"""Side view for the hive calibration sheet: camera, our cell's tags UP and DOWN, and the bands.
Numbers: docs/POLLEN_FIELD_LAYOUT.md (CAD) and PollenConfig (14 in lens, 49.7 / 35.6 in tags, 3 in band).
Usage: python3 tools/draw-hive-height.py  ->  docs/drive-team/img/hive-height.svg
"""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype'] = 'none'
matplotlib.rcParams['svg.hashsalt'] = 'pollen-hive-height'
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

UP, DOWN, BAND, LENS = 49.7, 35.6, 3.0, 14.0
WALL, ROBOT_CENTER = 70.585, 61.585            # end wall and robot center, inches from hive center
CAM_Z = ROBOT_CENTER - 9                        # camera on the back of the 18 in robot
TAG_UP_Z, TAG_DOWN_Z = 12.9, 11.4               # middle of our cell's tag row, UP and DOWN (CAD)

root = Path(__file__).resolve().parent.parent
fig, ax = plt.subplots(figsize=(7.4, 4.0))
ax.set_xlim(-8, 74); ax.set_ylim(0, 60); ax.set_aspect('equal')
ax.axis('off')
ax.plot([-8, WALL], [0, 0], color='#333', lw=2)                       # tiles
ax.add_patch(Rectangle((WALL, 0), 1.2, 14, color='#555'))              # end wall
ax.text(WALL + .6, 15.5, 'end\nwall', ha='center', fontsize=8)
# Bands
ax.add_patch(Rectangle((-8, UP - BAND), 70, 2 * BAND, color='#d8efd9', zorder=0))
ax.add_patch(Rectangle((-8, DOWN - BAND), 70, 2 * BAND, color='#fbe3c8', zorder=0))
ax.add_patch(Rectangle((-8, DOWN + BAND), 70, UP - DOWN - 2 * BAND, color='#eeeeee', zorder=0))
ax.text(61, UP - .6, 'UP band  %.1f ± %.0f in: may shoot' % (UP, BAND), fontsize=8, color='#1d5e27', ha='right')
ax.text(61, (UP + DOWN) / 2 - .6, 'between the bands: UNKNOWN, never shoots', fontsize=8, color='#555', ha='right')
ax.text(61, DOWN - .6, 'DOWN band  %.1f ± %.0f in' % (DOWN, BAND), fontsize=8, color='#8a4b0c', ha='right')
# Hive pivot and our cell's tag row in both states
ax.plot([0], [43.95], 'o', color='#20344a'); ax.text(-1, 43.95 + .8, 'hive\npivot', fontsize=7.5, ha='right')
ax.plot([0, TAG_UP_Z], [43.95, UP], color='#20344a', lw=1)
ax.plot([0, TAG_DOWN_Z], [43.95, DOWN], color='#20344a', lw=1, ls=':')
ax.add_patch(Rectangle((TAG_UP_Z - 2, UP - .6), 4, 1.2, color='#111', zorder=4))
ax.add_patch(Rectangle((TAG_DOWN_Z - 2, DOWN - .6), 4, 1.2, color='#999', zorder=4))
ax.text(TAG_UP_Z - 2.5, UP + 1.6, 'our cell UP', fontsize=8, zorder=5, ha='center')
ax.text(TAG_DOWN_Z - 2.5, DOWN - 2.6, 'our cell DOWN', fontsize=8, zorder=5, ha='center')
# Robot, camera and lines of sight
ax.add_patch(Rectangle((ROBOT_CENTER - 9, 1), 18, 11, fc='#cfe0f1', ec='#2368a0', lw=1.5))
ax.text(ROBOT_CENTER, 4.5, 'robot\nfront on wall', ha='center', fontsize=8)
ax.add_patch(Polygon([[CAM_Z, LENS], [CAM_Z - 3, LENS + 2.6], [CAM_Z - 3.6, LENS + .4]], color='#2368a0'))
ax.plot([CAM_Z, TAG_UP_Z], [LENS, UP], color='#2368a0', lw=1, ls='--')
ax.plot([CAM_Z, TAG_DOWN_Z], [LENS, DOWN], color='#c06b1f', lw=1, ls='--')
ax.plot([CAM_Z, CAM_Z - 14], [LENS, LENS], color='#777', lw=.8)
ax.text(CAM_Z - 14.5, LENS - .5, 'level', fontsize=7, color='#777', ha='right')
ax.text(34, 5, 'camera on the back, tilted up\n(Square measures the tilt)', fontsize=8, color='#153b60', ha='center')
ax.plot([CAM_Z + .3, CAM_Z + .3], [12, LENS], color='#2368a0', lw=2)
ax.annotate('', xy=(CAM_Z - 2, 0), xytext=(CAM_Z - 2, LENS), arrowprops=dict(arrowstyle='<->', color='#153b60', lw=.8))
ax.text(CAM_Z - 2.6, 5.5, 'lens\n%.0f in' % LENS, fontsize=7.5, ha='right', color='#153b60')
ax.text(36, 57, 'height = lens + z·sin(tilt) − y·cos(tilt)', fontsize=9.5, ha='center', weight='bold', color='#153b60')
fig.tight_layout(pad=.2)
out = root / 'docs' / 'drive-team' / 'img' / 'hive-height.svg'
fig.savefig(out)
print('wrote', out)
