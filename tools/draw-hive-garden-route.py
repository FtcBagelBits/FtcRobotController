#!/usr/bin/env python3
"""Draw the documented nominal garden route; dimensions from POLLEN_FIELD_LAYOUT.md.
Keep route values synchronized with PollenHiveRoute and its geometry tests.
"""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['svg.fonttype']='none'
matplotlib.rcParams['svg.hashsalt']='pollen-hive-route'
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

root=Path(__file__).resolve().parent.parent
fig,ax=plt.subplots(figsize=(10,5.5))
ax.set_aspect('equal'); ax.set_xlim(-76,4); ax.set_ylim(27,78)
ax.add_patch(Rectangle((-70.585,27),70.585,43.585,facecolor='#f5f8fb',edgecolor='#20344a',linewidth=2))
for x in [-47,-23.5,0]: ax.plot([x,x],[27,70.585],color='#d1d8df',ls='--',lw=.8)
ax.plot([-70.585,0],[47,47],color='#d1d8df',ls='--',lw=.8)
pts=[(-12.74164098,61.585),(-12.74164098,55.585),(-30,55.585),(-55.585,55.585),(-60.585,55.585),(-60.585,60.585)]
ax.plot([p[0] for p in pts],[p[1] for p in pts],'-o',color='#2368a0',lw=2,zorder=5)
for i,(x,z) in enumerate(pts):
 dx,dz=[(1,2),(1,-3),(0,-3),(1,-3),(-6,-3),(-6,2)][i]
 ax.text(x+dx,z+dz,'SABCDE'[i],fontsize=12,weight='bold',color='#153b60')
for x in [-69.31,-66.42,-63.53,-60.64]: ax.add_patch(Circle((x,69.3),1.4,fc='#f2bc39',ec='#936b16',zorder=8))
for i in [2,3]: ax.add_patch(Circle(pts[i],12.728,fill=False,ec='#6b869d',ls=':',lw=1))
ax.add_patch(Rectangle((-69.585,51.585),18,18,fill=False,ec='#29704c',lw=1.5))
ax.annotate('front intake at E',xy=(-61,69.585),xytext=(-50,74),ha='left',fontsize=10,
 arrowprops=dict(arrowstyle='->',color='#29704c'),color='#29704c')
ax.annotate('',xy=(-41,55.585),xytext=(-36,55.585),arrowprops=dict(arrowstyle='->',color='#2368a0',lw=2))
ax.annotate('',xy=(-60.585,59.5),xytext=(-60.585,57),arrowprops=dict(arrowstyle='->',color='#29704c',lw=2))
ax.text(-32,38,'B: turn front toward garden (-90°)\nC: turn front toward wall (0°)\nReturn E → D → C → A → S',ha='center',fontsize=11,color='#20344a')
ax.text(-30,29,'Red primary shown. Blue rotates the entire setup 180°.\nDotted circles show the 18-inch robot’s turning envelope.',ha='center',fontsize=9,color='#4b5b69')
ax.set_title('Garden collection: robot-center waypoints',fontsize=15,loc='left',pad=12)
ax.set_xlabel('Field X (inches)'); ax.set_ylabel('Field Z (inches, toward audience)')
fig.tight_layout()
output=root/'docs/POLLEN_HIVE_GARDEN.svg'
fig.savefig(output,metadata={'Date':None})
output.write_text('\n'.join(line.rstrip() for line in output.read_text().splitlines())+'\n')
