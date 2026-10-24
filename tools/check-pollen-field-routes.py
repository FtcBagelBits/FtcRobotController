#!/usr/bin/env python3
"""Nominal CAD geometry checks; see docs/POLLEN_FIELD_ROUTES.md. Not physical validation."""
import math
R=9; W=70.585; hx=-12.74164098
start=(-12.74164098, W-R); corner=(start[0]-25,start[1]); primary=(-56,-26); support=(-56,-47)
sx=-10.35; yaw=0
for _ in range(20):
 sz=-W+R*(math.cos(yaw)+math.sin(yaw));yaw=math.atan2(sx-hx,-15.7-sz)
sz=-W+R*(math.cos(yaw)+math.sin(yaw));ss=(sx,sz);sc=(sx-8*math.sin(yaw),sz+8*math.cos(yaw))
def box(x0,x1,z0,z1):return [(x0,z0),(x1,z0),(x1,z1),(x0,z1)]
def robot(p,a=0):
 f=(-math.sin(a),math.cos(a));r=(-math.cos(a),-math.sin(a))
 return [(p[0]+i*R*f[0]+j*R*r[0],p[1]+i*R*f[1]+j*R*r[1]) for i,j in [(1,1),(1,-1),(-1,-1),(-1,1)]]
def cross(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
def hull(ps):
 ps=sorted(set(ps));a=[];b=[]
 for p in ps:
  while len(a)>1 and cross(a[-2],a[-1],p)<=0:a.pop()
  a.append(p)
 for p in reversed(ps):
  while len(b)>1 and cross(b[-2],b[-1],p)<=0:b.pop()
  b.append(p)
 return a[:-1]+b[:-1]
def intersects(a,b):
 for p in [a,b]:
  for v,w in zip(p,p[1:]+p[:1]):
   axis=(v[1]-w[1],w[0]-v[0]);aa=[x*axis[0]+z*axis[1] for x,z in a];bb=[x*axis[0]+z*axis[1] for x,z in b]
   if max(aa)<min(bb)-1e-9 or max(bb)<min(aa)-1e-9:return False
 return True
def point_seg(p,a,b):
 d=(b[0]-a[0],b[1]-a[1]);t=max(0,min(1,((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1])/(d[0]**2+d[1]**2)))
 return math.dist(p,(a[0]+t*d[0],a[1]+t*d[1]))
def distance(a,b):
 if intersects(a,b):return 0
 return min([point_seg(p,v,w) for p in a for v,w in zip(b,b[1:]+b[:1])]+[point_seg(p,v,w) for p in b for v,w in zip(a,a[1:]+a[:1])])
# Conservative all-height projections of fixed field parts; hive projection includes both tips.
obstacles={'frame':box(-24.729,24.742,-19.476,19.476),
'hive both states':box(-23.354,23.354,-25.071,25.071),
'left flower':box(-71.794,-65.630,20.058,26.727),
'right flower':box(65.630,71.794,-26.727,-20.058),
'audience flower':box(20.058,26.727,65.630,71.794),
'scoring flower':box(-26.727,-20.058,-71.794,-65.630)}
foot=box(-24.729,-22.75,-19.472,19.472)
assert intersects(hull(robot(start)+robot(primary)),foot)
assert intersects(robot((hx,-W+R)),obstacles['scoring flower'])
plans={'primary leg 1':hull(robot(start)+robot(corner)), 'primary leg 2':hull(robot(corner)+robot(primary)), 'support leg 1':hull(robot(ss,yaw)+robot(sc,yaw)), 'support leg 2':hull(robot(sc,yaw)+robot(support,yaw))}
for name,p in plans.items():
 for obstacle,b in obstacles.items():
  assert not intersects(p,b),(name,obstacle)
 print(name,'closest',min((distance(p,b),n) for n,b in obstacles.items()))
assert max(p[0] for p in robot(ss,yaw))<0
assert abs(min(p[1] for p in robot(ss,yaw))+W)<1e-9
zone=box(-70.101,-59.585,-46.599,-23.907) # conservative overlap of STEP tape and 11-inch-from-wall check
assert intersects(robot(primary),zone) and intersects(robot(support,yaw),zone)
assert not intersects(robot(primary),robot(support,yaw))
assert all(not intersects(plans[p],plans[s]) for p in ['primary leg 1','primary leg 2'] for s in ['support leg 1','support leg 2'])
print('support start',ss,'yaw',math.degrees(yaw),'flower gap',distance(robot(ss,yaw),obstacles['scoring flower']),'half-field gap',-max(p[0] for p in robot(ss,yaw)))
print('parking gap',distance(robot(primary),robot(support,yaw)))
assert not intersects(robot((-56,-W+R)),zone), 'pure strafe should miss parking'
# True 45-degree line reaches the side boundary before the loading-zone row.
assert (start[1]-(start[0]-(-W+R)))-R > -23.907
print('PASS: nominal swept polygons, both parking overlaps, start legality geometry, and route separation')
