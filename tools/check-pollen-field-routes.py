#!/usr/bin/env python3
"""Nominal CAD geometry checks; see docs/POLLEN_FIELD_ROUTES.md. Not physical validation.
Usage: python3 tools/check-pollen-field-routes.py [--presets presets.json]
With --presets (written by tools/pollen-tests/PollenPresetExport.java; tools/test-pollen-sequence.sh
does this), the routes checked are rebuilt from the Java presets and must match this file's plan."""
import json, math, sys
R=9; W=70.585; hx=-12.74164098
# Primary (B Square): forward 10 cm off the wall, left 25 in, then diagonal to park.
start=(-12.74164098, W-R); off_wall=(start[0],start[1]-10/2.54); corner=(off_wall[0]-25,off_wall[1]); primary=(-56,-26); support=(-56,-47)
# Support (C): flush and square, centered between the flower's inboard edge and the midline.
flower_x=-20.058; midline=0
ss=((flower_x+midline)/2,-W+R); sc=(ss[0],ss[1]+8); yaw=0
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
'scoring flower':box(-26.727,flower_x,-71.794,-65.630)}
foot=box(-24.729,-22.75,-19.472,19.472)
assert intersects(hull(robot(start)+robot(primary)),foot), 'direct primary diagonal should hit the frame foot'
assert intersects(robot((hx,-W+R)),obstacles['scoring flower']), 'centered support start should overlap the flower'
routes={'primary':[start,off_wall,corner,primary],'support':[ss,sc,support]}

def java_route(preset, legs):
    """Waypoints for a preset's start and the given legs (cm, +left deg); 0-cm legs are skipped."""
    fwd,left=((0,-1),(-1,0)) if preset['facing']=='-Z' else ((0,1),(1,0))
    pts=[tuple(preset['start'])]
    for cm,deg in legs:
        if cm<=0: continue
        d=cm/2.54; a=math.radians(deg); x,z=pts[-1]
        pts.append((x+d*(math.cos(a)*fwd[0]+math.sin(a)*left[0]),z+d*(math.cos(a)*fwd[1]+math.sin(a)*left[1])))
    return pts

if '--presets' in sys.argv:
    with open(sys.argv[sys.argv.index('--presets')+1]) as f: java=json.load(f)
    for k,v in (('wall',W),('half',R),('red_hive_x',hx),('flower_inboard_x',flower_x)):
        assert abs(java[k]-v)<1e-9,('Java constant differs from checker',k,java[k],v)
    for name,key in (('primary','b_square'),('support','c')):
        p=java[key]; tuned=set(p.get('field_tuned_legs',[]))
        got=java_route(p,p['plan_legs']); want=routes[name]
        assert len(got)==len(want),(name,'Java plan has a different number of legs',got)
        for g,w in zip(got,want):
            assert math.dist(g,w)<0.01,(name,'Java plan waypoint',g,'differs from checker plan',w)
        assert math.dist(got[-1],tuple(p['park']))<0.01,(name,'Java plan does not end at its park point')
        for i,(leg,planned) in enumerate(zip(p['legs'],p['plan_legs']),1):
            if i in tuned:
                end=java_route(p,p['legs'][:i])[-1]
                print(f'NOTE {name} leg {i} is field-tuned: preset {leg[0]:.1f} cm @ {leg[1]:+.2f} deg vs plan '
                      f'{planned[0]:.1f} cm @ {planned[1]:+.2f} deg; the drive model would end it at '
                      f'({end[0]:.1f}, {end[1]:.1f}) instead of {tuple(p["park"])}. Verify the real endpoint on the field.')
            else:
                assert abs(leg[0]-planned[0])<1e-6 and abs(leg[1]-planned[1])<1e-6,(name,'preset leg',i,leg,'differs from plan',planned)
    print('Java presets match the checked plan (start and untuned legs; field-tuned legs reported above)')
else:
    print('NOTE: Java presets not compared; tools/test-pollen-sequence.sh runs this with --presets')
plans={f'{name} leg {i+1}':hull(robot(a)+robot(b)) for name,pts in routes.items() for i,(a,b) in enumerate(zip(pts,pts[1:]))}
for name,p in plans.items():
 for obstacle,b in obstacles.items():
  assert not intersects(p,b),(name,obstacle)
 print(name,'closest',min((distance(p,b),n) for n,b in obstacles.items()))
flower_gap=distance(robot(ss),obstacles['scoring flower']); half_gap=-max(p[0] for p in robot(ss))
assert half_gap>0 and abs(flower_gap-half_gap)<1e-9, 'support start must split the gap evenly'
assert abs(min(p[1] for p in robot(ss))+W)<1e-9, 'support rear edge flush on the wall'
assert abs(max(p[1] for p in robot(start))-W)<1e-9, 'primary rear edge flush on the wall'
zone=box(-70.101,-59.585,-46.599,-23.907) # conservative overlap of STEP tape and 11-inch-from-wall check
assert intersects(robot(primary),zone) and intersects(robot(support),zone)
# LEAVE is judged at the end of AUTO: a parked robot touching the side wall loses it.
wall_gaps={n:min(x for x,z in robot(p))+W for n,p in (('primary',primary),('support',support))}
assert all(g>0 for g in wall_gaps.values()), wall_gaps
print('parked side-wall gaps (in)',wall_gaps)
assert not intersects(robot(primary),robot(support))
assert all(not intersects(plans[p],plans[s]) for p in plans if p.startswith('primary') for s in plans if s.startswith('support'))
print('support start',ss,'flower gap',flower_gap,'half-field gap',half_gap,'offset from hive center',ss[0]-hx)
print('parking gap',distance(robot(primary),robot(support)))
assert not intersects(robot((-56,-W+R)),zone), 'pure strafe should miss parking'
# True 45-degree line reaches the side boundary before the loading-zone row.
assert (start[1]-(start[0]-(-W+R)))-R > -23.907
print('PASS: nominal swept polygons, both parking overlaps, start legality geometry, and route separation')
