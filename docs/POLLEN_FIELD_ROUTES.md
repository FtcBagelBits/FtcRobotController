# Pollen field routes: 18 x 18-inch robot

**These are nominal geometric plans and untested timed-drive presets.** The
code does not localize, measure clearance, detect a hive tip, or confirm parking.
A short test move and a full-field route are different things.

## Sources and coordinate convention

Downloaded and inspected October 23, 2026 Pacific:

- [Official field STEP v26-27.2](https://ftc-resources.firstinspires.org/ftc/field/field-cad-step),
  linked from [FIRST field resources](https://ftc-resources.firstinspires.org/ftc/field).
  The ZIP contains `field-cad-step.step`. Assembly transforms and BREP vertex
  coordinates were used for the dimensions below. Rounded conservative bounds
  are used for collision checks; these are not measurements of the practice field.
- [Competition Manual, Arena TU06](https://ftc-resources.firstinspires.org/ftc/game/manual-09),
  figures 9-2, 9-8 and 9-9: field layout, frame and hive geometry; section 9.1
  explains nominal dimensions and the general +/-1-inch tolerance.
- [Event Field Setup Guide V1.0](https://ftc-resources.firstinspires.org/ftc/field/eventfieldguide),
  pages 14 and 23: loading-zone tape and initial hive orientation.
- [Field Acceptance Checklist](https://ftc-resources.firstinspires.org/ftc/field/field-check),
  downloaded revision 26-27.2: frame placement and loading-zone tape checks.
- [Game Details TU05, sections 10.5 and 10.5.4, Tables 10-2 and 10-3](https://ftc-resources.firstinspires.org/ftc/game/manual-10):
  PARK requires partial presence in the alliance loading zone; LEAVE requires
  no longer contacting the perimeter wall. Both are assessed at the end of AUTO.
  LEAVE is 3 points (AUTO only); PARK is 5 in AUTO and 5 at the end of the match.
  The SWARM RP needs combined LEAVE + PARK points at or above the threshold:
  16 at "all other events"; championships TBA; premier events may set their own.
- [Game Rules TU06, G302-G304](https://ftc-resources.firstinspires.org/ftc/game/manual-11):
  a starting robot contacts the perimeter wall, is fully on its alliance's side,
  does not contact a flower or its scoring volume, is not in the loading zone
  and contacts exactly 4 pre-loaded pollen (G304.A-G). Alignment devices may be
  brought for setup (G302) but nothing but the robot may remain on the field (G303.C).

Coordinates below are **inches, centered on the field**, using the STEP model's
horizontal X and Z axes. +X points toward the blue alliance; +Z points toward
the audience. Red is X<0. The STEP vertical Y coordinate is not used for the
2D footprint check. The robot is modeled as a filled 18-inch square; no benefit
is assumed from cutout corners or low chassis height.

| Item | CAD / planning bounds in inches |
| --- | --- |
| End-wall inner top-rail face | Z = +/-70.585 |
| Red hive horizontal center | X = -12.742 |
| Blue hive horizontal center | X = +12.758 |
| Frame projection | X [-24.729, +24.742], Z [-19.476, +19.476] |
| Red-side floor-level frame bar | X [-24.729, -22.750], Z [-19.472, +19.472] |
| Hive projection, conservative union of both tipped states | X [-23.354, +23.354], Z [-25.071, +25.071] |
| Red loading-zone tape bounds in STEP | X [-70.101, -59.101], Z [-46.599, -23.907] |
| Flower on scoring end wall | X [-26.727, -20.058], Z [-71.794, -65.630] |
| Flower on red side wall | X [-71.794, -65.630], Z [+20.058, +26.727] |

The manual describes an approximately 144-inch field. This STEP model's inner
rail span is about 141.17 inches, and its side-glass plane is 0.089 inch farther
out than the top rail. The tape starts 0.484 inch inboard of the inner rail in
this export, while the current acceptance check measures 11 inches from the
inside wall. These differences are **not silently averaged**. The checks use
the inner rail for wall contact and conservatively use X<=-59.585 for parking
(the intersection of the CAD tape and the 11-inch-from-wall criterion).
Verify the actual bumper contact height, wall span, tape and flower position.

## Primary shooter: the direct diagonal hits the frame

Primary red starts at **(-12.742, +61.585)**, with its rear edge on the audience
end wall, facing the initially upward-facing red cell. For a nominal near
parking position **(-56, -26)**, the direct path is:

- 222.47 cm robot-forward and 109.88 cm robot-left;
- 248.12 cm total at +26.285 degrees left of forward.

Expanding the floor-level frame bar by the robot's 9-inch half-width/half-length
shows the direct path intersects it after about **93.8 cm of center travel**.
This is an actual floor-level obstruction, not merely the projected outline of
an overhead cell. The existing 60.96-cm test stops before it. Extending the old
+25.87-degree route to full-field distance is not a safe parking route.

A true 45-degree physical diagonal clears the frame but reaches the side wall
well before the loading-zone row. It therefore does not solve the parking task.
The reported 45-degree *motor command* also traveled at only 29.86 degrees;
do not confuse commanded angle with measured path direction.

**Primary field preset (Square in B):**

1. Shoot from the starting position (flush and square to the audience wall).
2. Leg 1: drive **10 cm forward** to (-12.742, +57.648) so the left strafe is
   not dragging along the wall.
3. Leg 2: strafe left **63.50 cm (25 inches)** to X=-37.742, holding the same heading.
4. Leg 3: travel **217.47 cm at +12.313 degrees** to (-56,-26).
5. Stop. The nominal footprint overlaps the loading zone without pressing the
   side wall, leaving room for the supporting robot's separate parking slot.

The preset uses MAT_TRIAL and power 0.25. Its calculated drive time is about
9.70 seconds; with the default 10-second feed, four-second spinup allowance and
0.5-second hold, the total is about **24.20 seconds**. Short loop transitions
consume part of the reserved margin. Triangle restores B's tested short baseline.

Swept-square checks clear the fixed frame, both hive projections and all four
flowers. The smallest nominal obstacle clearance along this three-leg route is
about **8.52 inches (21.6 cm)** to the red side-wall flower, slightly more than
the earlier two-leg version (8.18 inches) because leg 3 starts closer to the
park slot. This does not account for loose game pieces or another robot's route.

## Supporting shooter: flush, square, centered in the flower gap

The initially downward-facing red cell is approached from the scoring end wall.
A square, directly centered start at **(-12.742, -61.585)** overlaps the scoring
wall's flower by about **1.68 inches** in the horizontal projection. A purely
rightward strafe also misses the loading zone's row.

The robot must fit between the flower's inboard edge (**X=-20.058**) and the
alliance midline (**X=0**): 20.058 inches for an 18-inch robot, leaving only
**2.058 inches (5.2 cm)** of slack in total. The preset splits it evenly:

- Robot center: **X=-10.029, Z=-61.585 inches**; rear edge flush on the end
  wall, chassis square to it. No aiming yaw.
- Gap to flower and gap to midline: **1.029 inch (2.61 cm) each**.
- Robot centerline **2.713 inches (6.89 cm) toward the midline** from the hive
  center. The shot goes straight forward from the same distance as a centered
  start (center 45.9 inches from the estimated opening at Z=-15.7), so it lands
  about 6.9 cm off the cell center; the opening is about 20 inches (51 cm) wide.
  This assumes a launcher on the robot centerline; confirm the real shot.

The -20.058 edge is the flower's mounting bracket on top of the wall rail, about
11.4-11.8 inches above the floor. The flower body plates (about 4-5 and 20-21
inches up) end near X=-20.45. If the robot's rear-right corner is lower than
the bracket, the real slack is about 1 cm larger. These are CAD vertex bounds:
measure the actual flower-to-midline gap before relying on either number, and
never start partly across the alliance boundary.

**Supporting route, after its stationary shot:**

1. Leg 1: drive **20.32 cm (8 inches) forward** to clear the flower. The rear
   edge then sits about 3 inches past the flower's field-side face.
2. Leg 2: unused (0 cm) by default; available as a controller row.
3. Leg 3: travel **117.96 cm at -81.848 degrees**, i.e. **16.73 cm forward /
   116.77 cm right**, to nominal **(-56,-47)**. The footprint partially overlaps
   the loading zone with **3.0 inches (7.6 cm)** of separation from the primary
   robot's nominal parked footprint.

Leg 1's nominal flower clearance equals the 2.61-cm start gap; leg 3 clears the
flower by at least **3.16 inches (8.0 cm)**. The two prescribed routes' swept
footprints do not overlap. A different partner's actual route still needs
coordination. These coordinates describe two parking slots, not an official
team-specific assigned spot.

Supporting defaults use MAT_TRIAL, power **0.45**, 15 seconds before feeding,
and 7.5 seconds of feeding. The higher power is required by the short remaining
time budget and has not been validated for this sideways move. Calculated total
is about **25.85 seconds**, including the hold, below the existing 27-second
cutoff and the editor's stricter <26-second planning limit.

The flywheel alone spins from approximately t=11 to t=15. Feeding starts no
earlier than t=15 after speed qualification, and runs for 7.5 seconds. Drive
starts at approximately t=22.5, with all shooter outputs off. If spinup fails,
the robot waits through the feed slot before departing. It does **not** verify
that the partner tipped the hive; the wait is an open-loop coordination plan.

## Parking margins and the auto SWARM points

Two robots that each LEAVE (3) and AUTO PARK (5) score exactly 16, the default
SWARM threshold, before TeleOp. Both are judged at the end of AUTO, so a robot
that parks against the side wall still earns PARK but **loses LEAVE**, and the
alliance drops below 16. The two parking slots are placed to balance that risk
against missing the tape:

| Slot | Robot center | Gap to side wall | Overlap of the tape (X) | Overlap of the zone (Z) |
| --- | --- | ---: | ---: | ---: |
| Inner, primary (B) | (-56, -26) | 5.585 in (14.2 cm) | 5.415 in (13.8 cm) | 11.1 in |
| Outer, support (C) | (-56, -47) | 5.585 in (14.2 cm) | 5.415 in (13.8 cm) | 8.6 in |

The tape overlap uses the conservative X<=-59.585 edge; against the CAD tape
edge it is 5.9 inches. Neither robot needs to be fully inside the 22.69-inch
zone: PARK needs only partial presence, which is why both slots fit.

Those margins set the accuracy the timed drive needs. Toward the side wall,
B travels 109.9 cm in total (leg 2 plus leg 3's lateral part) and C 116.8 cm
(leg 3's lateral part). About **12% overshoot** reaches the wall and about **12%
undershoot** misses the tape, before counting heading drift. Measure the
lateral endpoint on the practice field and adjust the leg distances in INIT
until repeated runs land well inside that band.

Placement in tile terms (CAD tile pitch 23.50 inches): B's midline-side edge is
3.7 inches from the center seam; C has 1.0 inch to the flower bracket and 1.0
inch to the center seam. A 1.0-inch spacer gauge is an allowed alignment device
(G302) and must be taken off the field before the match (G303.C). The loading
zone tape sits about 0.4 inch inside one tile row along the side wall.

[POLLEN_PIT_SHEET.svg](POLLEN_PIT_SHEET.svg) is the simplified alliance version of
this guide: tile grid, both alliances, inner/outer slots, the no-wall strip and
timeline badges. `python3 tools/draw-pollen-pit-sheet.py out.pdf` makes a
printable letter-size copy.

## Blue alliance and physical validation

Rotate the complete red plan 180 degrees: (X,Z) -> (-X,-Z), including the
heading, flowers, start and parking slots. Robot-relative left/right controls
stay the same. The CAD hive center asymmetry is about 0.017 inch and is below
normal setup tolerance; use the actual cell center for aiming.

1. Check the starting footprint against wall, flower and alliance midline.
   Mark the center; verify the supporting shot after a hive tip.
2. With all feed settings zero, test short movements at the selected power.
   Confirm forward/right/left signs and both displacement components.
3. Test each leg on its own (zero the others), then the full route. Record actual endpoint and
   heading drift. Do not assume MAT_TRIAL's unconfirmed coefficients are accurate.
4. Run the full shot and route with the expected load. Coordinate both robots'
   actual timing and parking slots; account for pieces dumped when the hive tips.

`python3 tools/check-pollen-field-routes.py` checks the nominal swept polygons,
initial wall/half-field geometry, parking overlap and route separation.
`python3 tools/draw-pollen-field-routes.py` (needs matplotlib) redraws the SVG. It does
not account for calibration error, field tolerance, flexible parts or drift.
