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
- [Game Details TU05, section 10.5.4](https://ftc-resources.firstinspires.org/ftc/game/manual-10):
  PARK requires partial presence in the alliance loading zone; LEAVE requires
  no longer contacting the perimeter wall.
- [Game Rules TU06, G304](https://ftc-resources.firstinspires.org/ftc/game/manual-11):
  starting robot contacts the wall, stays entirely on its alliance half and
  does not contact a flower or its scoring volume.

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

1. Shoot from the starting position.
2. Strafe left **63.50 cm (25 inches)** to X=-37.742, holding the same heading.
3. Travel **227.25 cm at +11.775 degrees** to (-56,-26).
4. Stop. The nominal footprint overlaps the loading zone without pressing the
   side wall, leaving room for the supporting robot's separate parking slot.

The preset uses MAT_TRIAL and power 0.25. Its calculated drive time is about
9.69 seconds; with the default 10-second feed, four-second spinup allowance and
0.5-second hold, the total is about **24.19 seconds**. Short loop transitions
consume part of the reserved margin. Triangle restores B's tested short baseline.

Swept-square checks clear the fixed frame, both hive projections and all four
flowers. The smallest nominal obstacle clearance along this two-leg route is
about **8.18 inches (20.8 cm)** to the red side-wall flower. This does not account
for loose game pieces or another robot's route.

## Supporting shooter: offset and aim, then clear the flower

The initially downward-facing red cell is approached from the scoring end wall.
A square, directly centered start at **(-12.742, -61.585)** overlaps the scoring
wall's flower by about **1.68 inches** in the horizontal projection. A purely
rightward strafe also misses the loading zone's row. A straight right-forward
line from an offset start would clip the flower while departing.

The supporting preset therefore uses this **different starting pose**, chosen
for nominal clearance and requiring a physical check:

- Robot center: **X=-10.35, Z=-61.124 inches**.
- Relative to the directly aligned start, offset **6.07 cm toward the field
  centerline**; rotate about **3.014 degrees to robot-right**, aiming back at
  the red cell that the partner will tip upward.
- Let the rear corner contact the end wall. The rear edge is no longer square
  to the wall. Do not place it centered/square and use these route settings.
- The aiming ray assumes a launcher on the robot centerline and an upright
  opening-center horizontal projection near **(-12.742,-15.7)**. The Z target
  is estimated from the front rib of the opposite upright cell in CAD. Confirm
  the actual shot after tipping; CAD does not validate launch speed/elevation.

This fits the nominal field but has only **0.93 inch (2.37 cm)** flower clearance
and **0.89 inch (2.26 cm)** clearance to the alliance centerline. These margins
are small relative to field variation. Check the full real robot, any extended
intake, wall and flower before using this pose. If it does not fit, revise the
pose and recompute the route; do not start partly across the alliance boundary.
The offset/aim solution was selected to resolve the centered-start conflict.

**Supporting route, after its stationary shot:**

1. Drive **20.32 cm forward** along the already aimed chassis heading to clear
   the flower. This is a translation; the code does not turn the robot.
2. Travel **115.93 cm at -79.261 degrees** relative to that heading. This is
   mostly robot-right, with **21.60 cm forward / 113.90 cm right** displacement.
3. Stop at nominal **(-56,-47)**. The rotated footprint partially overlaps the
   loading zone and has about **2.54 inches (6.45 cm)** of separation from the
   primary robot's nominal parked footprint.

The first leg's minimum nominal flower clearance is 2.37 cm; after that leg,
the second leg clears it by at least **3.11 inches (7.9 cm)**. The two prescribed
routes' swept footprints do not overlap. A different partner's actual route
still needs coordination. These coordinates describe two parking slots, not
an official team-specific assigned spot.

Supporting defaults use MAT_TRIAL, power **0.45**, 15 seconds before feeding,
and 7.5 seconds of feeding. The higher power is required by the short remaining
time budget and has not been validated for this sideways move. Calculated total
is about **25.8 seconds**, including the hold, below the existing 27-second
cutoff and the editor's stricter <26-second planning limit.

The flywheel alone spins from approximately t=11 to t=15. Feeding starts no
earlier than t=15 after speed qualification, and runs for 7.5 seconds. Drive
starts at approximately t=22.5, with all shooter outputs off. If spinup fails,
the robot waits through the feed slot before departing. It does **not** verify
that the partner tipped the hive; the wait is an open-loop coordination plan.

## Blue alliance and physical validation

Rotate the complete red plan 180 degrees: (X,Z) -> (-X,-Z), including the aimed
heading, flowers, start and parking slots. Robot-relative left/right controls
stay the same. The CAD hive center asymmetry is about 0.017 inch and is below
normal setup tolerance; use the actual cell center for aiming.

1. Check the starting footprint against wall, flower and alliance centerline.
   Mark the center and heading; verify the supporting shot after a hive tip.
2. With all feed settings zero, test short movements at the selected power.
   Confirm forward/right/left signs and both displacement components.
3. Test each clearance leg, then the second leg. Record actual endpoint and
   heading drift. Do not assume MAT_TRIAL's unconfirmed coefficients are accurate.
4. Run the full shot and route with the expected load. Coordinate both robots'
   actual timing and parking slots; account for pieces dumped when the hive tips.

`python3 tools/check-pollen-field-routes.py` checks the nominal swept polygons,
initial wall/half-field geometry, parking overlap and route separation. It does
not account for calibration error, field tolerance, flexible parts or drift.
