# Pollen autos with odometry

Three new autonomous OpModes, in the group **Pollen Odometry**, do the same jobs as the timed
ones, but drive each leg to a **measured** end point with the Pinpoint instead of driving
for a guessed time:

| Odometry OpMode | Same job as | Code |
|---|---|---|
| Odo ROBOT Park | Pollen ROBOT Park (A) | `PollenOdoPark`, `PollenOdoParkTuning` |
| Odo ROBOT Shoot + Park | Pollen ROBOT Shoot + Park (B) | `PollenOdoRobotShoot` |
| Odo Supporting Shooter Auto | Pollen Supporting Shooter Auto (C) | `PollenOdoSupportingShooter` |

The timed OpModes are unchanged and stay as the fallback until the odometry ones have beaten
them on the field. Run [the odometry baseline test](ODOMETRY_BASELINE_TEST.md) first.

## What is the same

- INIT menus, buttons, memory and the time-budget check. B and C keep every row, and START
  still refuses settings that do not fit.
- Shooting: launcher speed, spin-up, feed time, intake and agitator power.
- C still starts feeding at 15 s.
- Nothing turns the chassis. Angles are directions of travel: + left, - right, 0 straight ahead.

## What is different

- **Leg cm and angle are real distances.** "Leg 3 172 cm @ +17 deg" now means the robot ends up
  172 cm away in that direction. To move the end point, measure how far off it is and change the
  number by that much.
- **Leg 3 defaults are the CAD plan, not the field-tuned timed numbers.** The timed leg 3 values
  were shortened during field tuning (by 17-21%), most likely to make up for timed driving going
  farther than its model predicted. As real distances they would stop B about 19 in short and C about 8 in
  short of the planned park spot. The plan is the route `tools/check-pollen-field-routes.py`
  checks against the field CAD:
  - B, Square (field preset): legs 10 cm @ 0, 63.5 cm @ +90, 217.5 cm @ +12.3.
  - C: legs 20.3 cm @ 0, 118.0 cm @ -81.85.
  - B Triangle: the same 2-foot move as before, 60.96 cm @ +25.87.
- **The Calibration row is now "Wall square first (0=off, 1=on)"** for B and C, and is on by default.
- **Odo ROBOT Park:** LEFT/RIGHT changes the distance by 5 cm (default 55 cm: the mat model's
  estimate for the tested 2 s at 0.20 power; that power is below the measured ones, so check it in test 1). UP/DOWN delay, L1/R1 power, Triangle reset are as before.
- **No end hold for B and C.** A leg only ends when the robot is stopped within 0.5 in of its end point.

## Wall squaring (B and C)

The robot starts with its back against the end wall. At the very start it backs into that wall at
0.30 power for 0.35 s. The wall stops it, and if one back corner was touching first, the push
turns the robot until its back is flat against the wall. Then the odometry is zeroed there, so
"straight ahead" means square to the wall. Every leg is measured from that point.

Why at the start and not just before the first leg: it squares the shot as well as the route. For C it
happens inside the 12 s start delay, so it costs no time. For B (no delay) it moves the shot
0.45 s later. If the robot gets nudged while shooting, the odometry measures it, and the legs still
end at the same spots.

What it fixes and what it doesn't: it fixes the starting **angle**, the error that grows with distance.
A 3 deg crooked start puts B's field-preset park spot about 5 in off (98 in of travel x 3 deg);
the simulation in `tools/pollen-tests` shows the squared run within 0.5 in. It does not fix
left/right placement. That is still set by hand at the start line.

It only works if the robot's back is flat and square to the drivetrain, and nothing on the back
reaches the wall first. Check this before relying on it. To compare, set the row to 0.

## The drive controller

`PollenOdometryDrive` drives straight at each end point: full leg power while far away, then
slowing (0.04 power per inch) to no less than 0.08, until within 0.5 in. It holds the zeroed
heading. Every number is a named constant at the top of the file:

| Symptom | Change |
|---|---|
| Stops short and sits there | raise `MIN_POWER` (0.08) |
| Overshoots and comes back | lower `KP_POWER_PER_INCH` (0.04) or the leg power |
| Ends visibly rotated | raise `HEADING_KP` (1.0) |
| Wobbles in heading | lower `HEADING_KP` |
| Squaring doesn't straighten it | raise `SQUARE_SECONDS` (0.35) or `SQUARE_POWER` (0.30) |

Pedro Pathing's path follower is not used yet: it needs AutoTune values for this robot, and the
ones in `pedroPathing/Constants` belong to another robot. The pose already comes from Pedro's
Pinpoint localizer, so switching to the follower later changes only how each leg is driven.

Safety: a leg that hasn't arrived within 2 x its estimate + 1 s gives up, and the next leg
starts toward its own end point. If the pose is lost (no reading), everything stops. The 27 s hard stop still applies.

## Time budget

Odometry legs are budgeted at an **estimate**: 120 cm/s per unit of power, plus 0.2 s per leg
for slowing down. If the tuned timed routes reached their planned spots, they moved at 125-150. **C is tight**: planned 25.5 s of the
26 s allowed. A simulated robot slower than that estimate finished at 26.6 s, with the hard stop
at 27 s. If C hits the hard stop on the field, take 0.5 s off its feed time or raise its power.

## First tests

Do these on the practice field if you can. A slippery floor makes squaring and stopping look worse than they are.

1. **Odo ROBOT Park**, defaults. Tape-measure the travel: 55 cm, within 1 in. Robot should not drift sideways.
2. **Odo ROBOT Shoot + Park**, Triangle defaults, **feed seconds 0** (skips the shot, saves 13 s).
   Ends 21.6 in forward and 10.5 in left of the start, within 1 in, not rotated.
3. **Wall square**, same as 2, but start the robot visibly crooked (3-5 deg) against a wall. With the
   row on, the robot should square up, then drive straight. With it off, it ends off by about
   (distance x angle). If the robot does anything strange while backing up, turn it off and tell us.
4. On the field: **B Square preset** and **C**. Mark where each robot actually parks and compare
   with the planned spot. Check the C finish time on the Driver Hub.

Write down for each run: OpMode, settings changed, where it ended (fwd/left from the start), time,
and anything odd. The numbers on the Driver Hub ("Inches to go", "Warning") help.
