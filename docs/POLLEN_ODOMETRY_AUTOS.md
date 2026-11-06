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
  - B default (Triangle): legs 10 cm @ 0, 63.5 cm @ +90, 217.5 cm @ +12.3.
  - C: legs 20.3 cm @ 0, 118.0 cm @ -81.85.
  - B Circle (leave-the-wall fallback): the 2-foot move, 60.96 cm @ +25.87. Square does nothing.
- **The Calibration row is now "Wall square first (0=off, 1=on)"** for B and C, and is on by default.
- **Odo ROBOT Park:** LEFT/RIGHT changes the distance by 5 cm (default 55 cm: the mat model's
  estimate for the tested 2 s at 0.20 power; that power is below the measured ones, so check it in test 1). UP/DOWN delay, L1/R1 power, Triangle reset are as before.
- **Faster: leg power defaults to 0.60 for A, B and C** (timed used 0.20-0.45), and the power row
  goes up to 0.80. Speed no longer costs accuracy because the robot checks where it is every loop
  and brakes on approach. It does cost stopping distance, which is what the controller below handles.
- **No end hold for B and C.** A leg only ends when the robot has stopped within 0.5 in of its end point.

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

`PollenOdometryDrive` drives straight at each end point and checks where the robot is every loop:

- **Start of a leg:** power ramps up at 2.0 per second, so the wheels don't spin. That's 0.3 s to reach 0.60.
- **Cruise:** the leg's power. Heading is held at the zeroed heading.
- **Approach:** power = 0.20 x sqrt(inches to go), an even braking rate. That's 0.60 until 9 in out, 0.40 at 4 in,
  0.20 at 1 in, never below 0.08. Braking uses where the robot will be in 0.15 s at its current
  speed, so it doesn't coast past.
- **Done:** within 0.5 in of the end point **and** stopped. Inside that circle the power is zero; if
  the robot rolls back out, it drives back in.

Every number is a named constant at the top of the file:

| Symptom | Change |
|---|---|
| Stops short and sits there | raise `MIN_POWER` (0.08) |
| Overshoots and comes back | raise `LOOKAHEAD_SECONDS` (0.15) or lower `SLOWDOWN_POWER_PER_SQRT_INCH` (0.20) |
| Slow, careful final approach | raise `SLOWDOWN_POWER_PER_SQRT_INCH` |
| Wheels spin at the start of a leg | lower `RAMP_POWER_PER_SECOND` (2.0) |
| Ends visibly rotated | raise `HEADING_KP` (1.0) |
| Wobbles in heading | lower `HEADING_KP` |
| Squaring doesn't straighten it | raise `SQUARE_SECONDS` (0.35) or `SQUARE_POWER` (0.30) |

In the simulation (assumed robot, not ours), at 0.60 and 0.80 power: overshoot at most 0.6 in, end error 0.3-0.5 in,
and C stays within 0.9 in of its straight-line path. C's start gap to the flower is only about 1 in,
so watch leg 1 of C on the real robot.

Pedro Pathing's path follower is not used yet: it needs AutoTune values for this robot, and the
ones in `pedroPathing/Constants` belong to another robot. The pose already comes from Pedro's
Pinpoint localizer, so switching to the follower later changes only how each leg is driven.

Safety: a leg that hasn't arrived within 2 x its estimate + 1 s gives up, and the next leg
starts toward its own end point. If the pose is lost (no reading), everything stops. The 27 s hard stop still applies.

## Time budget

Odometry legs are budgeted at an **estimate**: 120 cm/s per unit of power, plus 0.2 s per leg
for slowing down. If the tuned timed routes reached their planned spots, they moved at 125-150.
**C is still the tight one:** it can't start moving until its feed ends at 22.5 s. Planned 24.8 s; the
simulation finished at 25.6-26.2 s (0.60-0.80 power, quick or sluggish robot), with the hard stop at 27 s.
The time goes to the two stops (end of leg 1 and end of leg 3) as much as to cruising, so more power helps
only a little. If C hits the hard stop on the field, take 0.5 s off its feed time.

## First tests

Do these on the practice field if you can. A slippery floor makes squaring and stopping look worse than they are.

1. **Odo ROBOT Park**, defaults. Tape-measure the travel: 55 cm, within 1 in. Robot should not drift sideways.
   Try it at 0.30 power first (L1 to lower), then the 0.60 default.
2. **Odo ROBOT Shoot + Park**, **Circle** (short move), **feed seconds 0** (skips the shot, saves 13 s).
   Ends 21.6 in forward and 10.5 in left of the start, within 1 in, not rotated.
3. **Wall square**, same as 2, but start the robot visibly crooked (3-5 deg) against a wall. With the
   row on, the robot should square up, then drive straight. With it off, it ends off by about
   (distance x angle). If the robot does anything strange while backing up, turn it off and tell us.
4. On the field: **B with its defaults** (the full route) and **C**. Mark where each robot actually parks and compare
   with the planned spot. Check the C finish time on the Driver Hub.

Write down for each run: OpMode, settings changed, where it ended (fwd/left from the start), time,
and anything odd. The numbers on the Driver Hub ("Inches to go", "Warning") help.
