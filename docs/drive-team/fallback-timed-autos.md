# EMERGENCY ONLY: the timed autos (no odometry)

**Do not pick these for a normal match.** The Odo modes park much more accurately. Use a timed
mode only if the odometry sensor isn't working. You'll know because the Odo mode says
`BLOCKED: ... No goBILDA Pinpoint named "odometry"`, its Status says `PINPOINT FAULT`, or its
Odometry line says `NOT VALID`.

| Instead of | Pick |
| --- | --- |
| Odo ROBOT Shoot + Park | **EMERGENCY Timed Shoot + Park** |
| Odo Supporting Shooter Auto | **EMERGENCY Timed Supporting Shooter** |

Both are in the autonomous list, in the group *Pollen Emergency (timed)*. Every one of these modes
has **EMERGENCY** in its name and an `EMERGENCY ONLY` line on its INIT screen.

## What's the same

- Same job, same lane, same parking slot, same placement on the field. Use the pictures on the
  [Shoot + Park](1-shoot-and-park.md) and [Supporting Shooter](2-supporting-shooter.md) pages.
- Same buttons: **Triangle** = defaults (press it once); **Circle** and **Square** do nothing.
- Same shooting times. The Supporting Shooter still waits 12 s and shoots at 15 s.

## What's different

- **The robot drives by time, not by measuring.** It doesn't know where it is. A low battery,
  slippery tiles, or a robot that isn't perfectly straight at the start all change where it ends up.
  Place it as carefully as you can, front flat against the wall, back (launcher) to the hive.
- It does **not** drive into the wall first to line itself up.
- It drives slower (power 0.25 for Shoot + Park, 0.45 for Supporting Shooter), so the legs take longer.
- The screen has no Odometry line. Mode says `TIMED Shoot + Park` or `TIMED Supporting Shooter`.

## Check the screen

| Line | Should say |
| --- | --- |
| Build | `pollen-hive-v1` |
| EMERGENCY ONLY | `Timed driving, no odometry. Use the Odo mode unless odometry has failed.` |
| Mode | `TIMED Shoot + Park - real motors on START` (or `TIMED Supporting Shooter ...`) |
| Status | `READY - no movement until START` |

After the match, tell a programmer the odometry wasn't working, and where the robot parked.
