# Fallback: the timed autos (no odometry)

**Use these only if the odometry sensor isn't working.** You'll know because the Odo mode says
`BLOCKED: ... No goBILDA Pinpoint named "odometry"`, or its Odometry line says `NOT VALID`.

| Instead of | Pick |
| --- | --- |
| Odo ROBOT Shoot + Park | **Pollen ROBOT Shoot + Park** |
| Odo Supporting Shooter Auto | **Pollen Supporting Shooter Auto** |

Both are in the autonomous list, group *Pollen* (no "Odo").

## What's the same

- Same job, same lane, same parking slot, same placement on the field. Use the pictures on the
  [Shoot + Park](1-shoot-and-park.md) and [Supporting Shooter](2-supporting-shooter.md) pages.
- Same buttons: **Triangle** = defaults (press it once), **Square** does nothing,
  **Circle** = short fallback (Shoot + Park only).
- Same shooting times. The Supporting Shooter still waits 12 s and shoots at 15 s.

## What's different

- **The robot drives by time, not by measuring.** It doesn't know where it is. A low battery,
  slippery tiles, or a robot that isn't perfectly straight at the start all change where it ends up.
  Place it as carefully as you can, flat against the wall.
- It does **not** back into the wall first.
- It drives slower (power 0.25 for Shoot + Park, 0.45 for Supporting Shooter), so the legs take longer.
- The screen has no Odometry line. Mode says `Shoot + Park` or `Supporting Shooter` without `ODOMETRY`.

## Check the screen

| Line | Should say |
| --- | --- |
| Build | `pollen-odo-v5` |
| Mode | `Shoot + Park - real motors on START` (or `Supporting Shooter ...`) |
| Status | `READY - no movement until START` |

After the match, tell a programmer the odometry wasn't working, and where the robot parked.
