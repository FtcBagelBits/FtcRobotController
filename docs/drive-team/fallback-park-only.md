# Park only (no shooting)

**Odo ROBOT Park** doesn't shoot. It waits for the start delay, then drives straight **backward**
(launcher end first, like every auto) and stops. We use it for testing, or if the launcher is broken and we only want the LEAVE points.

**EMERGENCY Timed Park** does the same job by time. It is for **emergencies only**: use it when
the odometry sensor isn't working.

| Mode | How it drives | Default |
| --- | --- | --- |
| **Odo ROBOT Park** (group *Pollen Odometry*) | Measures the distance with odometry | 55 cm (about 22 in) backward, power 0.60 |
| **EMERGENCY Timed Park** (group *Pollen Emergency (timed)*) | Drives for a set time | 2.0 s backward at power 0.20 (about 55 cm) |

## Buttons during INIT

| Button | Odo ROBOT Park | EMERGENCY Timed Park |
| --- | --- | --- |
| D-pad UP / DOWN | Start delay +/- 0.5 s | Start delay +/- 0.5 s |
| D-pad RIGHT / LEFT | Distance +/- 5 cm | Drive time +/- 0.05 s |
| R1 / L1 | Power +/- 0.05 | Power +/- 0.05 |
| Triangle | Back to the defaults | Back to the defaults |

## Placement

These drive **straight backward** from wherever the robot is: the launcher end goes first. Point
the robot's back where you want it to go, so that straight behind it is clear and ends partly in the loading zone if you want PARK points too. Ask a programmer
how far to set the distance for that start. Don't drive the straight diagonal from the middle of
the audience wall: it hits the hive frame.

Check for **Build `pollen-hive-v1`** and `READY - waiting for START` before START.
