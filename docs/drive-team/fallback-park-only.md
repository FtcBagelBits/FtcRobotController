# Park only (no shooting)

**Odo ROBOT Park** and **Pollen ROBOT Park** don't shoot. They wait for the start delay, then
drive straight forward and stop. We use them for testing, or if the launcher is broken and we
only want the LEAVE points.

| Mode | How it drives | Default |
| --- | --- | --- |
| **Odo ROBOT Park** (group *Pollen Odometry*) | Measures the distance with odometry | 55 cm (about 22 in) forward, power 0.60 |
| **Pollen ROBOT Park** (group *Pollen*) | Drives for a set time | 2.0 s forward at power 0.20 (about 55 cm) |

## Buttons during INIT

| Button | Odo ROBOT Park | Pollen ROBOT Park |
| --- | --- | --- |
| D-pad UP / DOWN | Start delay +/- 0.5 s | Start delay +/- 0.5 s |
| D-pad RIGHT / LEFT | Distance +/- 5 cm | Drive time +/- 0.05 s |
| R1 / L1 | Power +/- 0.05 | Power +/- 0.05 |
| Triangle | Back to the defaults | Back to the defaults |

## Placement

These drive **straight ahead** from wherever the robot is. Point the robot so that straight ahead
is clear and ends partly in the loading zone if you want PARK points too. Ask a programmer
how far to set the distance for that start. Don't drive the straight diagonal from the middle of
the audience wall: it hits the hive frame.

Check for **Build `pollen-odo-v7`** and `READY - waiting for START` before START.
