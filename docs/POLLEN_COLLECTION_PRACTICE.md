# Garden pickup and flower drain practice

Select **PRACTICE Garden Pickup** or **PRACTICE Flower Drain** under Autonomous. Each
starts at the pickup site. The older **TEST Hive garden + return** starts at the shooting
position. These practice modes use the four drive motors, front intake and Pinpoint.
Flower also needs the positional servo named `pollenDrainer`. Neither mode uses the hive
camera, launcher or agitator.

## Mark the starting pose

* **Garden.** Point the intake squarely at the garden wall on the pickup lane. Tape a mark
  for the front chassis edge and a second for one side so both distance and heading repeat.
  The default matches point D of the existing garden route. For the team's 18-inch chassis,
  leave **15.24 cm from the front chassis edge to the inside face of the end wall** and
  **2.54 cm from the nearest chassis side to the adjoining side wall**. The first pass
  aims for 2.54 cm front clearance. Use the same rigid chassis edges every run.
  Set up the four balls in the same positions. If the intake extends past the front edge,
  measure its reach and increase final clearance as needed.
* **Flower.** With drive power off, find the chassis position at which the lowered drainer
  is centered and fully inserted without flexing. Mark a chassis edge on the tile there.
  Mark a parallel start line **20 cm back from that seated mark**, away from the flower.
  Line up the same chassis edge on it, front intake and drainer facing straight into the
  opening. This assumes forward chassis travel inserts the drainer. The stand-off depends
  on the mounted arm, so measure it on the robot. A tape outline on a bare floor can test
  directions and motion, while an actual flower is needed to tune release and count balls.

## Run and repeat

1. Place the robot at its marks before INIT. Check the build and wait for Pinpoint READY.
   The drainer is up and motors are idle during setup. D-pad up/down selects a row,
   left/right changes its value, Triangle resets that row, and Circle resets the mode's rows.
2. Press **Square** to save this mode's settings to the Hub. You can test edits immediately.
   The next INIT reloads the last saved values for the selected mode. Square after a reset
   also makes the reset persist.
3. Press START. Read the phase and elapsed time on the Driver Hub. At DONE, count balls
   retained. Press STOP, replace the balls, place the robot at the same marks and INIT again.
   For the next run, change one setting so the difference is easy to explain.
4. For each candidate setting, record balls available/retained, elapsed time, exit pose
   relative to the marks, and any contact or jam. Repeat the best profile three times.
   Each run records build, mode, overrides and phase times/poses in a bounded CSV under
   `FIRST/pollen-hive/auto`. After STOP, use the usual `bash tools/ftc.sh logs NEW_FOLDER`
   to download those files and both saved profiles (`garden-practice.properties` and
   `flower-practice.properties` once saved).

## What to tune

| Mode | Default sequence | INIT adjustments |
| --- | --- | --- |
| Garden | With intake at 0.50, drive in at 0.20, dwell 0.5 s, back off 10 cm at 0.25, then re-enter. Three total inward passes. Back out 12.7 cm on the last pass. | Start gap, final clearance, inward power, total passes (1–8), backoff distance/power, lateral step between passes, intake power, dwell, final exit distance. |
| Flower | Start intake at 1.0, lower to 0.02 and wait 0.4 s. Drive the first 17 cm at 0.20, then push at 0.15 for 0.30 s. From the reached position swing 3 cm left and 3 cm right three times, center, then withdraw 18 cm. Raise the drainer and drive 7 cm forward and back twice. Dwell 0.5 s, then stop intake. | Stand-off, final timed portion in cm, approach power, lower dwell and position, push power/time, cycle count (0–8), lateral amplitude/power, withdrawal, cleanup stroke/count/power, and final dwell. |

For a **first garden run**, set passes to 1 and lateral step to 0. Tune the depth until
the intake reaches the balls. Add retries if balls remain. A nonzero lane step shifts
after backing off and accumulates on each retry. Positive is physical robot left as
viewed from behind it. Check the last lane clears the side wall, especially on a mirrored
setup. With zero step and the default exit distance the robot finishes back at its mark.

For a **first flower run**, set both cycle counts to 0. Check arm height, approach and
withdrawal with the real opening. The last centimeters are a timed push, with no contact
sensor. Tune power and duration until it seats gently. Odometry targets for the swings
and withdrawal use the position actually reached. Add one swing cycle and observe clearance
before trying the default three. One cycle visits left and right. After all cycles it
returns to center once. Increase cleanup passes only if loose balls remain. The nearest
cleanup point is withdrawal minus stroke from the seated position (11 cm by default).
Full inward intake continues through withdrawal, every cleanup stroke and the final dwell.

## Calling the routines from a shooter later

The two match shooters still use their existing routes. Once a profile is repeatable,
record its settings and the robot's entry and exit marks, then adopt those settings as
code defaults. In the shooter, navigate to the entry pose and align the physical intake.
Construct `PollenCollectionSequence` with the kind, its settings and the current
`PollenFacing.routePose` (inches, radians). Tick it with the existing odometry and clock,
apply wheel commands through `PollenFacing.wheelPowers`, and apply intake and drainer
commands until DONE or fault. Exit in the same odometry frame. The caller owns its
remaining time budget and can stop the sequence at any point. No Pinpoint reset is needed.
