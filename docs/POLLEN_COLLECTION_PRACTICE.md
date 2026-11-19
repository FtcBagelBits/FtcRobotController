# Garden pickup and flower drain practice

Select **Calibration Garden Pickup** or **Calibration Flower Drain** under Autonomous. Each
starts at the pickup site. The older **Calibration Garden + Return** starts at the shooting
position. These practice modes use the four drive motors, front intake and Pinpoint.
Flower also needs the positional servo named `pollenDrainer`. Neither mode uses the hive
camera, launcher or agitator.

## Mark the starting pose

* **Garden.** Point the intake squarely at the wall behind the garden balls. For repeatable
  placement, start with the front chassis edge **15.24 cm from the garden wall** and the
  right chassis edge **2.54 cm from the adjoining side wall**. The routine always strafes
  right as viewed from behind the robot. Tape both starting edges and set up the
  four balls in the same positions each run.
* **Flower.** With drive power off, find the chassis position at which the lowered drainer
  is centered and fully inserted without flexing. Mark a chassis edge on the tile there.
  Mark a parallel start line **24 cm back from that seated mark**, away from the flower.
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
3. Press START. The Driver Hub shows the phase, elapsed time, and centimeters forward
   and left relative to the starting pose. At DONE, count balls
   retained. Press STOP, replace the balls, place the robot at the same marks and INIT again.
   For the next run, change one setting so the difference is easy to explain.
4. For each candidate setting, record balls available/retained, elapsed time, exit pose
   relative to the marks, and any contact or jam. Repeat the best profile three times.
   Each run records build, mode, overrides, phase changes, and pose about every 0.1 s in a bounded CSV under
   `FIRST/pollen-hive/auto`. After STOP, use the usual `bash tools/ftc.sh logs NEW_FOLDER`
   to download those files and both saved profiles (`garden-practice.properties` and
   `flower-practice.properties` once saved).

## What to tune

| Mode | Default sequence | INIT adjustments |
| --- | --- | --- |
| Garden | Run intake at 1.0. Strafe right at up to 0.30 until side contact, then drive forward at up to 0.25 until garden wall contact. Back off 3 in at up to 0.25 and return to garden wall contact once for two total inward passes. Stop in the corner. | Backoff and re-entry repeats (0–8), backoff distance in inches, strafe power, forward approach power, backoff power, intake power. |
| Flower | Start intake at 1.0, lower to 0.02 and wait 0.4 s. Approach at up to 0.20, switching to 0.15 near the flower. After contact, swing 3.5 cm left and 3.5 cm right at up to 0.40 three times, center, then withdraw 18 cm. Drive 7 cm forward and back twice at up to 0.30 and dwell 1 s. Raise the drainer and stop the intake at DONE. | Stand-off, distance at which to switch to seating power, approach and seating power, lower dwell and position, cycle count (0–8), lateral amplitude and power, withdrawal, cleanup stroke/count/power, final dwell, and intake power. |

For a **first garden run**, use zero repeats to see the side-wall and garden-wall approaches
once. The right strafe and forward passes have no programmed distance or time cutoff. Once
the strafe reaches the side wall, the robot drives along it toward the garden wall. Each backoff
is measured from garden wall contact.
The intake runs throughout the strafe, all forward passes, and every backoff.

For a **first flower run**, set both cycle counts to 0. Check arm height, approach and
withdrawal with the real opening. Tune the final approach power so the drainer seats gently.
Odometry targets for the swings
and withdrawal use the position actually reached. Add one swing cycle and observe clearance
before trying the default three. One cycle visits left and right. After all cycles it tries
to center once, then withdraws straight from the position it reached. Increase cleanup
passes only if loose balls remain. The nearest
cleanup point is withdrawal minus stroke from the seated position (11 cm by default).
The drainer stays down and the intake runs through withdrawal, every cleanup stroke and the final dwell.

When an approach stops making progress for about 0.75 s, the routine treats it as wall contact
if the robot is settled and still aligned. The trace marks it `CONTACT LIMIT`. The final flower
approach continues until contact. A stalled flower swing
advances to the next swing target. Backoff, withdrawal, and cleanup require reaching their
targets. Pinpoint faults and control-loop stalls stop the sequence.

Saved intake, drive, and compatible flower adjustments still load at INIT. Retired garden
placement, search distance, and pass settings and the old flower push timer are ignored. Set the garden
repeat count and backoff distance at INIT to override the defaults.

## Beta match autos

Primary Shooter + Garden uses one forward pass after right-wall contact and immediately
starts its return after garden-wall contact. Secondary Shooter + Flower uses two left-right swings
and one forward-back cleanup cycle. The **Beta garden total inward passes**, **Beta flower left-right
cycles**, and **Beta flower forward-back cycles** INIT rows tune those counts.
The garden or flower tuning rows follow **Alliance** and **Execution** in each beta INIT menu.
In Secondary Shooter + Flower, **Sideways power** defaults to 0.40. D-pad left/right adjusts it by
0.05, up to 0.50. **Intake power** during collection and **Beta shooting intake power** both
default to 1.00. Press **Square** to save match edits or run them immediately after editing.
After the first volley, each backs clear of the end wall and aligns with its collection lane.
While it leaves, the **Clear-out s after volley** row (3 s) keeps the agitator and launcher running
together to empty the robot; on the secondary both can still be running for the first part of the
flower approach. The same clear-out runs after the return volley while the robot waits
to park.
The two **Match collection entry** rows tune where the match auto starts collection, in route
coordinates measured from the shooting pose. In particular, adjust the secondary's lateral row
to center the drainer in the flower opening. The default offsets come from the practice traces
and field layout and need a field check.

Each mode backs clear before crossing back to the original shooting pose. If it is in position
and heading is more than 2° off, it turns toward 0° in place. It then settles and checks for fresh
Hive UP before feeding again. Both pre-spin the flywheel during return. They interrupt the active
phase when the measured parking duration plus the configured **Park margin s** remains. The margin defaults to 1.0 s and moves in 0.5 s steps. The **Match travel and park
power** row defaults to 1.00 for the outbound, return,
and park legs. The controller still ramps up and slows for target poses. Garden and flower
contact and collection use the powers in the table above.
Each beta saves its collection settings with its route. The isolated practice modes keep their
own saved profiles and cycle defaults.

For the drive controller, the vinyl and tile test plan, and a proposed comparison with
Pedro's full follower, see [Collection motion and Pedro Pathing](research/collection-motion-and-pedro.md).
