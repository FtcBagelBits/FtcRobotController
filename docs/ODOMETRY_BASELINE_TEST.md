# Odometry baseline test

Run this before trusting Pedro Pathing. It answers one question: **is the Pinpoint
odometry wired and configured so that the numbers it reports match what the robot
actually did?** Pedro can only be as good as these numbers.

OpMode: **Odometry Check** (TeleOp, group Pedro), file
`TeamCode/.../teamcode/OdometryCheck.java`.

It sets the Pinpoint up with exactly the values Pedro uses
(`pedroPathing/Constants.java`, `localizerConfig`), so a pass here is a pass for Pedro.
Those values were copied from another robot, so expect to change some of them.

## What you need

- A tape measure and a long straight edge (field wall or a board) to push against.
- Tape marks on the floor: a start line and a line 48 in ahead.
- A square corner (field wall corner or a carpenter's square) for heading.
- A notebook for the numbers in the table at the end.

## Directions (Pinpoint and Pedro agree)

- **X** = forward, **Y** = to the robot's **left**, **heading** = counterclockwise is positive.
- Every number on screen is "since zero". **Cross** zeroes. **Triangle** recalibrates the
  IMU and zeroes; the robot must be completely still when you press it.

## Steps

Steps 1-6 are done by hand **during INIT** (motors are off). Steps 7-8 need START.

| # | Do this | Pass (starting thresholds, our choice) | If it fails |
|---|---------|------|-------------|
| 1 | Select Odometry Check, press INIT, don't touch the robot for 1 s. | `Pinpoint status` = READY. LED on the Pinpoint green. `Pedro drive vs PollenConfig` = match. | "No goBILDA Pinpoint named odometry": on the Driver Hub, Configure Robot, open the active configuration and find the I2C device named `odometry`. Its type must be *goBILDA Pinpoint Odometry Computer*; fix the type, or the name in `Constants.localizerConfig`, so they match. FAULT_X / FAULT_Y / NO_PODS: that pod's cable. |
| 2 | Leave it alone 30 s. | X and Y change < 0.05 in. Heading changes < 0.2 deg. | Heading creeping: press Triangle with the robot still, retest. Still creeping: check the Pinpoint is firmly mounted. |
| 3 | Cross. Push the robot straight forward 48 in along the straight edge. | X = 47.5 to 48.5. Y within +/-0.5. Heading within +/-1. Raw X pod moves about 24,000 ticks; Raw Y pod barely moves. | Y moves instead of X: the pod cables are in each other's ports. Fix the cables, not the code. X negative: flip `xPodDirection` in Constants. X reads about 32 or 72: wrong pod type (`podType`). |
| 4 | Pull it back to the start line. | X back within +/-0.25 of 0. | Doesn't come back: the forward pod is skipping. Check it touches the floor with spring pressure. |
| 5 | Cross. Push the robot straight **left** 48 in. | Y = 47.5 to 48.5. X within +/-0.5. Heading within +/-1. | Y negative: flip `yPodDirection`. |
| 6 | Square the robot to the corner. Cross. Rotate it by hand counterclockwise exactly 90 deg against the corner. | Heading = +89 to +91. | Heading goes negative: stop and check how the Pinpoint is mounted against the goBILDA user guide before changing any code. |
| 7 | START. Robot on the floor, clear space. Cross. Turn **in place** with the right stick, slowly, about 180 deg. Read `Offset suggestion`. Cross and repeat 3 times, both directions. | The three suggestions agree within about 5 mm. | Write the average into `xPodOffset` / `yPodOffset` in Constants, rebuild, repeat step 7. When the suggestion stays within about 5 mm of what is in Constants, it's done. |
| 8 | Cross. Turn in place 3 full turns. Then drive forward, strafe left, turn left with the sticks. | During the turns `From zero ... max` stays < 0.5 in. Stick forward makes X go up, stick left makes Y go up, turning left makes heading go up. | Max large: redo step 7. Robot moves the right way but the number goes the wrong way: that pod's direction (steps 3/5). Robot moves the wrong way: drive motor directions, which PollenBot TeleOp would also show. |

Why step 7 works: if the offsets are right, the point the Pinpoint tracks is the point the
robot turns about, so turning in place does not move it. If they are wrong, the tracked
point swings around and lands somewhere else. Where it lands tells us exactly how far off the
offsets are. Pedro's own Pinpoint tuner uses the same idea.

## Record

| Date | Step | Reading | Pass? | Change made |
|------|------|---------|-------|-------------|
| | | | | |

## After this passes

Next, in order, each checked on the robot before moving on:

1. **Pedro reads the same numbers.** Pedro Test OpMode shows the same pose as this check
   when driven.
2. **Tune Pedro** with AutoTune (Mecanum, then Foresight), so paths track.
3. **One short path**: a straight 24 in line, measured end error.
4. **Option A (Park)** on Pedro, then **B** and **C**, each compared against the timed
   version on the field.
