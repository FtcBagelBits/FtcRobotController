# Pollen autonomous code

Build identifier: **pollen-odo-v9** (`PollenConfig.BUILD`). The match autos are the odometry
ones (below). Three timed OpModes are kept for **emergencies only** (a match where the Pinpoint
is not working), in the group *Pollen Emergency (timed)*: `EMERGENCY Timed Park`,
`EMERGENCY Timed Shoot + Park`, and `EMERGENCY Timed Supporting Shooter` (before `pollen-odo-v9`
they were named `Pollen ROBOT Park`, `Pollen ROBOT Shoot + Park` and
`Pollen Supporting Shooter Auto`). The PollenBot TeleOp itself
(`../BioBuzzStarterbotOpMode.java`, outside this package) uses `PollenTeleOpTuning`
for its INIT launcher-speed and drive-power menu and shows the same build id.

**Odometry versions** (group *Pollen Odometry*): `Odo ROBOT Park`, `Odo ROBOT Shoot + Park`,
`Odo Supporting Shooter Auto` do the same jobs, driving each leg to a measured end point. See
[docs/POLLEN_ODOMETRY_AUTOS.md](../../../../../../../../../docs/POLLEN_ODOMETRY_AUTOS.md).

## Where to start reading

New to the code? [docs/POLLEN_START_HERE.md](../../../../../../../../../docs/POLLEN_START_HERE.md)
walks through it with questions and exercises. In short:

Timed driving is a bridge until odometry and Pedro Pathing work. Read the parts
that carry over first; skim or skip the bridge code.

| Read | File | What it shows | Future |
| --- | --- | --- | --- |
| 1 | `../BioBuzzStarterbotOpMode.java`, `launch()` | When to feed the launcher: up to speed or after the backup timer | Carries over |
| 2 | `PollenSequence.java`, `tick()` and `compute()` | Each loop: look at the clock, return what every motor does now; one short method per kind of step | Carries over |
| 3 | `PollenShooterOpMode.java` | The OpMode lifecycle: init, init_loop, start, loop, stop; `loop()` is read, decide, act, report | Carries over |
| 4 | `PollenTeleOpTuning.java` | An INIT menu: select a row, edit it, Triangle resets, START locks | Carries over |
| 5 | `PollenFieldGeometry.java` | Field positions from the official CAD, in inches from the center | Becomes path targets |
| skim | `PollenShootTuning.java`, `PollenParkTuning.java`, `PollenRobotPark.java` | B/C and A menus, presets and time budgets | Mostly replaced |
| skip | `PollenDriveCalibration.java` | Turns a distance and angle into seconds and wheel powers (MAT_TRIAL model) | Bridge code, frozen |

Read the [operator manual](../../../../../../../../../docs/POLLEN_AUTONOMOUS.md),
[calibration analysis](../../../../../../../../../docs/POLLEN_MAT_CALIBRATION.md),
[field routes](../../../../../../../../../docs/POLLEN_FIELD_ROUTES.md),
[field layout](../../../../../../../../../docs/POLLEN_FIELD_LAYOUT.md),
[tested robot behavior](../../../../../../../../../docs/POLLEN_ROBOT_BEHAVIOR.md), and
[student explanation](../../../../../../../../../docs/POLLEN_AUTONOMOUS_EXPLAINED.md).
The experimental profiles/routes are not validated robot settings.

## Code map

| File | Responsibility |
| --- | --- |
| PollenRobotPark | Option A FTC lifecycle, four drives, INIT controls and distance estimate |
| PollenRobotShoot | Registered Option B wrapper |
| PollenSupportingShooter | Registered Option C wrapper |
| PollenShooterOpMode | Shared B/C hardware lifecycle, isolated memory, telemetry and faults |
| PollenConfig | Shared tested drive mapping; unchanged A default power/time |
| PollenParkTuning | A controller editing, bounds, delay budget and lock |
| PollenShootTuning | B/C menus (launcher, feed, power, three legs' cm/angle), defaults, field presets, budget and route construction |
| PollenDriveCalibration | Unchanged baseline; opt-in B / default C mat trial; A forward estimate |
| PollenFieldGeometry | Nominal CAD-derived start/parking coordinates and preset displacements |
| PollenSequence | Hardware-free step runner. Four kinds of step (`Kind`: DRIVE, SHOOT, MOVE, ZERO_ODOMETRY), one short method each (`driveStep`, `shootStep`, `moveStep`, `zeroOdometryStep`); stall/STOP/deadline checks; mixer |
| PollenOdometryDrive | Odometry legs: drive-to-point controller (`command()` returns a `Command`: forward, strafe, turn, inchesToGo), time estimates, wall-square and zero constants |
| PollenOdometryHardware | Opens the Pinpoint through Pedro's localizer (pedroPathing/Constants); pose telemetry |
| PollenOdoPark, PollenOdoParkTuning | Option A with odometry: zero, drive a measured distance, stop; distance menu |
| PollenOdoRobotShoot, PollenOdoSupportingShooter | B and C with odometry: same shared OpMode with odometry() on |
| PollenTeleOpTuning | PollenBot TeleOp (`BioBuzzStarterbotOpMode`) INIT menu: launcher target and drive-power scale, defaults from that OpMode's constants |

B and C use distinct static snapshots in the shared OpMode. Their INIT loads
never share settings. Start delay resets to the mode default. A remembered
snapshot that no longer fits the budget is kept as is: `budgetOk()` is false,
INIT shows OVER TIME BUDGET, `init()` skips building the sequence, and `start()`
refuses to run. While over budget, `adjust()` accepts only edits that do not
lengthen the route. START locks every control, including
Triangle (Circle and Square are not read). These are process-local snapshots, not persisted files.
Each snapshot also stores the `Route` (FULL_PARK, SUPPORT_PARK, EDITED), shown as the first INIT
line. `PollenAutoLog` writes INIT route changes, START settings, phases, warnings, the result and
0.25 s samples to RobotLog (tag `PollenAuto`); it is pure Java and never throws.
PollenBot TeleOp keeps its own static snapshot the same way; its defaults are the
OpMode's `LAUNCHER_TARGET_VELOCITY`, `PollenConfig.LAUNCHER_SPINUP_SECONDS`,
`PollenConfig.AGITATOR_FEED_POWER` and 100% drive power.

## Hardware contract

| Name | Device | Direction / control |
| --- | --- | --- |
| frontLeft | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |
| frontRight | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| backLeft | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| backRight | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |
| pollenLauncher | DcMotorEx | REVERSE, RUN_USING_ENCODER; PIDF 40/0/0/12.5 |
| intake | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| agitator | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |

A maps only the four drives; B/C map all seven. No windmill feeder is used.
Launcher targets are encoder ticks/s, not RPM. The existing hardware setup and
motor directions are unchanged. The shared shooter lifecycle is extracted from
the tested B implementation so the modes do not duplicate motor/fault handling.

Drive axes are positive forward, right, clockwise. Translation has zero turn.
Wheel order is front-left, front-right, back-left, back-right; mixer commands are
`f+s+t`, `f-s-t`, `f-s+t`, `f+s-t`, normalized together if magnitude exceeds one.

## Routes and timing

- A: delay -> timed forward -> hold.
- B default (Triangle): stationary shot -> leg 1 forward 10 cm off the
  wall -> leg 2 left 63.5 cm -> leg 3 172.47 cm at +17 deg (field-tuned) -> hold.
- Circle and Square do nothing. (Square loaded the three-leg route before pollen-odo-v5; Circle's
  leave-the-wall move, leg 3 only at 60.96 cm, was removed in pollen-odo-v6.)
- C: 12 s delay -> 3 s spin-up -> feed 15-22.5 s -> leg 1 forward flower
  clearance -> (leg 2 unused) -> leg 3 97.96 cm at -81.85 deg (field-tuned) -> hold.

A shoot step runs the launcher alone for `SPINUP` seconds (default
`PollenConfig.LAUNCHER_SPINUP_SECONDS` = 3), then launcher, intake and agitator
together for `FEED_SECONDS`, the intake at `FEED_POWER` (0.5) and the agitator at
`AGITATOR_POWER` (`PollenConfig.AGITATOR_FEED_POWER`, 1.0; it replaced a windmill
servo that ran at full power). Measured launcher speed is never used to delay,
pause, skip or stop feeding in auto; `PollenSequence.tick(now)` takes only the clock.
PollenBot TeleOp feeds on R1 when the launcher reads `LAUNCHER_READY_FRACTION`
(0.96) of target or its spin-up row elapses, whichever is first
(`launcherIsUpToSpeed` or `spinUpTimeIsOver` in `PollenTeleOpTuning`), and latches feeding until R1
is released: the intake gets +0.5 (`INTAKE_FEED_BOOST`, clipped to 1) and the
agitator its own feed power. The TeleOp reads the launcher's speed once per loop
(`launcherMeasuredVelocity`). Auto keeps fixed times so C never feeds before its
15 s slot.

Leg 3 defaults (`B_FIELD_LEG3_CM/DEGREES`, `C_LEG3_CM` in `PollenShootTuning`) are
field-tuned and no longer equal the CAD plan in `PollenFieldGeometry`; the route
checker reports the difference instead of requiring a match.

`LEG1_CM`/`LEG1_ANGLE` through `LEG3_CM`/`LEG3_ANGLE` are independent controller
rows; every leg uses `DRIVE_POWER` and the selected calibration profile, and a
0-cm leg is skipped. All three must be zero to disable all travel. Feed=0 removes
spin-up and feeding. If C's shooter is unavailable, a zero-output step occupies
its spin-up and feed slot. Explicit feed=0 instead departs after the chosen
start delay. Non-finite velocity stops the run.

Step clocks start on the first `tick()` that runs the step, not when the
previous step ends, so a delayed loop between steps cannot shorten or skip the
next one. If the previous tick commanded drive motion and the next tick arrives
more than `MAX_DRIVE_LOOP_GAP_SECONDS` (0.25 s) later, the sequence stops with
`LOOP STALL WHILE DRIVING - STOPPED`. Pauses while stationary are allowed. Real
elapsed time is therefore the plan plus about one loop per step change.

Budget includes all three drive legs and hold. Selected worst case must be <26 s;
the independent cutoff is 27 s. Runtime hardware exceptions attempt every stop
and request OpMode stop. Shooter INIT fallback requires all mapped shooter stop
commands to succeed; drive failure blocks. B/C never substitute A's starting-pose
route. Hardware output order stops mechanisms before any drive command.

## Calibration and geometry

A still commands seconds/power directly; its display uses the measured forward
fit, with explicitly unverified interpolation/extrapolation. B defaults to the
original 129 model at +25.87 degrees, 60.96 cm and power 0.20. The mat trial uses
separate measured direction/distance knots, not a direction-independent scalar.
See the calibration document for assumptions and exact measurements.

B's default is the complete three-leg field route. C's default start is flush and
square, centered between the flower and the midline (6.89 cm toward the midline
from the hive center); a hive-centered start overlaps the flower.
The field guide records the CAD geometry, tight initial margins and 180-degree
alliance transform. Neither these constants nor the timed routes use Pedro,
Pinpoint, AprilTags, voltage compensation or position/heading feedback.

## Tests and Android build

The test programs are in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen/`.
In Android Studio, run `PollenTests` there (green arrow; Windows or Mac). From the repository root
on the team's Mac, use the existing `ftcgradle` helper:

```bash
PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh     # the 7 programs, plus the field-route checker
ftcgradle :TeamCode:testDebugUnitTest    # the same 7 programs through JUnit (PollenTests)
ftcgradle :TeamCode:assembleDebug
```

The test programs use Java 8 compatibility and cover original A/B behavior,
mat arithmetic, field-preset selection, fixed spin-up/feed timing (including a launcher reading zero), C's slot, all three
movement legs, controller edits of each leg and launcher speed, budgets, memory and STOP,
plus the TeleOp INIT menu (defaults, limits, the two feed rules, memory, lock at START).
`PollenLoopTimingTest` drives B's full route and C through irregular loop
timing and 350 ms pauses at each step change, and checks that no leg is skipped
or shortened and that a pause while driving stops the run. The standard-library
Python script checks nominal swept polygons, including the rejected direct route
and rejected centered support start. The test script also exports the Java
presets (`PollenPresetExport`) and runs the checker with `--presets`, so the
checker fails if the Java routes and its plan disagree. Neither establishes
actual physical clearance.
`tools/draw-pollen-field-routes.py`, `tools/draw-pollen-pit-sheet.py` and
`tools/draw-pollen-field-layout.py` (matplotlib) redraw the route diagram, the
alliance pit sheet and the field layout; they draw, not check.

Every text file uses LF line endings (`.gitattributes`), including
`BioBuzzStarterbotOpMode.java`, which was CRLF before `pollen-odo-v7`.

`ftcgradle` is the team's existing shell helper; inspect `type ftcgradle` if it
is unavailable.

Deploy a freshly built APK through the team's working Wi-Fi/ADB connection or
Android Studio TeamCode run configuration to the Control Hub. USB-C has not
worked. Verify `pollen-odo-v9` in INIT. Check the actual installed commit and all
settings before physical tests. Java changes require rebuild/reinstall; INIT
settings and documentation edits do not.
