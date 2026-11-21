# Pollen code reference

Shared working instructions: [AGENTS.md](../AGENTS.md). Commands and desktop setup:
[Desktop tools](DESKTOP_TOOLS.md). This reference describes the established Odo/TeleOp code;
for the hive modes on `pollen-hive-ready`, also read [hive testing](POLLEN_HIVE_TESTING.md),
[benchmarking](POLLEN_HIVE_BENCHMARK.md) and [diagnostics](POLLEN_HIVE_DIAGNOSTICS.md).

## What this is

FTC team **31452** (FtcBagelBits) robot code for the 2026-27 **BIOBUZZ** season. It is a fork of
the *FIRST* Tech Challenge SDK (`FtcRobotController` **12.0**). The build produces one Android app
(`com.qualcomm.ftcrobotcontroller`) that runs on the REV Control Hub.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: all team code. Changes go here.
  - `pollen/`: this season's robot, **PollenBot**: autonomous modes, INIT menus, odometry driving.
  - `pedroPathing/Constants.java`: Pedro Pathing and Pinpoint settings.
  - `lox/`: last season's robot (Lox, DECODE 2025-26). Kept so it still runs; leave it alone.
- `TeamCode/src/test/java/.../teamcode/pollen/`: computer tests for `pollen/` (`PollenTests` runs them all).
- `FtcRobotController/`, `build.common.gradle`, `build.dependencies.gradle`: the upstream SDK.
  **Do not edit**; they are replaced wholesale on SDK updates. Team build changes go in `TeamCode/build.gradle`.
- `docs/`: start at [docs/README.md](README.md). The drive team's printable guides are in
  [docs/drive-team/](drive-team/README.md).
- `tools/`: desktop commands (`ftc.sh`), tests, downloads and analysis, and diagram/guide generators.

## Established Odo and TeleOp modes

| Driver Hub name | What it is | Who uses it |
| --- | --- | --- |
| **Emergency Odo Shoot + Park** | Shoots the preloads, drives to the inner parking slot using odometry | Drive team, primary fallback |
| **Emergency Odo Supporting Shooter** | Waits 12 s for the partner, shoots, parks in the outer slot | Drive team, supporting fallback |
| **PollenBot** (TeleOp) | Driving, intake, R1 shoots, L1 reverses the launcher | Drive team |
| Emergency Odo Park | Drive straight only (no shot), measured with odometry | Testing, or the launcher is broken |
| Calibration Angled Primary | **Experimental.** Emergency Odo Shoot + Park with an angled shot: two legs to a spot off the wall and to the side, turn the launcher toward the hive, shoot, turn back, park (on hold since v11: the robot now shoots straight from the wall; it still runs, back to the hive like every auto) | Programmers, until it has been tried on a field |
| Emergency Timed Shoot + Park / Emergency Timed Supporting Shooter / Emergency Timed Park | The same three jobs with timed driving (no odometry) | **Emergency only:** a match where the Pinpoint is not working |
| Calibration Odometry | Pinpoint wiring and settings test ([docs/ODOMETRY_BASELINE_TEST.md](ODOMETRY_BASELINE_TEST.md)) | Programmers |
| Calibration Pedro Pathing | Pedro pose on the Panels dashboard, plus a short test path (A also spins Lox's `flywheel` if the robot has one) | Programmers |
| Calibration Launcher | PollenBot's launcher, intake and agitator on their own, with the TeleOp's buttons, INIT menu and feed rule | Programmers |

`lox/` holds `MecanumOpMode`, `BioBuzzOpMode` ("Lox"), `BB2DecodeOpMode` and their helpers: older
robots' code (DECODE 2025-26). They share some hardware names but nothing else with PollenBot. Leave
them alone unless asked. `TeamCode/.../teamcode/readme.md` is the student-facing list of what's live.

Every Pollen and Odo mode shows a build id on the INIT screen (`PollenConfig.BUILD`, currently
`pollen-hive-v34` on this branch). **Bump it whenever behavior changes**, so the drive team can confirm which code is installed.

**Which way the robot faces (since `pollen-odo-v11`).** The launcher shoots out of the robot's back,
so every autonomous starts with the robot's front against the end wall and its back to the hive.
Routes, leg angles, menus, budgets, the field-route checker and the robot log stay in **route
directions** (as if the robot faced the hive: x forward toward the hive, y left). `pollen/PollenFacing`
is the only code that knows the robot is turned around: each autonomous OpMode passes the Pinpoint
pose through `PollenFacing.routePose` and its output through `PollenFacing.wheelPowers`. Keep it that
way: a new autonomous OpMode must use both, and nothing else should flip signs. TeleOp, Calibration Launcher,
Calibration Odometry and Calibration Pedro Pathing are unaffected.

## Hive modes on pollen-hive-ready

| Driver Hub name | Purpose |
| --- | --- |
| Calibration Hive Camera (No Motors) | Camera detection and calibration |
| Calibration Hive Pipeline Benchmark | Camera-only pipeline comparison with recorded results |
| Calibration Primary Shot Only / Calibration Secondary Shot Only | Isolated stationary shooting tests |
| Calibration Garden + Return | Isolated collection route and return |
| Primary Shooter + Garden / Secondary Shooter + Flower | Hive-aware autonomous modes; use the testing guide before running |
| Beta Primary Shooter - Hero | Primary garden route with independent INIT settings |
| Beta Primary Shooter - Angled Left | Primary garden route from an angled shot spot, own settings file `primary-angled`, rows `shotLeftIn`/`shotAwayIn`, aim from `PollenFieldGeometry.aimDegrees` (`PollenHiveRoute.angled`) |
| Calibration Hive Aim | PollenBot TeleOp started at the primary start; Circle aims by camera (D-pad left) or odometry (D-pad right); AIM rows compare both (`PollenAimCalibration`, `PollenHiveAim`). Its screen also shows the **Camera sideways correction** that would center the cell: with the robot carefully centered at the primary start, put that number in the camera row (default 0 since v38; 2.62 before the camera was straightened) |
| Beta Primary Shooter - Fast | Primary garden route, own settings file `primary-fast`, closer garden entry, 2.5 s feeds, one return nudge (`PollenHiveRoute.fast`) |
| Beta Secondary Shooter - Out of the Way | Shoot, wait in the scoring-end/side-wall corner (phases ROOM, WAIT), then park with the configured margin |

These use `PollenHiveAuto`. A volley starts from a settled shooting pose once measured launcher
speed is ready. Both return shots and the secondary preload require confirmed Hive UP for more
than one second. The main beta autos default to three seconds of continuous feed through subsequent
camera changes and speed dips, subject to the parking deadline and 30-second cutoff. The beta primary collects from the garden
and the main beta secondary drains the nearby flower before returning to the shooting pose and
checking UP again. They pre-spin the flywheel on the return, with the intake and agitator off.
See the hive testing and diagnostics guides for execution modes and current controls.

## Hardware configuration names (PollenBot)

These must match the active configuration on the Control Hub. A typo compiles fine and fails on the robot.

| Name | Device | Notes |
| --- | --- | --- |
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | Drive motors | Names and directions in `PollenConfig.DRIVE_NAMES` / `DRIVE_REVERSED`. `pedroPathing/Constants` has its own copy; Calibration Odometry reports whether they match |
| `pollenLauncher` | Launcher motor (`DcMotorEx`) | Velocity in ticks/s; 28 ticks per revolution (1180 ticks/s = 2529 RPM) |
| `intake`, `agitator` | Feed motors | |
| `limelight` | Limelight 3A camera | Used by the hive modes; configured in `PollenHiveCamera` and the benchmark |
| `odometry` | goBILDA Pinpoint, on I2C | Name set in `pedroPathing/Constants`. The forward-rolling pod goes in the **X** port |

Older robots also use `flywheel`, `coreHex`, `servo`, `leftDrive`, `rightDrive`, `leftIntakeServo`,
`rightIntakeServo` and `Webcam 1`.

## How the established Odo/timed autos work (short version)

- **INIT edits, START freezes.** Every auto shows a menu (D-pad selects and edits rows; Triangle
  restores the defaults). START locks it. Settings are remembered until the app restarts; the
  start delay resets every INIT.
  The first INIT line, **ROUTE**, says which route will run. Details: [docs/POLLEN_AUTONOMOUS.md](POLLEN_AUTONOMOUS.md).
- **B's buttons:** the default and Triangle give the full shoot-and-park route. Circle and Square do
  nothing (Square loaded the full route before `pollen-odo-v5`; Circle's leave-the-wall move was removed in v6).
- `PollenSequence` runs a list of steps without blocking, with a time budget checked at INIT and
  a hard stop at 27 s. There are five kinds of step (`Kind`: DRIVE, SHOOT, MOVE, TURN,
  ZERO_ODOMETRY) and one short method per kind; a new kind of step gets its own method the same way.
- One feed rule, in `PollenFeedRule`: feed when the launcher reads 96% of target or the spin-up time
  is over, whichever is first. TeleOp, Calibration Launcher and established Odo/timed autonomous shots use it (hive modes differ as described above). The Supporting
  Shooter modes use only the timer (`Step.shootAtSetTime`), so they never feed before the time
  agreed with the partner. Keep `launch()` in the TeleOp and in Calibration Launcher the same. An odometry
  leg that runs out of time more than 2 in from its end, or stops making progress, stops the
  whole route: it never starts the next leg from an unplanned spot.
- `PollenAutoLog` writes each shoot-auto run to RobotLog (see [Desktop tools](DESKTOP_TOOLS.md)). It is pure
  Java with computer tests, and it can never throw: logging must not stop the robot.
- **Odo modes** drive each leg to a measured end point (`PollenOdometryDrive`). They square
  against the start wall first, then zero the odometry. Leg cm/angle are real distances. Timed
  modes turn the same numbers into drive times. See [docs/POLLEN_ODOMETRY_AUTOS.md](POLLEN_ODOMETRY_AUTOS.md).

## Pedro Pathing and odometry: current status

- `com.pedropathing:revhub:3.0.2` and `com.pedropathing:tuning:1.0.3`, declared in
  `TeamCode/build.gradle`, from Maven Central. AutoTune also pulls `dev.frozenmilk.sinister:Sloth`
  from `repo.dairy.foundation`. Pedro 3 is a rewrite: Pedro 2 code found online
  (`FollowerBuilder`, `com.pedropathing.geometry.Pose`, ...) has to be translated, not copied.
- **The Odo autos use only Pedro's `PinpointLocalizer`** (the pose). They drive with the team's own
  simple controller. **Pedro's path follower is not used yet**: the Foresight constants in
  `Constants.java` are placeholders from Pedro's docs, measured on someone else's robot.
- **AutoTune is not set up.** Its web UI (`http://192.168.43.1:10158`) needs `pedroPathing/Tuning.java` and
  `pedroPathing/procedures/`, which are **not in this repo**. To tune, copy them from the
  [Pedro Quickstart](https://github.com/Pedro-Pathing/Quickstart) (versions above), changing only the `package` line.
- Pinpoint settings (`Constants.localizerConfig`) were copied from Lox. On PollenBot, pod directions
  and distances were checked with Calibration Odometry after swapping the pod cables. The pod **offsets** are still Lox's
  (156, -81 mm). A turn test on vinyl suggested about (167.8, -67.6); remeasure on field tiles before changing them.
- **Panels** dashboard (`http://192.168.43.1:8001`, `com.bylazar:fullpanels` from bylazar's Maven
  repo) is why `compileSdk` is 36 and the app is ~76 MB. It draws Lox mirrored. That is a drawing
  fix in `PedroLocalizationTest.REVERSE_XY`; never fix it by changing encoder or motor directions.

## Older robots (`lox/`)

- `MecanumOpMode` owns `MecanumDriveTrain`, `Launcher`, `Camera` and `Auto` (time-based).
  `BioBuzzOpMode` (Lox) pairs with `MecanumDrive`. `BB2DecodeOpMode` is a monolith for a two-motor tank robot.
- Helpers take a concrete OpMode type (`MecanumDriveTrain(MecanumOpMode)`), so they don't move between robots.
  Motor directions differ between `MecanumDrive` and `MecanumDriveTrain` because the robots are
  wired differently. Don't "harmonize" them.
- `Launcher` and `BB2DecodeOpMode` hold tuned shot constants and PIDF gains (`P = 265`, `F = 14.098`).
  They are data from a flywheel tuner (not in this repo).
- AprilTag vision: `AprilTagProcessor` + `VisionPortal` on `Webcam 1`; `Camera` wraps it; tag 20 is the blue goal.

## Pollen drainer integration (pollen-hive-v14)

This build combines main's run recording and rehearsal markers with the drainer branch.
PollenBot now requires a servo named `pollenDrainer`. During a run, D-pad Up/Down nudges
its commanded position by the selected step per press; A toggles between the selected up and down positions.
Lowering the drainer commands full inward intake. Small position adjustments keep the last
state until a nudge reaches or passes the opposite preset. The driver strafes with the left stick.
Raising the drainer (Cross, or Square since v44) returns intake control to the triggers and enables L1 launcher reversal.
Manual inward agitation follows the trigger up to 0.50. R1 shooting uses the configured feed power.

The INIT menu adds Drainer up position (default 0.45), Drainer down position (0.02),
and Drainer step (0.01). D-pad Up/Down selects a row; Left/Right edits up/down by 0.01
and step by 0.002 per tap. Positions stay within 0–1; step stays within 0.002–1.
Triangle restores defaults, settings are remembered until app restart, and START locks
settings and commands the selected up position. The servo does not move during INIT. INIT tuning and
Since v39 PollenBot has no rehearsal markers: L1 is the unjam button (Square before v39); Square raises the drainer (v44); Triangle does nothing after START.

The separate `Calibration Drainer Servo` OpMode moves only the servo for calibration. It uses the same hardware name
and controls, with the same three INIT settings, defaults and increments as PollenBot. Its INIT
menu contains only the drainer rows. Settings are remembered separately for each OpMode
until the app restarts. The test's up/initial default is now 0.45 (previously 0.60).

Flower collection practice and the main beta secondary in LIVE require `pollenDrainer`. They lower
it during the flower process with full inward intake and raise it at the end, on fault, or at STOP.
The remaining match autos hold the drainer UP (0.45) when it is mapped.
See [collection practice](POLLEN_COLLECTION_PRACTICE.md) for both isolated pickup routines.
