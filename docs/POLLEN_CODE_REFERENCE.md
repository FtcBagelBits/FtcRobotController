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
| **Odo ROBOT Shoot + Park** | Shoots the preloads, drives to the inner parking slot using odometry | Drive team, primary auto |
| **Odo Supporting Shooter Auto** | Waits 12 s for the partner, shoots, parks in the outer slot | Drive team, when the partner shoots first |
| **PollenBot** (TeleOp) | Driving, intake, R1 shoots, L1 reverses the launcher | Drive team |
| Odo ROBOT Park | Drive straight only (no shot), measured with odometry | Testing, or the launcher is broken |
| Angled Primary Shooter | **Experimental.** Odo ROBOT Shoot + Park with an angled shot: two legs to a spot off the wall and to the side, turn the launcher toward the hive, shoot, turn back, park (on hold since v11: the robot now shoots straight from the wall; it still runs, back to the hive like every auto) | Programmers, until it has been tried on a field |
| EMERGENCY Timed Shoot + Park / EMERGENCY Timed Supporting Shooter / EMERGENCY Timed Park | The same three jobs with timed driving (no odometry) | **Emergency only:** a match where the Pinpoint is not working |
| Odometry Check | Pinpoint wiring and settings test ([docs/ODOMETRY_BASELINE_TEST.md](ODOMETRY_BASELINE_TEST.md)) | Programmers |
| Pedro Test | Pedro pose on the Panels dashboard, plus a short test path (A also spins Lox's `flywheel` if the robot has one) | Programmers |
| LauncherTest | PollenBot's launcher, intake and agitator on their own, with the TeleOp's buttons, INIT menu and feed rule | Programmers |

`lox/` holds `MecanumOpMode`, `BioBuzzOpMode` ("Lox"), `BB2DecodeOpMode` and their helpers: older
robots' code (DECODE 2025-26). They share some hardware names but nothing else with PollenBot. Leave
them alone unless asked. `TeamCode/.../teamcode/readme.md` is the student-facing list of what's live.

Every Pollen and Odo mode shows a build id on the INIT screen (`PollenConfig.BUILD`, currently
`pollen-hive-v11` on this branch). **Bump it whenever behavior changes**, so the drive team can confirm which code is installed.

**Which way the robot faces (since `pollen-odo-v11`).** The launcher shoots out of the robot's back,
so every autonomous starts with the robot's front against the end wall and its back to the hive.
Routes, leg angles, menus, budgets, the field-route checker and the robot log stay in **route
directions** (as if the robot faced the hive: x forward toward the hive, y left). `pollen/PollenFacing`
is the only code that knows the robot is turned around: each autonomous OpMode passes the Pinpoint
pose through `PollenFacing.routePose` and its output through `PollenFacing.wheelPowers`. Keep it that
way: a new autonomous OpMode must use both, and nothing else should flip signs. TeleOp, LauncherTest,
Odometry Check and Pedro Test are unaffected.

## Hive modes on pollen-hive-ready

| Driver Hub name | Purpose |
| --- | --- |
| TEST Hive camera (NO MOTORS) | Camera detection and calibration |
| TEST Hive pipeline benchmark | Camera-only pipeline comparison with recorded results |
| TEST Hive primary shot ONLY / TEST Hive secondary shot ONLY | Isolated stationary shooting tests |
| TEST Hive garden + return | Isolated collection route and return |
| EXPERIMENTAL Hive Primary + Garden / EXPERIMENTAL Hive Secondary | Hive-aware autonomous modes; use the testing guide before running |

These use `PollenHiveAuto`, not `PollenSequence`. Feeding requires a settled shooting
pose, fresh hive readiness and measured launcher speed; there is **no timed speed
bypass**. After the first shot the primary mode can leave directly for the budgeted
garden route with shooting outputs off. There is **no flywheel coast/spin-down gate**.
The agitator meters the balls; retained speed readiness applies to feeding, not departure.
See the hive testing and diagnostics guides for execution modes and current controls.

## Hardware configuration names (PollenBot)

These must match the active configuration on the Control Hub. A typo compiles fine and fails on the robot.

| Name | Device | Notes |
| --- | --- | --- |
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | Drive motors | Names and directions in `PollenConfig.DRIVE_NAMES` / `DRIVE_REVERSED`. `pedroPathing/Constants` has its own copy; Odometry Check reports whether they match |
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
  is over, whichever is first. TeleOp, LauncherTest and established Odo/timed autonomous shots use it (hive modes differ as described above). The Supporting
  Shooter modes use only the timer (`Step.shootAtSetTime`), so they never feed before the time
  agreed with the partner. Keep `launch()` in the TeleOp and in LauncherTest the same. An odometry
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
  and distances were checked with Odometry Check after swapping the pod cables. The pod **offsets** are still Lox's
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

## Pollen drainer integration (pollen-hive-v11)

This build combines main's run recording and rehearsal markers with the drainer branch.
PollenBot now requires a servo named `pollenDrainer`. During a run, D-pad Up/Down nudges
its commanded position by 0.01 per press; A toggles between 0.55 (up) and 0.02 (down).
The initial command at START remains 0.55, as on the drainer branch. INIT tuning and
Circle/Square recording markers retain main's behavior.

The separate `PollenDrainerTest` OpMode is also included. It uses the same hardware name
and controls, with an up/initial position of 0.60 and a down position of 0.02. These
servo positions are preserved from the drainer branch; they have not been retuned.
