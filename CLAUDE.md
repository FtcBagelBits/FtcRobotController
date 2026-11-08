# CLAUDE.md

Guidance for anyone working in this repository: team members, mentors, and Claude Code (claude.ai/code).
Commands are for the team's Macs. On Windows or Linux use `gradlew.bat` / `./gradlew` and the `adb`
from your Android SDK's `platform-tools`.

## What this is

FTC team **31452** (FtcBagelBits) robot code for the 2026-27 **BIOBUZZ** season. It is a fork of
the *FIRST* Tech Challenge SDK (`FtcRobotController` **12.0**). The build produces one Android app
(`com.qualcomm.ftcrobotcontroller`) that runs on the REV Control Hub.

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`: all team code. Changes go here.
  - `pollen/`: this season's robot, **PollenBot**: autonomous modes, INIT menus, odometry driving.
  - `pedroPathing/Constants.java`: Pedro Pathing and Pinpoint settings.
- `FtcRobotController/`, `build.common.gradle`, `build.dependencies.gradle`: the upstream SDK.
  **Do not edit**; they are replaced wholesale on SDK updates. Team build changes go in `TeamCode/build.gradle`.
- `docs/`: start at [docs/README.md](docs/README.md). The drive team's printable guides are in
  [docs/drive-team/](docs/drive-team/README.md).
- `tools/`: computer tests, the field-route checker, and the scripts that draw the diagrams and print the guides.

## What runs on the robot

| Driver Hub name | What it is | Who uses it |
| --- | --- | --- |
| **Odo ROBOT Shoot + Park** | Shoots the preloads, drives to the inner parking slot using odometry | Drive team, primary auto |
| **Odo Supporting Shooter Auto** | Waits 12 s for the partner, shoots, parks in the outer slot | Drive team, when the partner shoots first |
| **PollenBot** (TeleOp) | Driving, intake, R1 shoots, L1 reverses the launcher | Drive team |
| Pollen ROBOT Shoot + Park / Pollen Supporting Shooter Auto | Same as the Odo ones but timed | Fallback if the Pinpoint fails |
| Odo ROBOT Park / Pollen ROBOT Park | Drive forward only (no shot) | Fallback, testing |
| Odometry Check | Pinpoint wiring and settings test ([docs/ODOMETRY_BASELINE_TEST.md](docs/ODOMETRY_BASELINE_TEST.md)) | Programmers |
| Pedro Test, LauncherTest, AprilTagTest, Sensor: GoBilda Pinpoint | Older diagnostics | Programmers |

`MecanumOpMode`, `BioBuzzOpMode` ("Lox") and `BB2DecodeOpMode` are older robots' code (DECODE
2025-26). They share some hardware names but nothing else with PollenBot. Leave them alone unless
asked. **Pedro Test** reads a `flywheel` motor that only Lox has, so on PollenBot it stops at INIT.

Every Pollen and Odo mode shows a build id on the INIT screen (`PollenConfig.BUILD`, e.g.
`pollen-odo-v6`). **Bump it whenever behavior changes**, so the drive team can confirm which code is installed.

## Hardware configuration names (PollenBot)

These must match the active configuration on the Control Hub. A typo compiles fine and fails on the robot.

| Name | Device | Notes |
| --- | --- | --- |
| `frontLeft`, `frontRight`, `backLeft`, `backRight` | Drive motors | Names and directions in `PollenConfig.DRIVE_NAMES` / `DRIVE_REVERSED`. `pedroPathing/Constants` has its own copy; Odometry Check reports whether they match |
| `pollenLauncher` | Launcher motor (`DcMotorEx`) | Velocity in ticks/s; 28 ticks per revolution (1180 ticks/s = 2529 RPM) |
| `intake`, `agitator` | Feed motors | |
| `odometry` | goBILDA Pinpoint, on I2C | Name set in `pedroPathing/Constants`. The forward-rolling pod goes in the **X** port |

Older robots also use `flywheel`, `coreHex`, `servo`, `leftDrive`, `rightDrive`, `leftIntakeServo`,
`rightIntakeServo` and `Webcam 1`.

## One-time setup (Mac)

Install Android Studio. The build needs the **android-36** platform (SDK Manager in Android Studio),
because `TeamCode/build.gradle` sets `compileSdk 36` for the Panels dashboard. Then add this to `~/.zshrc`:

```bash
# Java that ships with Android Studio, and adb from the Android SDK
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
export PATH="$JAVA_HOME/bin:$HOME/Library/Android/sdk/platform-tools:$PATH"
export HUB=192.168.43.1:5555   # the Control Hub, always this address on its own Wi-Fi

# Some team Macs already have an ftcgradle helper (check: type ftcgradle).
# If yours doesn't, this does the same job: run Gradle from the repo root.
type ftcgradle >/dev/null 2>&1 || ftcgradle() { (cd "$(git rev-parse --show-toplevel)" && ./gradlew "$@"); }
```

Open a new Terminal window afterward. Android Studio itself is optional after setup: everything below works from Terminal.

## Test, build, install

Run these from the repository root.

```bash
bash tools/test-pollen-sequence.sh       # computer tests: 7 test programs + the field-route checker (needs Java and python3)
ftcgradle :TeamCode:assembleDebug        # build only: fastest check that it compiles for Android
```

Install over the Control Hub's Wi-Fi:

```bash
# 1. Join the Control Hub's Wi-Fi network on the Mac. Turn off any VPN.
ping -c 3 192.168.43.1                   # must reply
adb connect $HUB                         # expect: connected to 192.168.43.1:5555
adb devices                              # expect: 192.168.43.1:5555   device
ftcgradle :TeamCode:installDebug         # builds and installs; ends with BUILD SUCCESSFUL
```

The Robot Controller app restarts on the hub. On the Driver Hub, open any Pollen mode's INIT and
check the build id. The team has never got USB-C to the hub working, so use Wi-Fi.

## After an event: pull the robot logs

The Pollen shoot autos write each run to the robot log (tag `PollenAuto`): the route chosen in INIT,
every setting at START, each phase with position and battery, warnings, and the final result. The
log rotates and old runs get overwritten, so **pull it after every event, before more testing**:

```bash
adb connect $HUB
mkdir -p ~/ftc/robot-logs/$(date +%Y-%m-%d) && cd ~/ftc/robot-logs/$(date +%Y-%m-%d)
for f in $(adb shell ls /sdcard/ | tr -d '\r' | grep -i robotControllerLog); do adb pull "/sdcard/$f"; done
adb pull /sdcard/FIRST/matchlogs         # per-run match logs, if the SDK wrote any
grep -h PollenAuto robotControllerLog*   # just our lines: START ... route=..., PHASE, WARNING, RESULT
```

Without adb: the Control Hub's Manage page (`http://192.168.43.1:8080`) has **Download Logs**.

## When the robot won't connect: start over from scratch

Do these in order and stop at the first check that fails. This is the sequence that has fixed every connection problem so far.

```bash
# 1. Clear every old adb, Gradle and emulator process. Quit Android Studio first.
adb kill-server
pkill -f "adb.*fork-server"
pkill -f "qemu-system" 2>/dev/null       # a stray Android emulator ("emulator-5554") confuses installs
ftcgradle --stop
ps aux | grep -i "[a]db"                 # expect: nothing

# 2. Network: Mac on the hub's Wi-Fi, VPN off
ipconfig getifaddr en0                   # expect 192.168.43.x
ping -c 3 192.168.43.1                   # expect replies
nc -vz 192.168.43.1 5555                 # expect "succeeded"

# 3. Fresh adb server from THIS Terminal, then connect
adb start-server
adb connect $HUB
adb devices                              # expect: 192.168.43.1:5555   device

# 4. Install
ftcgradle :TeamCode:installDebug
```

| What you see | Do this |
| --- | --- |
| No `192.168.43.x` address, or ping gets no reply | Rejoin the hub's Wi-Fi (Macs drift back to home Wi-Fi). Wait until the hub has finished booting (about 30 s). Turn off the VPN. A work laptop's security software can block `192.168.x.x` too. |
| `nc` refused | Power-cycle the Control Hub, rejoin its Wi-Fi, redo step 2. |
| `nc` succeeds but `adb connect` says "No route to host" | An old adb server is still running (often Android Studio's). Repeat step 1; `ps -o pid,ppid,command -p $(pgrep adb)` shows who started it. |
| Both `nc` and `curl http://192.168.43.1:8080` fail with "No route to host" while ping works | macOS Local Network permission: System Settings → Privacy & Security → Local Network → turn on Terminal, then quit and reopen Terminal. |
| `adb devices` shows `offline` or `unauthorized` | `adb disconnect`, power-cycle the hub, redo steps 2-3. |
| Gradle: "Failed to exec spawn helper" | Java changed under a running Gradle. `ftcgradle --stop`, retry once. If it still fails, install the app Gradle already built: `adb install -r TeamCode/build/outputs/apk/debug/TeamCode-debug.apk` |
| `INSTALL_FAILED_UPDATE_INCOMPATIBLE` | The app on the hub was installed from a different computer. Last resort, ask a mentor first: `adb uninstall com.qualcomm.ftcrobotcontroller`, then install again. Robot configurations are kept on the hub's storage, not in the app, but check the configuration afterward anyway. |

## Changing code

- Branch from `main`, open a PR to merge. Keep diffs small and readable: students maintain this
  code during competition. Keep existing commented-out code; it is often a hardware option toggled between matches.
- **Run `bash tools/test-pollen-sequence.sh` before every commit** that touches `pollen/`. The tests
  run on a computer with no SDK and no robot. They check sequencing, menus, time budgets and loop
  timing, and they run the odometry autos against a simulated robot. The field-route checker compares the
  Java presets with the field CAD. A pass is not a robot test: say which you did.
- Update the docs listed in [docs/README.md](docs/README.md) when behavior changes, including the
  drive-team guides; then rebuild their PDFs: `python3 tools/build-drive-team-pdfs.py`.
- **`BioBuzzStarterbotOpMode.java` has Windows (CRLF) line endings.** Keep them: edit it in an
  editor that preserves them, and apply patches that touch it with `git am --keep-cr` (plain
  `git am` strips the endings and the patch fails).
- Patches shared outside git (for example from a Claude chat that can only read the repo): name
  them `description-YYYYMMDD-HHMM.patch` (Pacific time) so downloads don't collide; apply with
  `git am`; push from a team Mac. Don't add `Co-Authored-By` trailers to commit messages.
- Launcher speeds, PIDF gains and drive calibrations are measured on the robot. Treat them as
  data: if a change alters one, say so explicitly.

## How the Pollen autos work (short version)

- **INIT edits, START freezes.** Every auto shows a menu (D-pad selects and edits rows; Triangle
  restores the defaults). START locks it. Settings are remembered until the app restarts; the
  start delay resets every INIT, and B's Circle fallback is never carried into the next INIT.
  The first INIT line, **ROUTE**, says which route will run. Details: [docs/POLLEN_AUTONOMOUS.md](docs/POLLEN_AUTONOMOUS.md).
- **B's buttons since `pollen-odo-v5`:** the default and Triangle give the full shoot-and-park route.
  Circle loads the short leave-the-wall move. Square does nothing (it used to load the full route).
- `PollenSequence` runs a list of steps without blocking (timed drives, shots, odometry moves),
  with a time budget checked at INIT and a hard stop at 27 s. An odometry leg that runs out of
  time more than 2 in from its end, or stops making progress, stops the whole route: it never
  starts the next leg from an unplanned spot.
- `PollenAutoLog` writes each shoot-auto run to RobotLog (see "After an event" above). It is pure
  Java with computer tests, and it can never throw: logging must not stop the robot.
- **Odo modes** drive each leg to a measured end point (`PollenOdometryDrive`). They square
  against the start wall first, then zero the odometry. Leg cm/angle are real distances. Timed
  modes turn the same numbers into drive times. See [docs/POLLEN_ODOMETRY_AUTOS.md](docs/POLLEN_ODOMETRY_AUTOS.md).

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

## Older robots

- `MecanumOpMode` owns `MecanumDriveTrain`, `Launcher`, `Camera` and `Auto` (time-based).
  `BioBuzzOpMode` (Lox) pairs with `MecanumDrive`. `BB2DecodeOpMode` is a monolith for a two-motor tank robot.
- Helpers take a concrete OpMode type (`MecanumDriveTrain(MecanumOpMode)`), so they don't move between robots.
  Motor directions differ between `MecanumDrive` and `MecanumDriveTrain` because the robots are
  wired differently. Don't "harmonize" them.
- `Launcher` and `BB2DecodeOpMode` hold tuned shot constants and PIDF gains (`P = 265`, `F = 14.098`).
  They are data from `FlywheelTuner`. `FlywheelTuner.java` sits in `teamcode/` but declares package
  `...teamcode.tutorials`: move the file rather than "fix" the package.
- AprilTag vision: `AprilTagProcessor` + `VisionPortal` on `Webcam 1`; `Camera` wraps it; tag 20 is the blue goal.
