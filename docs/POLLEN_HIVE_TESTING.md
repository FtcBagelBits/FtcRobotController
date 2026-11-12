# Limelight hive readiness and garden autonomous

Build **pollen-hive-v1**. Experimental patch based on main **b5f578ce2c7574de0aa15e7d9c9a56e8649565b3**.
The existing match autos keep their behavior. Select the new entries in **Pollen Hive** explicitly.

## What is implemented

| Driver Hub entry | Purpose |
| --- | --- |
| TEST Hive camera (NO MOTORS) | Camera connection, capture/calibration, saved profiles, one-second readiness telemetry. Never maps a motor. |
| EXPERIMENTAL Hive Secondary | Wait for the near cell to be UP, feed only while it stays ready, then depart. No fixed 12-second assumption. |
| EXPERIMENTAL Hive Primary + Garden | First gated volley, stop flywheel, collect with front intake, return to the starting shooting pose, wait/shoot again, depart. |

Both experimental autos default to **DRY** on every INIT, including after saved settings load. Their execution row has three values:

- **0 DRY:** motors stay zero; the controller uses a virtual flywheel speed to demonstrate feed requests. Real camera observations and real odometry still apply. This is a stationary first-shot check, not a whole-route simulation: once it requests driving, an unmoved robot will hit the no-progress stop.
- **1 DRIVE:** real measured driving and turns; launcher, intake and agitator remain off. Shots are skipped; primary exercises garden/return/park. Camera/profile not required. This is the first on-field route test.
- **2 LIVE:** real drive and mechanisms, with calibrated hive gating and measured launcher speed. Select deliberately after the first two tests.

`IsHiveReadyForShot` classifies UP, DOWN or UNKNOWN from a selected near-cell tag. Its boolean returns true only for confirmed UP. Invalid data, missing selected tag, stale/frozen frames, outside-band observations and no profile all return false. Production also requires the robot at the shooting pose, settled, and launcher at measured speed. The new modes do **not** use the old launcher timer override. Mount height is approximately **14 inches to the lens center**; calibrate with the actual fixed height, pitch and orientation.

## Apply and build on Lev's Mac

Use the kit's `apply-on-new-branch.sh` from inside the existing repository. It requires a clean checkout, fetches origin/main, checks the exact base SHA, creates a new branch, then applies the supplied mail patch with `git am`. It never pushes, resets, changes main, or edits authentication settings. If main has advanced, stop and rebase/regenerate the patch against that new main rather than forcing it.

```bash
# Inside the existing FtcRobotController checkout:
bash "$HOME/Downloads/Pollen_Hive_Kit/apply-on-new-branch.sh"
git status --short --branch

# Pure Java simulator and existing field-route checks:
bash tools/test-pollen-sequence.sh

# Actual Android SDK build and all JUnit tests, including JSON contract tests:
ftcgradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
```

The team's `ftcgradle` wrapper supplies Android Studio's JBR. If it isn't defined on Lev's Mac, use Android Studio's **Gradle JDK = Embedded JDK/JBR**, or run Gradle with that JDK as JAVA_HOME. Do not change the work git identity to apply this patch. See [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md) for existing install/Wi-Fi steps. There is no ready-to-install APK in this kit.

## Before practice: camera and Mac only

1. Run the computer tests above. Open `PollenHiveTest.java` and `PollenHiveCameraTest.java` in Android Studio to see the failure cases and simulated sequences. They need no connected robot.
2. On the camera, retain/export the working AprilTag pipeline. Use **36h11**, actual black-square tag size **82.55 mm / 3.25 in**, 3-D pose output, and a live camera image source. Record pipeline, exposure, resolution, firmware and downscale; keep them fixed through calibration.
3. Put the lens center 14 inches above the floor and use the existing printed official tags. Check visibility above the camera at realistic viewing angles. Paper results are transport/visibility tests, not real-hive calibration.
4. The new read-only recorder works with standard Python 3. It does not install anything on the camera or alter its pipeline. Set the pipeline in the camera UI first. Substitute the camera hostname/IP and selected printed tag:

```bash
python3 tools/hive-camera-record.py --host limelight.local --pipeline 0 --tag 34 \
  --label paper-up --seconds 10 --out "$HOME/Desktop/hive-paper-up.csv"
```

It writes CSV and raw `.ndjson`. Repeat with a different filename for covered tag, wrong sheet, out of view and tipped paper. If `.local` doesn't resolve, use the IP shown by the existing camera setup. `/results` uses port 5807. A connection failure, missing pose or frozen camera frame is reported. The recorder never declares the hive ready; the Java classifier is exercised by the computer tests and robot diagnostic.

## Connect to the robot

1. Power off, secure the rear camera and strain-relieve a USB data cable. Lens center approximately 14 inches above tiles. The robot's **front intake faces the end wall**, its **back launcher/camera faces the hive** at the shooting position.
2. Camera USB-C to Control Hub's **blue USB-A 3.0** port; USB supplies power. This is the Control Hub, not Driver Hub.
3. Power on, scan/edit the existing robot configuration, name the Limelight Ethernet device **`limelight`**, save and activate. Preserve all motor and Pinpoint names.
4. Build/install using the normal team workflow. Confirm build `pollen-hive-v1`. The autos retain names `frontLeft`, `frontRight`, `backLeft`, `backRight`, `pollenLauncher`, `intake`, `agitator`, `odometry`.
5. Select **TEST Hive camera (NO MOTORS)**. Prove a printed selected tag is received before debugging the real hive view.

## Real-hive calibration and independent validation

Use one profile slot per alliance/start spot/camera setup. Suggested slot labels: 1 red primary, 2 red secondary, 3 blue primary, 4 blue secondary. These are human labels, not an automatic alliance detector. A saved profile contains its pipeline, selected tag and camera-space envelopes. Changing a mount, tag, tag size, camera pipeline settings or shooting position requires calibration again.

1. Position the robot precisely at its actual shooting pose and leave it still. Confirm the selected ID is on the **near moving cell**, not the far cell or another hive. The diagnostic defaults to tag 34 as a candidate for red primary; verify the actual visible ID. Secondary/blue IDs must be selected from the real cell, not guessed.
2. During INIT: D-pad up/down selects **slot / pipeline / tag**; left/right changes it. START opens the camera. It still commands no motors.
3. **Triangle clears the candidate** in memory (does not erase disk). This also removes a previously loaded DOWN box if you are changing to UP-only calibration.
4. Hold the near cell physically UP and press **Square** once. Keep it stationary for five seconds. Capture needs at least 40 distinct valid frames and rejects noisy/moving samples. It saves raw capture samples on the Hub even when the fit is rejected.
5. If the selected tag is visible DOWN, tip it, settle it, then press **Circle** for a five-second DOWN capture. If DOWN is hidden, keep an **UP-only** profile; DOWN/transition/no tag will report UNKNOWN, never UP. Do not label disappearance as proof of UP.
6. Press **Cross** to save the candidate. It refuses overlapping UP/DOWN boxes or a missing UP box. Every axis is bounded; margins initially add 25 mm for x/y and 40 mm for depth around the sample extrema. These are experimental margins, not established field accuracy.
7. Now validate without pressing capture buttons: alternate the physical hive 20 times, cover the tag, show only a different tag, unplug/reconnect camera, rock the hive through intermediate angles, and slightly vary robot placement within the expected return error. Require **zero false UP** in the tested non-shootable conditions and reliable UP at the settled shooting pose. Confirm both immediate loss of permission on a contradicting frame and reconfirmation after recovery.
8. STOP, INIT again with the same slot/pipeline/tag, START: confirm the saved profile reloads and gives the same answers. Repeat for the secondary shooting pose.

The helper runs every loop; telemetry refreshes once per second. It needs at least four distinct agreeing frames spanning 0.15 s to grant UP/DOWN. Its permission expires 0.20 s after the last distinct accepted observation. The reader separately rejects results older than 150 ms on the Hub or over 200 ms including reported image latency. These are bounds on software observations, not a guarantee that the basket stays shootable until a launched ball arrives.

Files live in the Hub's FIRST/pollen-hive directory (normally `/sdcard/FIRST/pollen-hive`). Profiles/settings are `.properties`; captures include a `samplesCsv` field. Back them up after practice:

```bash
adb pull /sdcard/FIRST/pollen-hive "$HOME/Desktop/pollen-hive-backup"
adb pull /sdcard/robotControllerLog.txt "$HOME/Desktop/RobotControllerLog-hive.txt"
```

Log filename/capitalization can vary; use the team's documented [robot log retrieval](POLLEN_AUTONOMOUS.md#robot-log) if that path is absent. Search log lines for `PollenHive`. State changes, feed requests, autonomous phases and errors are logged. Capture files also record tag/pipeline and camera-space metres. Photograph the camera settings/mount; a pipeline number alone cannot detect later edits to that pipeline.

## Default garden route, derived from the saved repo layout

The source of field measurements is [POLLEN_FIELD_LAYOUT.md](POLLEN_FIELD_LAYOUT.md), especially Zones and Game pieces. No field measurements were re-derived for this patch. Nominal red primary start is `(X,Z)=(-12.74164098,61.585)` inches. Garden ball centers are approximately `(-69.31,69.3), (-66.42,69.3), (-63.53,69.3), (-60.64,69.3)`. Blue uses the 180-degree rotated setup, with identical relative route settings.

![Default garden approach and return](POLLEN_HIVE_GARDEN.svg)

Route forward = toward hive from START; route left = left while looking at hive. **All menu distances are centimeters, absolute robot-center coordinates from START.** Heading 0 is the starting chassis orientation (front toward wall, rear toward hive). Heading -90 degrees turns the front toward the garden on the left. This distinction matters: translation angle and chassis heading are separate.

| Point/action | Field X,Z (in, red) | Route forward,left (cm) | Heading |
| --- | --- | --- | --- |
| S: shooting start | -12.742, 61.585 | 0, 0 | 0 |
| A: clear end wall | -12.742, 55.585 | 15.24, 0 | 0 |
| B: first turning point | -30, 55.585 | 15.24, 43.836 | turn to -90 |
| C: garden turning point | -55.585, 55.585 | 15.24, 108.822 | arrive -90; turn to 0 |
| D: intake alignment | -60.585, 55.585 | 15.24, 121.522 | 0 |
| E: collect toward wall | -60.585, 60.585 | 2.54, 121.522 | 0 |
| Return | E → D → C → A → S | same points in reverse corridor | 0 |

B is shifted away from the alliance midline before the first turn. C leaves 15 inches to both walls, more than the 12.73-inch circumradius of an 18-inch square. No turn happens at E. D/E leave the body's left edge 1 inch from the side wall; E leaves its front edge 1 inch from the end wall. The intake runs only for D→E and the collection hold, after flywheel speed has fallen to at most 100 ticks/s; agitator stays off. The robot backs out before crossing back to the shooting position. It does not reset odometry during the cycle.

**Intake reach is the remaining mechanical assumption.** At E, the leftmost ball center is 8.725 inches left of the robot centerline. A centered mouth would need about **17.45 inches of usable width** to cover all four centers at once, plus appropriate reach into the ball row. A narrower intake can collect fewer balls on this route; the outside balls may need another approach or funnel. The code cannot confirm ball count. Measure the actual intake envelope/reach and test with balls before claiming a four-ball pickup. The footprint checker assumes the entire robot, including intake, fits within 18×18 inches.

The INIT geometry checks sample translations at at most 0.1-inch intervals and turns at one-degree intervals against the repo's fixed obstacle envelopes, walls and alliance midline. They are nominal checks, not avoidance of partners, scattered pieces, build tolerances or odometry errors. The real collection clearances are tight. Start with DRIVE and stop if the path differs from the drawing.

## INIT controls and what to tune

D-pad up/down selects a row; left/right edits; tap/release each change. **Square saves** settings independently for primary and secondary; **Triangle restores CAD defaults**. START freezes settings. Execution mode always resets to DRY on the next INIT. The profile line shows the saved tag/pipeline before START; DRY/LIVE START refuses a missing/invalid profile. DRIVE doesn't require one.

| Rows | Defaults / purpose |
| --- | --- |
| Profile slot | Primary 1, secondary 2; choose the validated alliance/position slot |
| Execution | 0 DRY, 1 DRIVE, 2 LIVE |
| Start delay | 0 seconds; supporting mode waits for actual UP, not a guessed partner delay |
| First / second feed | 1.5 / 1.5 seconds of actual commanded feed; pauses don't spend feed duration |
| First-shot window | Primary waits at most 5 seconds in FIRST; if it has fed, proceeds to collection; otherwise departs |
| Launcher | 1180 ticks/s; PIDF unchanged |
| Travel / turn / pickup drive power | 0.60 / 0.40 / 0.20 |
| Collection intake / hold | 0.50 power / 0.50 s; hold set to zero leaves one control-tick transition |
| Shooting intake / agitator | 0.50 / 1.00 |
| Clearance forward / first turn left / garden turn left | 15.24 / 43.836 / 108.822 cm |
| Pickup left / forward | 121.522 / 2.54 cm |
| Transit / pickup heading | -90 / 0 degrees; final shooting heading always returns to zero |
| Park | 1 enabled; 0 stops after the shooting sequence, useful for isolated tests |
| Park1/2/3 forward and left | Editable absolute waypoints; CAD-derived existing departure corridor and parking spot, not the timed routine's shortened field calibration |

Distance increments are 1 cm, heading 5 degrees, powers 0.05, launcher 20 ticks/s, times 0.5 seconds. Invalid geometry blocks START/save; reduce errors rather than bypassing the checker. Primary garden is about **11.22 s** and primary park **4.76 s** by the existing drive/turn estimates, including transition allowance. These are planning estimates only. No default is presented as physically tuned.

## Timing and autonomous behavior

- Primary first volley ends when actual feed duration is used, confirmed DOWN follows some feeding, or its first-shot window ends. A missing/ambiguous observation immediately stops feeding but does not invent a tip.
- Before garden collection, launcher is commanded off. If it doesn't slow below 100 ticks/s within two seconds, collection is skipped and the robot departs. This avoids running the shared intake toward a spinning launcher during pickup.
- Garden admission reserves its estimated duration, three seconds for second spin-up, selected second feed duration, estimated parking duration and margins. If time is insufficient, it skips collection and parks. Slow launcher spin-up or a delayed first shot can trigger this; telemetry states why. Raising drive power is not a substitute for measuring the cycle.
- After return, position must be within 0.5 inch of START, heading within 2 degrees, and speed/turn rate settled for 0.3 seconds. Old readiness is discarded while traveling; fresh observations reconfirm UP.
- If the near cell is DOWN or UNKNOWN after return, primary waits. If it becomes UP in time, it shoots. Secondary behaves similarly for its first volley. Both reserve departure time and stop all commands at **27 seconds**.
- A 0.25-second control-loop gap, invalid odometry, lost Pinpoint READY status, stuck drive/turn or route failure latches a stop. A motion fault does not shortcut across the field to park. There is no guarantee of finishing parking if the robot runs slower than the estimates; the hard deadline still wins.
- Permission is checked every loop during feeding. Already released balls continue flying after feeding stops. Detection is not predictive hive-motion control.

## Practice sequence and acceptance record

1. **No motors:** printed tag connection, then real UP/DOWN calibration and repeated validation for both shooting positions.
2. **DRY:** choose each experimental auto, correct profile, park disabled if isolating the shot; tip/cover the hive and watch `Feed requested`. Virtual speed is clearly labeled. The real launcher never runs.
3. **DRIVE:** no balls loaded, clear lane, test primary garden turns/return/park with shooter and intake off. Verify front-first travel to C, front-to-wall pickup D→E, rear-to-hive at S. Confirm the 14-inch mount stays clear. Then test secondary departure. Check both alliances' placement separately.
4. **LIVE pickup/shooting:** use small batches and a teammate on STOP. Confirm speed sign is positive at the desired launcher direction. Observe actual pickup width/reach and whether stopped-flywheel intake collection retains balls. Verify the complete return shot and camera reliability after movement/vibration.
5. Repeat with a tipped return, covered tag, wrong tag, and camera disconnect. Keep the detector fail-closed; do not replace false/UNKNOWN with timer-based shooting.
6. Record installed commit, profile slot/tag, mount dimensions, pipeline settings, INIT settings, cycle time, pickup count, false-ready count and return pose error. Save settings, photos, capture files and robot log. Keep legacy autos available if any acceptance item fails.

## Code map and validation performed for this patch

- `IsHiveReadyForShot`: pure Java profile, capture, classification and freshness.
- `PollenHiveCamera`: strict SDK JSON adapter; rejects missing/default-zero poses, wrong pipeline/tag, duplicate IDs and excessive age/latency.
- `PollenHiveFiles`, `PollenHiveCheck`: persisted calibration/settings and no-motors diagnostic.
- `PollenHiveRoute`, `PollenHiveMotion`: CAD defaults, INIT settings, footprint checks and measured moves/turns.
- `PollenHiveAuto`: pure state machine and feed gates.
- `PollenHiveOpMode` plus two tiny subclasses: hardware mapping and mode selection.
- `PollenHiveTest`: classifier, geometry, motion faults, timing, primary/secondary behavior; uses the existing simple drivetrain simulator.
- `PollenHiveCameraTest`: real JSON parser contract; runs with Gradle/JUnit and was also run directly against FTC 12 jars.
- `tools/hive-camera-record.py`: Mac read-only camera recorder.

Computer tests and direct compilation against FTC 12.0.0/Pedro 3.0.2 libraries passed during preparation. The simple simulation completed default primary UP and tipped-return cases; it assumes idealized launcher response and is not a timing guarantee. A full `:TeamCode:assembleDebug` attempt could not download the Gradle distribution in the preparation environment. **Run the Gradle unit tests and Android build on the Mac before installation. No robot, camera or real-hive validation has been performed with this patch.**

Primary technical references: repo field layout/source links above; FTC SDK 12.0.0 source and dependency declarations; [Limelight AprilTag tracking](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags), [coordinate systems](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems), and [FTC Limelight programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming).
