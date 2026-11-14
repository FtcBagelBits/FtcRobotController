# Hive calibration and practice

Intermittent tags? Start with the [automatic pipeline benchmark](POLLEN_HIVE_BENCHMARK.md).
It records raw detections before calibration passes; the current confirmation rule is unchanged.

Build **pollen-hive-v5**. Experimental, for field testing. Keep the existing match autos available.
Print [the two-page practice sheet](drive-team/print/4-hive-practice.pdf). This guide explains the
controls and checks in full; the patch kit README covers applying, building, installing and pushing.

## Choose the smallest useful test

| Driver Hub entry (Pollen Hive group) | INIT default | Motor permissions and end condition |
| --- | --- | --- |
| TEST Hive camera (NO MOTORS) | No execution row | Never maps motors or odometry. Continuous camera readiness until STOP. |
| TEST Hive primary shot ONLY | DRY | No drive in any execution setting. LIVE enables launcher and feed. One volley or primary window, then DONE. |
| TEST Hive secondary shot ONLY | DRY | No drive. LIVE enables launcher and feed. Waits for UP, one volley, then DONE; 27 s limit. |
| TEST Hive garden + return | DRIVE | No launcher or agitator in any setting. LIVE adds collection intake. Returns to original S and stops; no shot/park. |
| EXPERIMENTAL Hive Primary + Garden | LIVE | Shoot, coast, collect, return, wait/shoot if UP, park. |
| EXPERIMENTAL Hive Secondary | LIVE | Wait for UP, shoot, park. |

The three isolated motor tests use the same controller/settings as the full autos. They still need
Pinpoint READY and correct motor names. **Execution resets to the default above every INIT and is
never saved.** DRY uses virtual launcher readiness and commands zero real motor output. DRIVE
commands only the drivetrain. LIVE enables the mechanisms allowed by the selected test.
Camera-only testing needs just the configured Limelight and the Hub; it does not need Pinpoint.

## Before the hive is available

1. Apply/build/install the patch. Verify `BUILD pollen-hive-v5` on the Driver Hub.
2. Mount the Limelight 3A rigidly on the back, looking toward the hive, with negligible camera
   roll. Measure tiles to **lens center**; 14 inches is a default, not a substitute for measuring.
3. Connect USB-C on the Limelight to the Hub's blue USB-A 3.0 port. Configure it as `limelight`.
4. Pipeline 0: AprilTag **36h11**, tag size **82.55 mm / 3.25 in**, full 3-D results. Record exposure,
   resolution and downscale. Use the camera INIT row if you use another pipeline index.
5. On the Mac, use the printed tags to check IDs, camera connection and interruption/recovery.
   A print at arbitrary height is not a real-hive calibration. A Mac alone cannot run an FTC OpMode;
   computer tests exercise Java logic, while the camera web page exercises live image detection.
6. Once the Hub is available, use TEST Hive camera to enter measured lens height in INIT and
   confirm IDs after START. Do not calibrate to a print at the wrong physical height.

## Camera INIT: measure once, edit without switching modes

D-pad up/down selects one of eight rows; left/right changes it. Triangle restores **one selected
row**. Square saves the camera rows on the Hub. Camera reset is deliberately separate from route
reset. START uses the displayed settings even if not yet saved.

| Camera row | Default | What to do |
| --- | --- | --- |
| Lens height | 14.0 in | Measure floor to lens center. |
| UP tag height | 49.7 in | Measure floor to center of the tag row on the physically UP cell. |
| DOWN tag height | 35.6 in | Measure on the physically DOWN cell when practical. |
| Tilt up | NOT SET | Fit after START with Square, then validate and save with Cross. |
| Sideways correction | 0 in | Fitted with tilt if at least 10 accepted frames see all four tags. |
| Pipeline / band / max sideways | 0 / +/-3 in / 12.5 in | Keep unless field evidence supports changing them. |

Enter measurements rather than rounding them to the nominal field values. The tilt fit absorbs a
lens-height error where it was fitted: with the lens entered 1 in wrong, heights read correctly at
the fitting distance and about 0.5 in off at 1.5 times that distance (error = lens error x
(1 - distance / fitting distance)). The DOWN check at the same spot cannot see a lens-height error;
the check at the other start can. A DOWN height mismatch points at the UP/DOWN tag-height entries
(a 1 in UP-entry error shows as 1 in on DOWN) or at the tag size setting (a 10% pose-scale error
shows as about 1.4 in), not at the lens. Fix the measurements in this menu and repeat calibration. **There is no automatic DOWN-height
rewrite**: changing it to fit a bad camera estimate can conceal a calibration problem.

## First calibration: one mode, three short captures

Place the robot at the **primary start**, front against the end wall, rear camera/launcher toward
the hive. Make the near cell physically UP. Confirm the printed tag IDs belong to that cell:
red primary **34-37**, blue primary **42-45**. For the sideways fit, see all four tags and center the
robot on the cell. Do not use the secondary offset start to fit the sideways correction.

1. START **TEST Hive camera (NO MOTORS)**. Confirm the *Our cell* and *Tags* lines against the real
   hive. The camera test can identify any cell; the operator must verify this is the near cell.
2. With the cell **UP and still**, press **Square** once. Hold still for three seconds.
   `CANDIDATE` means the displayed tilt/sideways numbers are in use but not saved. At least 20
   distinct solved frames spanning one second are required; rejected solves never count. Per-tag
   spread is checked so inconsistent tags cannot cancel into a convincing average.
3. Keep it **physically UP**. The label defaults to UP; press **Circle**, hold still three seconds.
   Expect `PASS: physical label and production detector agree`.
4. Tip that same cell **DOWN**, let it settle, press **D-pad right** to label it DOWN, then **Circle**.
   Expect PASS if its tags are visible. If all its tags disappear, fresh empty camera frames can
   report **BLOCKED ONLY**: no permission observed, not a positive DOWN detection or height check.
   A disconnected/frozen camera cannot satisfy this check. Visible inconsistent heights are a
   rejection, not a hidden-DOWN pass.
5. Press **Cross**. It saves only after both labeled checks succeed on the current profile and
   cell. It saves exactly the rounded values you previewed. STOP / INIT / START to verify reload.

Square always starts a new candidate version and clears both old checks. A rejected new fit
returns the display to the settings used at START; an older candidate is not left saveable.
Repeating a check revokes that state's previous pass if the repeat fails. Triangle forgets the
cell and both check results; use it after moving to another start. Save a good candidate **before**
moving. At another start, validate the saved mount; do not automatically refit it.

| After START | Action |
| --- | --- |
| Square | New three-second tilt fit; cell must physically be UP at primary start. |
| D-pad left / right | Label the next check physically UP / DOWN. |
| Circle | Three-second check against that label, using the production detector. |
| Cross | Save current profile after both labeled checks. |
| Triangle | Forget cell/checks after moving to another start; retains current mount. |

The screen updates four times a second while the detector runs every loop. This persistent mode
shows **IsHiveReadyForShot**, the camera gate only. It does not claim the launcher is at speed or
the robot is settled. The actual shot tests check those additional conditions.

## Field validation, without repeated installs

At each intended shooting start: identify the correct cell while UP, do labeled UP and DOWN
checks, then leave the mode running for the following tests. Secondary IDs: red **30-33**, blue
**38-41**. Use Triangle after changing starts; keep the robot square and correctly placed.

- Tip the hive 20 times. **Zero false UPs** while physically DOWN or tipping. Allow the detector's
  confirmation delay before expecting UP. A frame that shows our tags between the bands removes
  permission on that frame; frames that show no tags at all keep it for up to 0.20 s after the
  last sighting. Write down any tip where the tags vanished without a between-bands reading.
- Cover our tags while the other hive stays visible. Read UNKNOWN / false, never permission from
  the other hive. Unplug/replug the camera and verify recovery needs fresh confirmations.
- Move the robot roughly one inch and a few degrees around the shooting position. Verify UP still
  works. Restore the exact starting placement before motion tests.
- Rock slowly through a tip. Verify UNKNOWN in the transition. Recheck with launcher vibration
  during the stationary shot test; static-camera success alone does not establish live reliability.
  The shot modes log a `HIVE` line for every new camera frame while shooting (tags seen, height,
  state, feed, launcher speed). Pull logcat after a LIVE shot test: a drop in `tags=` right after
  `feed=true` means our own balls or vibration hide the tags.

A height transform remains valid across position/yaw changes **only with a level chassis, rigid
mount, negligible unmodeled roll, and correct tag pose scale**. Recalibrate after moving the mount,
changing lens height, or changing pipeline geometry/resolution/downscale. Confirm at each event.

## Fast motor-testing loop

1. **Primary shot ONLY, DRY:** at the proper S pose, test UP, DOWN, obscured tags. Virtual flywheel,
   no motors. It finishes a volley/window; use camera-only mode for indefinite observation.
2. **Primary shot ONLY, LIVE:** small batch, clear shot area, someone on STOP. No drive output.
   Feed needs settled odometry, fresh steady UP and measured launcher speed >=96% of target.
   Repeat with secondary shot ONLY at the secondary start: DOWN must wait, UP must feed.
3. **Garden + return, DRIVE, no balls:** start at normal primary S. It skips shooting, drives the
   existing CAD route, returns and settles at S, then DONE. No park and no coordinate re-zero midrun.
4. **Garden + return, LIVE:** collection intake runs on pickup/hold, launcher and agitator stay off.
   Check actual pickup, retention with a stopped flywheel, clearances and return error.
5. **Full primary / secondary, LIVE:** integrate only after the isolated tests work. First run
   with small batches. Tip before the return shot to confirm waiting; tip UP to confirm resuming.
   Keep the 27-second deadline and departure reserve. Existing match autos remain available.

Motor tests use the regular INIT menu: D-pad row/value, LB/RB jump five, Triangle resets one row,
Square saves. **Circle resets route values only, keeping camera settings and selected Execution.**
Resetting the Execution row itself uses the selected OpMode's default. All three practice modes
share their corresponding primary/secondary saved settings with full autos; a saved route/feed
edit applies to both. Review the CHANGED and UNSAVED lines every INIT.

## Session reports and promoting results

Camera capture outcomes and saves are recorded automatically in
`/sdcard/FIRST/pollen-hive/reports/camera-*.txt`. Each motor run writes its run type, settings,
phase/state/feed changes, elapsed time and measured pose; the return/finish event includes the
return error relative to S. Reports are best-effort; a write failure is shown and never authorizes
feeding. Detailed live `PollenHive` lines remain in the Robot Controller log. Reports do not
infer actual pickup count or automatically score an unlabeled 20-tip test: record those manually.
Pull the settings and reports once after the session:

```bash
adb connect 192.168.43.1:5555
session_dir="$HOME/Desktop/pollen-hive-$(date +%Y%m%d-%H%M%S)"
adb pull /sdcard/FIRST/pollen-hive "$session_dir"
python3 tools/hive-promote-defaults.py "$session_dir"            # preview
# Only after the numbers are proven on the field:
python3 tools/hive-promote-defaults.py "$session_dir" --write
bash tools/test-pollen-sequence.sh
ftcgradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
git diff
# Commit the intended defaults, then install/push through the usual workflow.
```

Pull the Robot Controller log too (see [operating manual](POLLEN_AUTONOMOUS.md#robot-log)).
Record build/commit, physical cell IDs, lens/UP/DOWN measurements, pipeline settings, false-UP count,
pickup count, return error, and cycle time. Settings files contain overrides, not full snapshots;
keep the commit with the reports. Execution is never promoted; review any forced RED/BLUE side.

## How height and readiness work

Limelight camera coordinates are x right, y down, z out of the lens, in metres. Convert the tag's
vertical displacement to inches after rotating by measured camera pitch:

`tag height = lens height + (z*sin(tilt) - y*cos(tilt)) / 0.0254`

The camera is mounted on the rear; that changes viewing direction, not this vertical transform.
The state classifier requires the accepted visible tags of our locked cell to agree. A state
needs at least four distinct frames showing our tags over 0.15 s, and expires 0.20 s after the
last new frame or the last frame that showed our tags. A contradicting observation (between bands,
tags that disagree, the other state) revokes UP immediately and restarts confirmation. A new frame
without our tags (ball in flight, blur, glare) is not a contradiction: it neither counts nor
resets, until 0.20 s have passed since the last sighting. Uncalibrated, invalid, missing or inconsistent
observations produce UNKNOWN. Only UP grants camera readiness. Camera input rejects wrong pipeline,
invalid poses, duplicate IDs, stale SDK results and excessive total image latency.

Side AUTO locks the nearest eligible cell after four consecutive fresh observations. Primary
allows cells 34/42, secondary 30/38; RED or BLUE forces the appropriate one. Losing observations
before identification now restarts its count. An established lock survives the garden trip.
INIT uses the production detector too, so it does not call an average of conflicting tags UP.

## Default garden route, derived from the saved repo layout

The source of field measurements is [POLLEN_FIELD_LAYOUT.md](POLLEN_FIELD_LAYOUT.md), especially Zones and Game pieces. Nominal red primary start is `(X,Z)=(-12.74164098,61.585)` inches. Garden ball centers are approximately `(-69.31,69.3), (-66.42,69.3), (-63.53,69.3), (-60.64,69.3)`. Blue uses the 180-degree rotated setup, with identical relative route settings.

![Default garden approach and return](POLLEN_HIVE_GARDEN.svg)

Route forward = toward hive from START; route left = left while looking at hive. **All distances are centimeters, absolute robot-center coordinates from START.** Heading 0 is the starting chassis orientation (front toward wall, rear toward hive). Heading -90 degrees turns the front toward the garden on the left. Translation angle and chassis heading are separate.

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

**Intake reach is the remaining mechanical assumption.** At E, the leftmost ball center is 8.725 inches left of the robot centerline. A centered mouth would need about **17.45 inches of usable width** to cover all four centers at once, plus appropriate reach into the ball row. A narrower intake can collect fewer balls on this route. The code cannot confirm ball count. Measure the actual intake envelope/reach and test with balls before claiming a four-ball pickup. The footprint checker assumes the entire robot, including intake, fits within 18×18 inches.

The geometry check (run by the computer tests and at INIT) samples translations at most 0.1 in apart and turns at one-degree steps against the repo's fixed obstacle envelopes, walls and alliance midline. It doesn't account for partners, scattered pieces, build tolerances or odometry error. Start with DRIVE and stop if the path differs from the drawing.

## Route and timing defaults (`PollenHiveSettings.ROUTE`)

Change any of them in INIT (the geometry check blocks START and Square if a change would hit a wall
or the midline), or promote saved changes into the code. The robot log's START line lists every value.

| Setting | Default |
| --- | --- |
| Start delay | 0 s; secondary waits for an actual UP, not a guessed partner delay |
| First / second feed | 1.5 / 1.5 s of actual commanded feed; pauses don't count |
| First-shot window | Primary waits at most 5 s in FIRST; if it has fed, goes to collect; otherwise departs |
| Launcher | 1180 ticks/s; PIDF unchanged |
| Travel / turn / pickup drive power | 0.60 / 0.40 / 0.20 |
| Collection intake / hold | 0.50 power / 0.50 s |
| Shooting intake / agitator | 0.50 / 1.00 |
| Clearance forward / first turn left / garden turn left | 15.24 / 43.836 / 108.822 cm |
| Pickup left / forward | 121.522 / 2.54 cm |
| Transit / pickup heading | -90 / 0 degrees; final shooting heading always returns to zero |
| Execution / Side | LIVE (not saved) / AUTO |
| Park | on; three CAD-derived waypoints (existing departure corridor and parking spot) |

Primary garden is about **11.22 s** and primary park **4.76 s** by the existing drive/turn estimates. These are planning estimates. No default is physically tuned.

## Timing and autonomous behavior

- Primary first volley ends when the feed time is used, a confirmed DOWN follows some feeding, or its first-shot window ends. A missing/ambiguous observation immediately stops feeding but does not invent a tip.
- Before garden collection, the launcher is commanded off. If it doesn't slow below 100 ticks/s within two seconds, collection is skipped and the robot departs, so the shared intake never runs toward a spinning launcher during pickup.
- Garden admission reserves its estimated duration, three seconds for second spin-up, the second feed duration, estimated parking duration and margins. If time is short, it skips collection and parks; telemetry says why.
- After return, position must be within 0.5 inch of START, heading within 2 degrees, and speed/turn rate settled for 0.3 seconds. Old readiness is discarded while traveling; fresh observations reconfirm UP. Our cell stays locked.
- If our cell is DOWN or UNKNOWN after return, primary waits. If it becomes UP in time, it shoots. Secondary behaves the same for its volley. Both reserve departure time and stop all commands at **27 seconds**.
- No camera, or no calibration: the auto still drives and parks; it never shoots.
- A 0.25-second control-loop gap, invalid odometry, lost Pinpoint READY status, stuck drive/turn or route failure latches a stop. There is no guarantee of finishing parking if the robot runs slower than the estimates; the deadline still wins.
- Permission is checked every loop during feeding. Already released balls keep flying.

## Code and verification

`IsHiveReadyForShot` is the state detector; `PollenHiveCalibration` captures/validates;
`PollenHiveCalibrationSession` manages the current candidate and check results. `PollenHiveCheck`
is the motor-free UI. `PollenHiveAuto` has MATCH, SHOT_ONLY and GARDEN_RETURN run types; hardware
limits are also enforced in `PollenHiveOpMode`. Settings and routes remain in `PollenHiveSettings`
and `PollenHiveRoute`. `PollenHiveStore` writes overrides and compact reports.

Computer regressions cover the wrong-state fit, impossible solves, per-tag disagreement, duplicate
frames/minimum span, hidden versus disconnected DOWN, candidate resets, cell identification,
settings and primary/secondary/practice simulations. The camera decoder has 13 real-JSON checks.
All hive hardware classes compile against genuine FTC 12.0.0 / Pedro 3.0.2 libraries. The kit's
mail patch is checked on its exact base. **No full Android APK build or physical camera/robot/hive
validation was performed here.** Run the README's Gradle checks on the Mac before installing.
