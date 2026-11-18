# Recordings and Control Hub storage

Build **pollen-hive-v10**. What the robot writes down in each mode, where it goes, how much room
it may use, how to get it off the Control Hub, and how to turn a move rehearsed in TeleOp into
draft autonomous waypoints.

Status: the code and tools below pass the computer tests. **Nothing here has run on the robot
yet** (written in a cloud session, 2026-11-16). The first-run checks are at the end.

## What is recorded

| Mode | Robot log (`PollenAuto` / `PollenHive` / `PollenTeleOp` lines) | Its own file under `/sdcard/FIRST/pollen-hive` |
| --- | --- | --- |
| **Calibration Garden Pickup / Flower Drain** | Driver Hub shows current phase and elapsed seconds. | `auto/auto-DATE-TIME.csv` records settings overrides, phase times and poses, final state and faults. See [collection practice](POLLEN_COLLECTION_PRACTICE.md). |
| **PollenBot TeleOp** | START line with every setting; STOP line | `teleop/teleop-DATE-TIME.csv`: 20 rows a second while anything is happening (1 a second when idle): sticks, triggers, R1/L1, power to all four wheels, intake and agitator power, launcher target and measured speed, launching/feeding/reversing, Pinpoint position, battery, slowest loop. Rehearsal marks. |
| **Odo Shoot + Park, Odo Supporting Shooter, Angled, Emergency Timed Shoot/Support** | Route during INIT, every setting at START, phases, warnings, result, 4 samples a second | `auto/auto-DATE-TIME.csv`: the same lines, with the Hub's clock. New in v10. |
| **Hive autos and Calibration shot and garden modes** | START, transitions, camera frames while shooting | `reports/*.txt` (settings, motor map, decisions) and `traces/*.csv` (10 rows a second: decisions, motors, encoders, camera tags, battery). v10 adds every camera value, the trace name, install time and storage to the report. |
| **Calibration Hive Camera (No Motors)** | CHECK lines | `reports/camera-*.txt` |
| **Calibration Hive Pipeline Benchmark** | none | `benchmark/hive-*.jsonl` (large) |
| **Emergency Odo Park, Emergency Timed Park, Calibration Launcher, Calibration Odometry** | **nothing** | **nothing** (not changed by this build) |

Every START line now also has `installed=` (when the app was last installed, by the Hub's
clock), so two installs with the same Build label can be told apart, and `storage=[...]`.

TeleOp does **not** record: gamepad 2, drive-motor encoders or currents, the camera, or any
mechanism that is not in `BioBuzzStarterbotOpMode` yet (the flower drainer servo, when it is
merged, needs a column added in `PollenTeleOpRecorder`).

## Where it lives and how big it can get

| Folder | One run may write | Folder budget | When the budget is used up |
| --- | ---: | ---: | --- |
| `teleop/` | 8 MiB (about 24 min of nonstop driving) | 128 MiB (about 6 h) | New TeleOp runs are not recorded; the robot drives normally |
| `auto/` | 1 MiB (a 30 s auto is about 20 kB) | 32 MiB | Match autos log to the robot log only |
| `traces/` | 2 MiB | 64 MiB | Hive autos run without a trace |
| `reports/` | a few kB | none | |
| `benchmark/` | 500 MiB | none | v10: a benchmark refuses to start with less than about 600 MB free |

- **Nothing on the robot ever deletes a recording.** A full budget stops new recordings of that
  kind and says so on the INIT screen; it never stops the robot.
- No recording starts unless 32 MiB plus the run limit would still be free.
- The largest the three budgeted folders can reach together is 224 MiB. The benchmark folder is
  the one that can fill a Hub: each full run is 0.1-0.2 GB.
- The SDK's own logs limit themselves: `robotControllerLog.txt` rotates at 4 MB and keeps 4 older
  copies (about 20 MB), and only the last 9 match logs are kept (SDK source: `RobotLog.java`,
  `AppUtil.MAX_MATCH_LOGS_TO_KEEP`). That is also why they are not a safe long-term record: a
  busy hive session can push an earlier run out of the robot log.
- Sizes measured in a computer simulation: 289 bytes per TeleOp row, about 130 bytes per auto line.

## How much room is left

1. **On the Driver Hub, every INIT** of TeleOp, the match autos and the hive autos shows
   `Hub storage: 5.21 GB free | teleop 20/128 MB, auto 1/32 MB, traces 12/64 MB, ...`.
   A `STORAGE WARNING` line appears when a folder is 80% full, when a budget is used up
   (`NOT RECORDING`), or when less than 500 MB is free.
2. **From the laptop:** `bash tools/ftc.sh storage` (read-only).
3. **In every download:** `storage.txt`.

REV lists the Control Hub's storage as 8 GB; some units have 16 GB. What matters is the free
number above: it has not been read from this robot yet.

## The lifecycle

1. **Record.** Each run opens its own file at START and closes it at STOP.
2. **Download** after each block of practice and after every event, with the OpMode stopped:
   `bash tools/ftc.sh logs "$HOME/Desktop/session-$(date +%Y%m%d-%H%M%S)"`.
   The ZIP holds everything: recordings, reports, settings, SDK logs, `teleop-summary.md`,
   `auto-summary.md` and a `rehearsals/` folder.
3. **Keep the ZIP** (laptop and the team Drive). After step 5 it is the only copy.
4. **Check:** `bash tools/ftc.sh storage`.
5. **Prune when a warning shows**, or before an event:
   `bash tools/ftc.sh prune "$HOME/Desktop/session-..."` lists what would go;
   add `--delete` to do it. It removes from the Hub only recordings whose checksum equals the
   downloaded copy. Saved settings (`*.properties`), configurations and SDK logs are never
   touched. A file that changed since the download, or was never downloaded, stays.

Every download pulls every recording still on the Hub, so downloads get slower until a prune.

## Rehearse a move in TeleOp, then make it autonomous

For moves that are easier to find by driving than by measuring: collecting from the garden,
lining up on the flower, a parking path.

1. Put the robot where the autonomous would begin the move, facing the way it would face. For a
   move after the first volley that is the start spot S: front flat on the end wall, launcher to
   the hive. **Keep it still through TeleOp INIT** (the Pinpoint calibrates then). INIT must show
   `Recorded position: Pinpoint READY`.
2. START. Press **Circle**: one buzz, and the screen shows `REHEARSAL 1`.
3. Drive the move the way the robot should do it: smoothly, **without turning** (today's routes
   hold one heading), intake on the triggers where the auto should collect. Stopping for a
   moment is fine: a pause becomes a hold.
4. Press **Triangle**: two buzzes. To try again, put the robot back and press Circle: that is
   rehearsal 2. Note which ones were good.
5. STOP, then download. `teleop-summary.md` lists each rehearsal: its timeline, a waypoint table
   in cm from the Circle spot (forward = toward the hive, left = left looking at the hive), each
   leg as `cm @ angle` like the Odo menus, and draft `move(...)` lines in the style of
   `PollenHiveRoute`. `rehearsals/*.svg` draws the driven path and the waypoints. By hand:
   `python3 tools/teleop-rehearsal.py FOLDER --out FOLDER/rehearsals`
   (`--tolerance-in 2` gives fewer waypoints; `--absolute` measures from the INIT pose).
6. Put the numbers into a route (hive INIT rows or `PollenHiveRoute`), then test in **DRIVE**
   with an empty robot before **LIVE**.

What the tool does not do: it copies where the driver went, not how fast. Autonomous uses its own
power and ramps, so read the planned time on INIT. It does not check walls or the flower; the
hive route's own footprint check still does. A turn of more than 5 degrees is flagged, not
converted.

## First checks on the robot

1. INIT PollenBot: Build `pollen-hive-v10`, a `Hub storage` line with a believable free number,
   `Recorded position: Pinpoint READY`.
2. Drive 30 s, mark one rehearsal, STOP. Loop time: compare `Slowest loop` in
   `teleop-summary.md` with what the drivers feel; TeleOp now reads the Pinpoint every loop and
   the battery once a second.
3. Run one Odo auto. The download should hold one `auto/*.csv` whose rows match the `PollenAuto`
   lines in `robotControllerLog.txt`.
4. `bash tools/ftc.sh storage`, then `bash tools/ftc.sh prune FOLDER` without `--delete`: the
   list should name every downloaded recording. This is the first proof that the Hub's shell
   has `md5sum`, `find` and `du` as the tool expects. Only then use `--delete`.
5. Check the date in a file name against the real time. The file names and `wall_ms` use the
   Hub's clock, which the SDK only corrects when it is wildly wrong.

Evidence and open options: [research note](research/logging-and-rehearsal.md).
