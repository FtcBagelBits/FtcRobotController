# Logging, Hub storage and rehearsing autonomous in TeleOp

Question: are we recording enough to explain a run afterward, can the recordings fill the Control
Hub, and can a move driven in TeleOp become part of an autonomous route?
Started: 2026-11-16 by Claude (cloud session). Based on branch `main` at `f792a2c`.

**Where this ran:** a cloud session. It read this repository and the public SDK source, and ran
the computer tests. It did not reach the laptop, the Control Hub or the Driver Hub. Labels are
explained in the [folder README](README.md).

## Summary

Before `pollen-hive-v10`: hive autos were well recorded (report + trace), match autos went only
to the robot log (which rotates), TeleOp and the park-only autos recorded nothing, and nothing
measured or freed Hub storage. v10 adds a TeleOp recording with rehearsal marks, a file per
match auto, a storage line on INIT, `tools/ftc.sh storage` and `prune`, and
`tools/teleop-rehearsal.py`. Operating instructions: [Recordings and Control Hub storage](../POLLEN_LOGGING.md).

## What is known

- `[repo]` Before v10, `PollenAutoLog` was used only by `PollenShooterOpMode` and its sink was
  `RobotLog` alone (`main` at `f792a2c`). `BioBuzzStarterbotOpMode`, `PollenOdoPark`,
  `PollenRobotPark` and `LauncherTestOpMode` wrote no log lines and no files.
- `[repo]` `PollenHiveTrace` caps one trace at 2 MiB and the `traces` folder at 64 MiB and never
  deletes; before v10 a full budget was visible only on the run screen, not in INIT.
- `[repo]` `PollenHiveBenchmark` caps one run at 500 MiB and showed free space in INIT, but did
  not refuse to start when the Hub was short of room.
- `[doc]` The SDK writes logcat to `robotControllerLog.txt` with `-r 4096 -n 4`: 4 MB per file,
  4 rotated copies. `RobotLog.java` (`kbLogcatQuantum`, `logcatRotatedLogsMax`),
  https://github.com/OpenFTC/Extracted-RC
- `[doc]` The SDK keeps at most 9 match logs: `AppUtil.MAX_MATCH_LOGS_TO_KEEP = 9`,
  `RobotLog.pruneMatchLogsIfNecessary` (same repository).
- `[doc]` The SDK sets the Hub's clock from outside only when the Hub's own clock reads before
  1976: `AppUtil.setWallClockIfCurrentlyInsane` (same repository). So recorded times are Hub time.
- `[doc]` Game Manual 0's "Control System Electronics" page says REV officially lists 8 GB eMMC
  for the Control Hub, and that the units it checked all had 16 GB chips (read on the mirror
  https://gm0-ru.readthedocs.io/ru/latest/docs/software/adv-control-system/control-system-internals.html).
  REV's own Driver Hub page lists 8 GB eMMC for the Driver Hub
  (https://docs.revrobotics.com/duo-control/control-system-overview/driver-hub-specifications).
- `[repo]` `origin/test/pollendrainer` adds the drainer servo to TeleOp on D-pad up/down and
  Cross (A). The rehearsal marks use Circle and Square so they do not collide with it.
- `[tested]` Cloud session, 2026-11-16, computer only (SDK stand-ins, simulated clock, no robot):
  a simulated TeleOp run wrote 289 bytes per row; an Odo auto wrote the same 55 lines to the
  robot log and to its file; TeleOp kept driving when the Pinpoint was missing and when it began
  throwing mid-run.

## Options

| ID | Option | Status | Evidence | Next step |
| --- | --- | --- | --- | --- |
| R1 | Mark rehearsals in TeleOp (Circle/Square), convert the recorded Pinpoint path to waypoints on the laptop | Trying (v10) | Computer tests only | First-run checks in POLLEN_LOGGING.md; compare one rehearsal's waypoints with tape-measured positions |
| R2 | A "TEST replay rehearsal" autonomous that drives the latest rehearsal's waypoints straight from the file on the Hub, DRIVE only | Proposed | `[inferred]` shortens rehearse-to-test from a download and rebuild to two button presses | Decide after R1 works; needs the footprint check from `PollenHiveRoute` applied to arbitrary waypoints |
| R3 | Replay the recorded motor powers open loop | Rejected | `[inferred]` battery voltage, wheel slip and start-position error make timed replay drift; the Nov 7 meet already showed odometry beating timed driving (`[reported]` Ari, 2026-11-09) | none |
| R4 | Record the drainer servo and any new mechanism in TeleOp | Proposed | `[repo]` not in `main` TeleOp yet | Add a column when `test/pollendrainer` is merged |
| R5 | Give the park-only autos and LauncherTest the same log | Proposed | `[repo]` they record nothing | Small: `PollenAutoLog` already takes a `PollenSequence.Output` |
| S1 | Never delete on the robot; budgets per folder; prune from the laptop only after a checksum match | Trying (v10) | Fake-adb tests only | Run `prune` without `--delete` on the real Hub first |
| S2 | Download only what is new (today every download pulls every file) | Proposed | `[repo]` `tools/hive-benchmark-download.sh` | Not needed if S1 is used regularly |
| S3 | Shrink benchmark logs (raw JSON repeats on duplicate polls) | Deferred | Handoff note, 2026-11-14 | After the benchmark data has been analyzed once |

## Not verified

- Free space on this robot's Hub: no `df` output has been seen by a cloud session.
- That the Hub's shell (Android 7 toybox) has `md5sum`, `find -exec` and `du -k` with the output
  the prune tool parses. If `md5sum` is missing, prune removes nothing.
- TeleOp loop time with the Pinpoint read and the file write added.
- That the Pinpoint's calibration at TeleOp INIT is harmless right after autonomous in a match.
- Whether the Hub's clock is right. File names and `wall_ms` depend on it.
- How well driver-made waypoints transfer: a rehearsal has not been compared with a measured path.

## Decision log

- 2026-11-16, Claude cloud session: proposed R1 and S1 as a patch for Ari to review. The patch
  changes one AGENTS.md rule (remote deletion is allowed only through `tools/ftc.sh prune`, on a
  mentor's request). Not a decision until a mentor merges it.

## Sources

- OpenFTC/Extracted-RC, `RobotCore/.../util/RobotLog.java`, `.../internal/system/AppUtil.java`,
  `.../eventloop/opmode/OpModeManagerImpl.java`, read 2026-11-16.
- Game Manual 0 control-system internals and the REV Driver Hub specifications page, read 2026-11-16.
