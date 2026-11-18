# Run-recording branch: first robot validation

Plan prepared on the local Mac, 2026-11-16, from `origin/pollen-run-recording`
at **681298a37fc4577879c5cffffecec5f546ccc94d**. Expected build: **pollen-hive-v10**.
The checkout stayed on `pollen-hive-ready`; this document does not mean v10 is installed.
No tests, build, installation or physical trials were performed while preparing this plan.

Goal: establish that recording preserves normal operation, produces complete and useful evidence,
and converts a measured TeleOp rehearsal into an accurate draft path. Storage cleanup is a
separate, mentor-authorized test. Read `POLLEN_LOGGING.md` on the recording branch for its controls
and budgets. Existing hive calibration and motion procedures still apply.

## People, equipment and records

Use a driver on STOP and a second person taking notes/video. Bring the Mac, charged battery,
tape measure, floor tape, and a clear practice area. Start with an empty robot. Keep the robot
still throughout TeleOp INIT because v10 initializes/calibrates the Pinpoint there.

For each trial record: test ID, source SHA, displayed Build, `installed=` value, OpMode, execution
setting if present, effective tuning, battery, approximate real start time, recording filename,
rehearsal number, observed result, intentional STOP or fault, and associated video. Measure robot
center positions and heading consistently; record a Hub clock offset rather than renaming raw files.

Stop a physical trial for unexpected movement, loss of control, unusual feed, or an odometry fault.
STOP and download before investigating. Do not tune launcher speed, PIDF, directions or calibration
to make a recording test pass.

## 1. Computer checks before field time

Run these from the actual recording-branch checkout, confirming its SHA first. These are the
execution checklist for a later validation session, not commands already run for this plan.

```bash
bash tools/ftc.sh doctor
bash tools/ftc.sh test
bash tools/ftc.sh gradle :TeamCode:testDebugUnitTest :TeamCode:assembleDebug
git diff --check HEAD
```

Pass: computer regressions and Android unit tests/build succeed. The computer suite includes
`PollenRecordingTest`, rehearsal conversion, fake-ADB storage/prune, and download tests. Check
their output actually appears. Cover these failure cases with desktop fixtures: no recording
file, write failure, run limit, full folder budget, missing/non-READY positions, incomplete CSV,
checksum failure, changed/undownloaded files, and invalid download paths. Do not fill the real Hub
or disconnect hardware during motion to reproduce them.

Only install when deployment is requested and the operator has confirmed STOP. Use the documented
`ftc.sh install` or `deploy` workflow. At Driver Hub INIT verify **pollen-hive-v10**; retain the APK
and source identity used. A build label alone cannot distinguish two installs of v10.

## 2. First 10 minutes: prove the whole recording loop

| ID | Operator action | Required result |
| --- | --- | --- |
| S1 | INIT PollenBot TeleOp, stationary. Photograph the screen. | Correct Build; plausible Hub storage/free space; recorded position reaches Pinpoint READY. No mechanism starts during INIT. |
| S2 | START, idle 5 s; Circle; drive a short straight path slowly, pause 1 s, return; Square; idle 5 s; STOP. About 30 s total. | Circle gives one buzz and REHEARSAL 1; Square gives two buzzes and ends it. Driving and STOP feel normal. |
| S3 | After STOP, download with `bash tools/ftc.sh logs NEW_SESSION_FOLDER`. | New folder and ZIP; raw `pollen-hive/teleop/teleop-*.csv`, `storage.txt`, `teleop-summary.md`, and rehearsal Markdown/SVG under `rehearsals/`. |
| S4 | Open the CSV, summary and drawing; inspect analysis error files. | Settings-bearing START, matching marks, final STOP; sensible battery/pose values; summary names the rehearsal and drawing matches its direction. No unexplained analysis error. |

Do not spend the full practice session collecting runs until S1–S4 pass. A successful ZIP creation
does not prove analysis succeeded: the downloader preserves raw data even when an analyzer fails.

## 3. TeleOp controls, boundaries and responsiveness (15–20 minutes)

Run the following short trials, downloading after each block. Use empty mechanisms first; run
loaded shooting only after the empty checks pass and the normal shot area is clear.

| ID | Trial | Acceptance / evidence |
| --- | --- | --- |
| T1 | Command forward/back, left/right strafe, and rotation separately; exercise intake in/out, R1 launcher/feed and L1 reverse using established controls. | Controls match established behavior; CSV inputs, four wheel commands, mechanism commands and launch/feed/reverse flags agree with the filmed sequence. Launcher target and measured speed remain distinguishable. |
| T2 | Idle 10 s, drive 10 s, idle inside a rehearsal 5 s; let an empty launcher coast after release. | Approximately 1 SAMPLE/s when fully idle, up to 20/s during activity, rehearsal pauses and launcher coast above 50 ticks/s. Sustained lower rate is investigated alongside loop timing; 20 Hz is a sampling cap. |
| T3 | Hold Circle 2 s, release; hold Square 2 s. Press Square with no rehearsal open. Then Circle, release, Circle again. Finish another open rehearsal with STOP. | Held buttons create one event each. Square without a rehearsal does nothing. A new Circle closes the previous rehearsal and starts the next. STOP closes the open rehearsal and writes one final STOP. |
| T4 | Two consecutive TeleOp runs with different recorded paths. | Separate files, each with its own START/STOP and rehearsal numbering. No path or settings carried into the wrong run. |
| T5 | One normal 2-minute TeleOp practice run. Compare driver response with the established v9 experience and review `slowest_loop_ms`. | No driver-visible lag, new control dropouts or recording-related exceptions. Flag any loop gap >=250 ms for investigation; this is a review threshold, not a new TeleOp stop rule. Inspect raw timing as well as the summary. |
| T6 | With robot powered off, use an approved configuration without Pinpoint; restart and run a short controlled TeleOp trial. Restore the normal configuration afterward. | TeleOp remains usable; missing position is visible and recorded as unavailable, not a plausible zero path. Analyzer provides a timeline without fabricated waypoints. Skip and mark untested if a suitable configuration is unavailable. |

Logging commands are software evidence, not independent confirmation of physical motor wiring.
Use observation/video for what actually moved. Gamepad 2, drive encoders/currents, camera data and
the unmerged drainer servo are outside this TeleOp recording's coverage.

## 4. Autonomous-to-TeleOp handoff (10 minutes, priority check)

This is a new hardware interaction: TeleOp now opens the Pinpoint during INIT.

1. Run an already validated autonomous from its marked start, empty robot first. STOP normally.
2. With the robot stationary at its actual finishing pose, select and INIT PollenBot TeleOp.
3. Confirm Pinpoint READY, then START. Drive forward/strafe/turn slowly; STOP and download.
4. Repeat the transition three times, including a fresh RC-app start in one cycle.

Pass: no movement during TeleOp INIT, no new fault/delay that prevents normal handoff, normal
TeleOp controls, fresh usable position data and a complete file each time. A new position origin
at TeleOp INIT is expected; continuity with the autonomous coordinate origin is not required.
Do not approve match use until this handoff is established.

## 5. Rehearsal accuracy against floor tape (15–20 minutes)

Use an open area. Keep the chassis at its starting heading. Tape a path from robot-center origin:
**60 cm toward the hive, 60 cm left, pause 1 s with intake on, then retrace to the origin**.
Keep the launcher off. Circle at the origin; Square after returning. Drive it three times as
separate rehearsals and note actual endpoint offsets with the tape measure.

Pass targets below are proposed acceptance criteria for this trial, not established robot accuracy:

- Correct signs/axes, approximately `(60,0)` then `(60,60)` cm, and a return near `(0,0)`.
- Recorded corners/endpoints agree with measured positions within **2.54 cm (1 inch)** per axis;
  heading stays within **2 degrees** of the starting heading. Investigate systematic error before
  loosening these targets. Keep simplification error separate from odometry error.
- The summary preserves the intake interval and approximately 1-second hold (within 0.2 s).
  Inspect both the waypoint table and draft lines: a hold is emitted as a comment and needs
  an actual hold step when someone later implements the route.
- Repeat with Circle pressed after driving away from the INIT origin. Default output must begin
  at the Circle pose; `--absolute` must instead reflect the INIT origin. For these useful runs,
  keep Circle heading aligned with the intended autonomous start heading.
- On one separate empty trial deliberately rotate more than 5 degrees. The converter must flag
  the turn. Its draft `move(...)` lines use heading zero; do not adopt them as a faithful turn replay.

Review the SVG for corners lost by simplification. A copied driver path has no automatic wall,
obstacle, partner or footprint clearance validation. Approving the recording/converter does not
approve a new route. Implementing and testing a converted autonomous is a separate follow-up:
review geometry/time budget, empty DRIVE first, then loaded LIVE under existing procedures.

## 6. Autonomous and hive recording regression (15–25 minutes)

| ID | Trial | Required result |
| --- | --- | --- |
| A1 | One previously validated Odo Shoot + Park run, empty first. | One `auto/auto-*.csv`; buffered INIT information, START settings/Build/install/storage, phases, periodic samples and STOP. Compare message text with the same run's `PollenAuto` robot-log lines, allowing the CSV's INIT prefix. Normal completion and timing. |
| A2 | Short trials of Supporting Shooter, Angled and emergency timed shoot/support modes that will be used. Include one operator STOP before completion. | Each supported mode writes its own complete file and records the stop/result. Mark modes not exercised as untested. |
| H1 | TEST Hive primary shot ONLY, DRY. | Existing zero-output behavior; report and trace present. New `CAMERA_VALUES` includes every camera setting; `HUB` identifies install, trace filename and storage. Values agree with INIT, including defaults. |
| H2 | TEST Hive garden + return, DRIVE, empty; then a previously validated combined DRIVE route if needed for match coverage. | Normal route, mechanism restrictions and return behavior; trace/report still usable by the hive analyzer. No new loop-stall faults from recording. |
| H3 | One short single-pipeline benchmark with ample free space, after camera readiness is established. | Normal recording, STOP/download and summary; no changed saved calibration/pipeline settings. Low-space start refusal is checked with desktop fixtures, not by filling the Hub. |

`auto-summary.md` currently analyzes hive **traces**, not the new match `auto/*.csv` files.
Inspect A1/A2 CSVs directly; an empty hive summary is not a match-recording failure. Park-only
modes, LauncherTest and Odometry Check still have no new run file and are not positive file tests.
Keep existing hive UP/DOWN/feed gating tests for any intended LIVE use; this branch's added
recording does not establish loaded pickup or shooting reliability.

## 7. Storage and archive checks (5–10 minutes)

After STOP, run `bash tools/ftc.sh storage`. Compare free space/folder usage with INIT, allowing
for intervening files, rounding, and filesystem allocation. INIT is a snapshot, not a live counter.
Check that new files increase usage sensibly and that the Hub clock/file dates are understood.

Run `bash tools/ftc.sh prune DOWNLOADED_SESSION_FOLDER` **without `--delete`**. It must list the
downloaded matching recordings and remove nothing. A second short run made after that download
must be listed as not downloaded and kept. Confirm settings remain outside the candidate list.
This validates real-Hub `find`/`md5sum` compatibility; fake-ADB tests cannot establish it.

Actual deletion is excluded from this initial plan: current checkout instructions prohibit remote
deletion. Even on the recording branch, its revised rule requires an explicit mentor request in
the session, STOP, a reviewed preview, and a verified retained archive before `--delete`. If later
authorized under applicable instructions, verify only matching recordings disappear; new/changed
files, properties, configurations and SDK logs remain, and a subsequent new recording succeeds.

Open the ZIP and verify raw CSV/report files are present and readable. Retain the original session
and ZIP, notes and video; copy the archive to team storage using the team's approved workflow.
Missing mandatory hive data, unreadable CSVs or unexplained analyzer errors fail the archive check.

## Release decision

Approve normal recording use only after computer/Android checks, the smoke test, TeleOp controls
and responsiveness, three handoffs, intended autonomous modes and a verified download pass.
Approve rehearsal drafts for engineering use separately, after measured-path checks pass.
Storage deletion remains a separate authorization and validation. Record any skipped test as
**NOT TESTED**, not PASS; do not let a successful recording stand in for a successful robot task.

Result sheet (one row per trial):

| Test ID / repetition | Build / installed / source SHA | File / rehearsal | Measured result / loop timing | PASS / FAIL / NOT TESTED | Notes / video / archive |
| --- | --- | --- | --- | --- | --- |
| | | | | | |
