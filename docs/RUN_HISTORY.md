# Run history: what to keep in Git from downloaded robot runs

For the coding agent on the laptop that has the downloaded sessions (`bash tools/ftc.sh logs ...`
folders and their ZIPs). Goal: one small, reviewable record of every run in this repository, so
any agent, including a cloud session that can only `git clone`, can answer "how often do the match
autos get both a shot and a reload, and where does the time go?" without the raw logs.

Written 2026-11-19 by a Claude cloud session from the code at `b1126e0` plus `pollen-hive-v33`. The
formats below are read from the code (`[repo]`). **No real download was seen in writing this**: check
each field against your local runs and fix this file where it is wrong (add a dated line, per
[research/README.md](research/README.md)). [AGENTS.md](../AGENTS.md) rules apply: raw data is
preserved, nothing on the Hub is deleted, nothing is pushed unless a mentor asks.

## Rules for the history

1. **Never commit raw downloads.** Keep the ZIPs on the laptop and in the team Drive. Do not commit
   `logcat.txt`, `robot-logs/`, `installed-app.txt`, `configs/`, `checkout.txt`,
   `benchmark/*.jsonl` or TeleOp CSVs. The repo is public; SDK logs and `dumpsys` output may hold
   the robot's network name, passphrase or device identifiers `[inferred]`, and the files are large
   (TeleOp up to 8 MiB a run, benchmark 0.1-0.2 GB).
2. **Generated files come from a script, not by hand.** Write `tools/run-history.py` (Python 3
   standard library only, like the other tools) and a fake-data test `tools/test-run-history.py`.
   Add it to `tools/test-ftc-tools.py` or `tools/ftc.sh test` the way the other tool tests are run.
   Running it twice over the same folders must change nothing.
3. **Human facts go in `notes.md`, which the script creates once and never overwrites.**
4. **One run = one report file.** Every download pulls every file still on the Hub, so the same run
   appears in many sessions. Identify a run by its report file name and MD5, not by session.
5. Label simulated or derived numbers as such; only `notes.md` and the robot's own files are evidence.

## Where it goes

```text
runs/
  README.md             3 lines: what this is, link to docs/RUN_HISTORY.md
  index.csv             one row per run, all kinds, sorted by hub start time
  sources.csv           one row per downloaded file used: session, path in session, bytes, md5, run_id
  YYYY-MM-DD/RUN_ID/    date from the Hub clock at START
    summary.md          generated: settings in effect, timeline table, metrics, faults, warnings
    timeline.csv        generated: one row per phase change
    slim-trace.csv      generated, hive autos only (see below)
    notes.md            human: created once from the template below, never overwritten
```

Size check: a 30 s hive trace at 10 rows/s trimmed to the columns below is roughly 300 rows of about
120 bytes, ~40 KB `[inferred]`. Hundreds of runs stay in the low megabytes. If it grows much past
that, keep slim traces only for `run=match` with execution LIVE.

## The files in a download (what to read)

All under `SESSION/pollen-hive/` unless noted.

| Recording | File | Written by | What identifies the run |
| --- | --- | --- | --- |
| Hive autos: Primary Shooter + Garden, Secondary Shooter + Flower, Beta Hero / Out of the Way, Calibration shot-only and Garden + Return | `reports/KIND-MILLIS-NNN.txt` and `traces/KIND-MILLIS-NNN.txt.csv` (same stem) | `PollenHiveOpMode`, `PollenHiveStore`, `PollenHiveTrace` | KIND = `match`, `shot_only` or `garden_return`; MILLIS = Hub wall clock at START; the `START` line's `opmode=` names the mode |
| Calibration Garden Pickup / Flower Drain | `auto/auto-DATE-TIME.csv` | `PollenCollectionPractice` | first row `START,BUILD,profile,overrides` |
| Odo and Emergency autos | `auto/auto-DATE-TIME.csv` | `PollenAutoLog` | `PollenAuto` text lines (`t=.. PHASE`, `RESULT`, `WARNING`, `STOP`) |
| PollenBot TeleOp | `teleop/teleop-DATE-TIME.csv` | `PollenTeleOpRecorder` | `START` row |
| Camera benchmark | `benchmark/hive-*.jsonl`, `SESSION/analysis/summary.md` | benchmark | file name |
| Saved INIT settings | `*.properties` | `PollenHiveStore` | settings at download time, not per run |
| Hub storage | `SESSION/storage.txt` | download script | per session |

**Report lines** (`reports/*.txt`) are `SECONDSs TEXT` (`%.3fs`). In order: `build=.. run=..`;
`START build=.. opmode=.. wall_ms=.. run=.. secondary=.. makeRoom=.. settings=.. camera=..
unsaved_route=.. unsaved_camera=..` (`settings=` lists only rows changed from their defaults);
`CAMERA_VALUES ...`; `HUB installed=.. trace=.. storage=[..]`; one `MAP` line per motor; then one
`PHASE / HIVE_STATE / feed=BOOL reason=.. poseIn=X,Y headingDeg=H` line plus a `DECISION ..` line at
every change of phase, hive state or feed; `FAULT ..` and `STOP requested_by_driver previous=..` at
the end.

**Trace columns** (`traces/*.csv`, every value quoted, 10 rows/s plus every transition, continuing
after DONE until the driver presses STOP): `t_s, wall_ms, kind (SAMPLE|TRANSITION|FAULT|STOP),
phase, reason, decision_inputs, execution (0 DRY, 1 DRIVE, 2 LIVE), odom_status, x_in, y_in,
heading_deg, raw_x_in, raw_y_in, raw_heading_rad, decision_launcher_tps, requested_forward,
requested_strafe, requested_turn, applied_wheels_FL_FR_BL_BR, requested_launcher_tps,
applied_launcher_tps, measured_launcher_tps, applied_intake, applied_agitator, feed_requested,
hive_state, cell, hive_reason, frame_stamp, camera_status, tags_id_xyz_m_height_in, battery_v,
launcher_current_a, frontLeft, frontRight, backLeft, backRight, pollenLauncher, intake, agitator,
drainer_down`. Motor columns hold `sdk_power=.. encoder=.. velocity=..`. Pose is in route
coordinates: inches from the start spot S, x toward the hive, y to the left looking at the hive.

**Phases** (`phase` starts with one of): `DELAY, FIRST, OUTBOUND, GARDEN, FLOWER, RETURN, SETTLE,
ALIGN_SHOT, SECOND, ROOM, WAIT, PARK_WAIT, PARK, DONE`. Moving phases add `: LEG NAME`.
A volley is a run of rows with `feed_requested=true`. From v33 on, `decision_inputs` contains
`clear_out_s_left=` while the post-volley clear-out runs. Builds before v33 have no clear-out.

## `index.csv`: one row per run

Hive-auto runs fill every column; other kinds fill the identity and outcome columns and leave the
rest blank. Times are seconds from START (trace `t_s`). Blank means "did not happen" or "not in
this recording"; never write 0 for that.

| Column | How to get it |
| --- | --- |
| `run_id` | report file stem (`match-1763512345678-123`), or the CSV stem for other kinds |
| `kind` | `hive`, `collection_practice`, `odo_auto`, `teleop`, `benchmark` |
| `hub_start` | ISO time from `wall_ms`. Hub clock: the SDK only corrects it when it reads before 1976, so it can be off; `sources.csv` keeps the laptop's download time as a cross-check |
| `first_seen_session` | earliest session folder that contains the file |
| `build`, `installed` | `START build=`, `HUB installed=` |
| `opmode`, `run`, `secondary`, `make_room` | `START` line |
| `execution` | trace `execution`: DRY / DRIVE / LIVE |
| `settings_changed` | the `START settings=` list as written; empty when all defaults |
| `cell` | most common non-negative trace `cell` |
| `result` | `reason` of the first DONE row, else of the last row |
| `fault` | text of any `FAULT` row or report line |
| `stopped_early` | `STOP requested_by_driver` arrived before DONE |
| `feed1_start`, `feed1_end` | first volley: first and last `t_s` with `feed_requested=true` while phase is FIRST |
| `feed2_start`, `feed2_end` | same while phase is SECOND |
| `both_volleys` | true when `feed1_start` and `feed2_start` are both present |
| `feed2_cut_by_park` | a PARK phase starts while the second volley is running (reason starts `Park deadline during SECOND`) |
| `outbound_start`, `collect_start`, `collect_end`, `return_end`, `park_wait_start`, `park_start`, `done_at` | first `t_s` of OUTBOUND, GARDEN or FLOWER, RETURN, SETTLE, PARK_WAIT, PARK, DONE |
| `spin_up_s` | `feed1_start` minus first row where `requested_launcher_tps > 0` |
| `up_wait_s` | `feed2_start` minus SECOND start (hive check plus settle); for the secondary also FIRST start to `feed1_start` |
| `spare_before_park_s` | `park_start` minus `feed2_end` (negative or blank means no slack) |
| `return_error_in`, `return_heading_deg` | `x_in`, `y_in`, `heading_deg` on the first SECOND row (S is 0,0,0) |
| `final_x_in`, `final_y_in` | pose on the first DONE row |
| `min_launcher_tps_feed1`, `min_launcher_tps_feed2` | lowest `measured_launcher_tps` during each volley |
| `dips_feed1`, `dips_feed2`, `dips_clear_out` | count of drops of `measured_launcher_tps` below 90 % of `requested_launcher_tps` that recover above 95 %, in each window. 10 rows/s can miss short dips: a lower bound, not a ball count |
| `last_dip_into_feed1_s`, `last_dip_into_feed2_s` | time of the last dip after its volley started |
| `battery_start_v`, `battery_min_v` | `battery_v` |
| `max_gap_s` | longest gap between consecutive trace rows (loop stalls) |
| `camera_status_last` | last `camera_status` |
| `trace_rows`, `trace_status` | row count; report `HUB trace=` value (`NONE (...)` means no trace) |

For **collection practice** runs add `collect_s` (START to the last collection phase) and the final
phase. For **Odo autos** add each `PHASE` start time and the `RESULT` line. For **TeleOp** add duration,
number of R1 presses (rising edges of `r1`), rehearsal count, `slowest_loop_ms` max. For a
**benchmark** add one line linking the session's `analysis/summary.md` (do not copy it).

## `timeline.csv` and `slim-trace.csv`

- `timeline.csv`: `t_s, phase, hive_state, feed, reason, x_in, y_in, heading_deg`, one row per report
  transition line. This is the run at a glance.
- `slim-trace.csv` (hive autos): every trace row from START until 1 s after the first DONE (or the
  last row), columns `t_s, kind, phase, execution, x_in, y_in, heading_deg, requested_forward,
  requested_strafe, requested_turn, requested_launcher_tps, measured_launcher_tps, applied_intake,
  applied_agitator, feed_requested, hive_state, cell, battery_v, launcher_current_a`, plus
  `clear_out` (true when `decision_inputs` contains `clear_out_s_left=`). Round to 2 decimals.
  Leave out tags, motor sample strings, `decision_inputs` and `hive_reason`: they are the bulk of the
  bytes and remain in the ZIP.

## `notes.md` template (what only people know)

The logs cannot see balls. Most of what decides "shot and reload" is here, so ask a mentor to fill it
after each session; leave a field blank rather than guess.

```markdown
# RUN_ID
- Event or practice: (League meet + match number, or practice + date)
- Field surface: tiles / vinyl
- Partner and what they did: (tipped our cell at about __ s?)
- Balls loaded at start:
- Balls scored, first volley / second volley:
- Balls left in the robot after the first volley (counted at the garden or flower):
- Balls collected:
- Jam or overload seen: (where, when)
- Parked: yes / no / partly
- Video: (Drive link)
- Anything odd:
```

## What to report after building it

Write `runs/SUMMARY.md` (generated, overwritten each time) with one table per opmode and build,
LIVE runs only:

1. Runs, runs with both volleys, runs where the park deadline cut the second volley, faults.
2. Median and worst of `spin_up_s`, volley 1, outbound, collection, return, `up_wait_s`, volley 2,
   `spare_before_park_s`.
3. `last_dip_into_feed*_s` against the feed setting (is 3 s longer than needed?).
4. `return_error_in` spread; runs where SETTLE or ALIGN_SHOT took over 1 s.
5. From `notes.md`: balls left after volley 1 before and after v33, and any jam.
6. Secondary runs: `up_wait_s` for the first volley against whether volley 2 happened.

Then tell the mentor, in a few lines, which phase costs the most time and which runs lack notes.

## Options considered

| Option | Status | Why |
| --- | --- | --- |
| Index and summaries only | Possible | Smallest; loses the per-row data a cloud agent needs to check a timeline |
| **Index + timelines + slim hive traces + notes** | **Proposed** | Enough to recompute every metric above from Git; tens of KB per run |
| Full traces gzipped in Git | Not proposed | ~10x larger, mostly camera tags; the ZIP in Drive already has them |
| Git LFS or a private data repo for ZIPs | Mentor's call | Keeps raw data versioned; LFS on a public repo has bandwidth limits `[inferred]` |

## Gaps in today's logs (code changes to consider later)

- No ball count. A beam break or distance sensor at the agitator, or a faster launcher-speed log
  during volleys, would give per-ball timing; 10 rows/s is coarse for dips.
- Partner tip time is only visible through `hive_state` from our camera, and only while the robot
  checks the hive.
- The Hub clock can be wrong; record the laptop time of each download in `sources.csv` too.
- Emergency Odo Park, Emergency Timed Park, Calibration Launcher and Calibration Odometry record
  nothing (see [POLLEN_LOGGING.md](POLLEN_LOGGING.md)).
