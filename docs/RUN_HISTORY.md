# Run history: what to keep in Git from downloaded robot runs

**Policy for every agent and person who has downloaded sessions on a laptop** (`bash tools/ftc.sh
logs ...` folders and their ZIPs). Goal: one small, reviewable record of every run in this
repository, so any agent, including a cloud session that can only `git clone`, can answer "how often
do the match autos get both a shot and a reload, and where does the time go?" without the raw logs.

Written 2026-11-19 by a Claude cloud session from the code at `b1126e0` plus `pollen-hive-v33`.
Built and checked against 21 real downloads (2026-11-12 to 2026-11-18, builds v3-v29) the same day
by a Claude local session; corrections from that data are in
[What the real downloads showed](#what-the-real-downloads-showed). [AGENTS.md](../AGENTS.md) rules
apply: raw data is preserved, nothing on the Hub is deleted, nothing is pushed unless a mentor asks.

## Adding your downloads (the procedure)

For any agent on the laptop (Claude, Codex, Astra, anyone else). Work in the laptop checkout on
whatever branch is checked out, as AGENTS.md says. The history is shared, so add to it rather than
start a second one.

1. `git status --short runs/` first. Uncommitted or untracked files there are normally an earlier
   agent's finished work (agents run one after another on purpose, see AGENTS.md): build on them.
   Stop and ask only if another agent seems to be running the tool right now.
2. Run the tool over the sessions you have, as folders, ZIPs, or a folder that holds them:

   ```bash
   python3 tools/run-history.py ~/Desktop
   ```

   ```bash
   python3 tools/run-history.py ~/Desktop/hive-session-20261119-170000 ~/Downloads/other-session.zip
   ```

   It reads only `SESSION/pollen-hive/`, never changes it, and prints how many runs changed.
   Runs already in `runs/` are kept: a run another agent added from a download you do not have stays
   as it is. A run you also have is re-derived only when your copy is larger (a download made while
   the run was still being written is shorter) or the tool version changed.
3. Run it a second time: it must print `0 run file(s) changed`.
4. Check what you are adding: `git status --short runs/` and `git diff --stat runs/`. Only files the
   tool writes may appear (see the layout below). If any raw file shows up, stop and report it.
5. Read `runs/SUMMARY.md` and tell the mentor what changed: new runs, which phase costs the most
   time, and which runs still need `notes.md`.
6. Leave the result in the working tree. Commit only when the user asks (AGENTS.md).

Never rename, edit or delete a generated file by hand; fix `tools/run-history.py` (and its test,
`tools/test-run-history.py`, which `bash tools/ftc.sh test` runs), bump its `TOOL_VERSION`, and rerun.
Never delete a `notes.md`.

## Rules for the history

1. **Never commit raw downloads.** Keep the ZIPs on the laptop and in the team Drive. Do not commit
   `logcat.txt`, `robot-logs/`, `installed-app.txt`, `configs/`, `checkout.txt`, `storage.txt`,
   `*.properties`, `benchmark/*.jsonl`, full traces or TeleOp CSVs. The repo is public; SDK logs and
   `dumpsys` output may hold the robot's network name, passphrase or device identifiers
   `[inferred]`, and the files are large (TeleOp up to 8 MiB a run, benchmark 0.1-0.2 GB).
2. **Generated files come from `tools/run-history.py`, not by hand.** Python 3 standard library only.
   Running it twice over the same folders changes nothing.
3. **Human facts go in `notes.md`, which the script creates once and never overwrites.** It creates
   one for hive autos, collection practice and Odo autos (the runs where balls matter). For a TeleOp
   run, a person may create `notes.md` by copying one; the script will count it.
4. **One run = one recording file.** Every download pulls every file still on the Hub, so the same
   run appears in many sessions. A run is identified by its file name; `sources.csv` lists every
   session and MD5 it was seen with.
5. Label simulated or derived numbers as such; only `notes.md` and the robot's own files are evidence.

## Where it goes

```text
runs/
  README.md             3 lines: what this is, link here
  index.csv             one row per run, all kinds, sorted by Hub start time
  sources.csv           one row per downloaded file used: session, session_time, path, bytes, md5, run_id
  SUMMARY.md            generated report (see the end of this file), overwritten each time
  YYYY-MM-DD/RUN_ID/    date from the Hub clock at START (Pacific)
    metrics.json        generated: the index row plus full settings and source MD5s. index.csv and
                        SUMMARY.md are rebuilt from these, so no agent needs another agent's ZIPs
    summary.md          generated: settings in effect, metrics, timeline table, faults, warnings
    timeline.csv        generated: one row per phase change (or per event for other kinds)
    slim-trace.csv      generated, hive autos with a trace only (see below)
    notes.md            human: created once from the template below, never overwritten
```

Size, measured 2026-11-19: 196 runs (66 hive autos) from 21 sessions take about 2 MB in 721 files;
the slim traces are under 1 MB of that. If it grows much past 20 MB, keep slim traces only for
`run=MATCH` with execution LIVE.

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
| `kind` | `hive`, `camera_check` (`reports/camera-*.txt`), `collection_practice`, `odo_auto`, `teleop`, `benchmark` |
| `hub_start` | ISO time (Pacific) from `wall_ms`, else the MILLIS or `DATE-TIME` in the file name. Hub clock: the SDK only corrects it when it reads before 1976, so it can be off; `sources.csv` keeps the laptop's download time (from the session folder name) as a cross-check |
| `first_seen_session` | earliest session folder that contains the file |
| `build`, `installed` | `START build=`, `HUB installed=` |
| `opmode`, `run`, `secondary`, `make_room` | `START` line |
| `execution` | trace `execution`: DRY / DRIVE / LIVE |
| `settings_id` | first 8 hex digits of the MD5 of the `START settings=` list: runs with the same id ran the same settings. The list itself is in `metrics.json` and `summary.md` |
| `cell` | most common non-negative trace `cell` |
| `result` | `reason` of the first DONE row, else of the last row |
| `fault`, `warnings` | text of any `FAULT` row or report line; Odo autos' `WARNING` lines |
| `stopped_early` | `STOP` arrived before DONE and there was no fault |
| `feed1_requested_s`, `feed2_requested_s` | `requested_s=` in the first feeding row's `decision_inputs` (the feed time in effect) |
| `feed1_start`, `feed1_end` | first volley: first and last `t_s` with `feed_requested=true` while phase is FIRST |
| `feed2_start`, `feed2_end` | same while phase is SECOND |
| `both_volleys` | true when `feed1_start` and `feed2_start` are both present |
| `feed2_cut_by_park` | a PARK phase starts while the second volley is running (reason starts `Park deadline during SECOND`) |
| `outbound_start`, `collect_start`, `collect_end`, `return_end`, `second_start`, `park_wait_start`, `park_start`, `done_at` | first `t_s` of OUTBOUND, GARDEN or FLOWER, RETURN, SETTLE or ALIGN_SHOT or SECOND, SECOND, WAIT or PARK_WAIT, PARK, DONE |
| `park_reason` | `reason` on the first PARK row: why the robot left for park (for example `Hive not ready for return shot`) |
| `clear_out_s` | first to last row whose `decision_inputs` has `clear_out_s_left=` (v33 on) |
| `spin_up_s` | `feed1_start` minus first row where `requested_launcher_tps > 0` |
| `up_wait_first_s`, `up_wait_s` | `feed1_start` minus FIRST start; `feed2_start` minus SECOND start (hive check plus settle) |
| `settle_align_s` | SETTLE or ALIGN_SHOT start to the next other phase |
| `spare_before_park_s` | `park_start` minus `feed2_end` (negative or blank means no slack) |
| `return_error_in`, `return_heading_deg` | distance from S of `x_in`, `y_in`, and `heading_deg` (-180..180), on the first SECOND row (S is 0,0,0) |
| `final_x_in`, `final_y_in` | pose on the first DONE row |
| `min_launcher_tps_feed1`, `min_launcher_tps_feed2` | lowest `measured_launcher_tps` during each volley |
| `dips_feed1`, `dips_feed2`, `dips_clear_out` | count of drops of `measured_launcher_tps` below 90 % of `requested_launcher_tps` that recover above 95 %, in each window. 10 rows/s can miss short dips: a lower bound, not a ball count |
| `last_dip_into_feed1_s`, `last_dip_into_feed2_s` | time of the last dip after its volley started |
| `battery_start_v`, `battery_min_v` | `battery_v` |
| `max_gap_s` | longest gap between consecutive trace rows (loop stalls) |
| `camera_status_last` | last `camera_status` |
| `trace_rows`, `trace_status` | row count; report `HUB trace=` value (`NONE (...)` means no trace) |
| `last_phase`, `notes_filled` | last phase seen; `yes` when any line of `notes.md` has an answer |

Without a trace (builds before v7) the hive columns come from the report's transition lines:
phase times and feed windows work; launcher, battery and camera columns stay blank.

For **collection practice** runs: `result` DONE or FAULT, `fault`, `done_at`, `last_phase`, `duration_s`
(to STOP); its timeline has each phase change and FAULT. For **Odo autos**: `result` from `RESULT`,
`warnings`, battery, `duration_s`; the timeline has each `PHASE`, `RESULT`, `WARNING` and `STOP`. For
**TeleOp**: `duration_s`, `r1_presses` (rising edges of `r1`), `rehearsals` (MARK_START rows),
`slowest_loop_ms` max, battery. For a **benchmark**: `result` (complete or not); `summary.md` names the
session whose `analysis/summary.md` has the analysis (not copied). **Camera checks**: build, time, STOP line.

## `timeline.csv` and `slim-trace.csv`

- `timeline.csv`: `t_s, phase, hive_state, feed, reason, x_in, y_in, heading_deg`, one row per report
  transition line. This is the run at a glance. Other kinds have their own columns (header row).
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

`runs/SUMMARY.md` (generated, overwritten each time) has run counts by kind and build, then one table
per opmode and build, LIVE hive runs only:

1. Runs, runs with both volleys, runs where the park deadline cut the second volley, faults, why it
   left for park.
2. Median and worst of `spin_up_s`, volley 1, outbound, collection, return, `up_wait_s`, volley 2,
   `spare_before_park_s`.
3. `last_dip_into_feed*_s` against the feed setting (is 3 s longer than needed?).
4. `return_error_in` spread; runs where SETTLE or ALIGN_SHOT took over 1 s.
5. From `notes.md`: balls left after volley 1 before and after v33, and any jam.
6. Secondary runs: `up_wait_s` for the first volley against whether volley 2 happened.

Then tell the mentor, in a few lines, which phase costs the most time and which runs lack notes.

## What the real downloads showed

2026-11-19, Claude local session, 21 sessions on the laptop's Desktop (`[tested]` against those files,
not on the robot). Corrections to the first draft of this file:

- `[tested]` `START settings=` lists **every** route setting, not only changed ones (v8-v29). Hence
  `settings_id` instead of `settings_changed`.
- `[tested]` On these downloads the Hub clock was right: `wall_ms` in Pacific time matched the
  `DATE-TIME` in TeleOp and auto file names to the second.
- `[tested]` Phase `WAIT` is the make-room park wait (`In the corner; park with 2.0 s left`); there is no
  `PARK_WAIT`. Phase `BLOCKED` also occurs.
- `[tested]` Builds v3-v6 write reports without traces, and v6 reports have no `opmode=` or `wall_ms=`.
- `[tested]` The same file can differ between sessions (a run downloaded while still being written,
  and every `*.properties`). The tool keeps the largest copy; `sources.csv` lists all of them.
- `[tested]` Collection practice CSVs have no `wall_ms`; their time comes from the file name.
- `[tested]` Scanning the generated `runs/` for network names, passphrases and serials found none.

What the first history says (`runs/SUMMARY.md`, `runs/index.csv`): 29 LIVE match-kind runs (v6-v29);
7 came back from collection, and 1 of those (v27 Primary, 2026-11-18) fired the second volley. Of the
other 6: 3 left for park with `Hive not ready for return shot`, 1 with `Return pose did not settle`,
2 were stopped by the driver. 6 LIVE runs faulted at START: 4 with no alliance chosen, 2 with the
Pinpoint moved since INIT. No `notes.md` is filled in yet and no v33 run has been downloaded yet.

- 2026-11-19, Codex local, `[tested]` on the laptop: all 2,421 recording copies in the 21 Desktop
  ZIPs match `sources.csv` by MD5. The same ZIPs contain 45 saved settings `.properties` files.
  `sources.csv` inventories recording copies only, so it has no rows for those settings snapshots.
- 2026-11-19, Codex local, `[tested]` on the laptop: the trace header omits `drainer_down` in the
  sampled v8–v20 builds. It includes the column in the sampled v26–v29 builds.
- 2026-11-19, Codex local, `[tested]` on the laptop: 12 v6 runs with a first-volley feed and no trace
  display `dips_feed1=0` in the index. Launcher velocity is unavailable for those runs, so the
  displayed zeros should be treated as missing measurements.
- 2026-11-19, Codex local, `[tested]` on the laptop: the generated summary for the one incomplete
  v10 benchmark points to `analysis/summary.md`. That file is absent from all 11 downloaded folders
  containing the benchmark recording.

## Options considered

| Option | Status | Why |
| --- | --- | --- |
| Index and summaries only | Possible | Smallest; loses the per-row data a cloud agent needs to check a timeline |
| **Index + timelines + slim hive traces + notes** | **In use** (2026-11-19) | Enough to recompute every metric above from Git; tens of KB per run |
| Full traces gzipped in Git | Not proposed | ~10x larger, mostly camera tags; the ZIP in Drive already has them |
| Git LFS or a private data repo for ZIPs | Mentor's call | Keeps raw data versioned; LFS on a public repo has bandwidth limits `[inferred]` |

## Gaps in today's logs (code changes to consider later)

- No ball count. A beam break or distance sensor at the agitator, or a faster launcher-speed log
  during volleys, would give per-ball timing; 10 rows/s is coarse for dips.
- Partner tip time is only visible through `hive_state` from our camera, and only while the robot
  checks the hive.
- The Hub clock can be wrong; `sources.csv` records the laptop time of each download (from the
  session folder name) as a cross-check.
- Emergency Odo Park, Emergency Timed Park, Calibration Launcher and Calibration Odometry record
  nothing (see [POLLEN_LOGGING.md](POLLEN_LOGGING.md)).
