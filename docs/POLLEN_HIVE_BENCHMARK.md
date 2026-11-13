# Hive pipeline benchmark: collect evidence with four button presses

Build **pollen-hive-v4**. Start here when tag detection is intermittent. This adds a camera-only
experiment; it does not change the autonomous routes, calibration thresholds, or shooting gate.
The earlier calibration and practice instructions remain in [POLLEN_HIVE_TESTING.md](POLLEN_HIVE_TESTING.md).

## What to prepare tonight

Apply the accompanying mail patch on a new branch from the pushed `pollen-hive-ready` commit
`ab7d23b72feb98eedde68fe3a80276f1f0baf4db`. Run the computer tests and build the APK. You do not
need the camera for that. Install when the Control Hub is available. Check **pollen-hive-v4**.
Print [the benchmark card](drive-team/print/5-hive-benchmark.pdf). Bring the Mac, normal Limelight
data cable, and measured lens height. Have Python 3 and adb available before practice.

The new Driver Hub mode is **TEST Hive pipeline benchmark**. It maps only `limelight`: no motors,
launcher or odometry. You can use it before calibration passes. An unset tilt produces UNKNOWN
readiness, but raw detection and camera-space poses are still useful and are saved.

## Can software change camera settings?

Yes. The FTC 12 SDK provides `pipelineSwitch(index)` and `uploadPipeline(json, index)`.
Limelight's REST API on port 5807 also accepts individual setting updates at
`POST /update-pipeline`; `flush=1` persists them. `GET /pipeline-atindex?index=N` reads the saved
configuration, and `/upload-pipeline?index=N` replaces a saved slot.

**This patch automatically switches saved slots, rather than editing their settings during a run.**
That gives each trial a reproducible configuration and avoids relying on guessed setting keys or
resolution enum values before we have this camera's exported JSON and firmware version. It does
not upload pipelines, change exposure, or save new camera settings. Prepare the variants once in
the web UI tomorrow; all switching and measurement after that is automatic. Once we have the
exports we can automate creating further variants, including live overrides if useful.

Do not use an FRC NetworkTables example as an FTC Java API. Do not update firmware during this
experiment just to gain a feature. REST documentation describes current firmware; use the UI
export fallback if an endpoint is unavailable on your installed version.

## Tomorrow: 5-minute smoke test before the full experiment

1. Rear camera faces our near hive cell from the actual shooting position. Keep mounting height,
   aim, robot position and lighting fixed. Confirm the printed IDs of that cell once.
2. Install v4. In TEST Hive camera INIT, verify/save your measured lens height (14 in is only the default).
   Select **TEST Hive pipeline benchmark**. INIT defaults to the saved production
   pipeline only. Set **Passes = 1**, **Seconds = 5**, and **Our cell first ID** correctly.
3. START. Put our cell physically UP, wait for it to stop rocking, tap **D-pad LEFT**. Do not move
   it until the screen says WAIT. Then put the same cell DOWN, let it settle, tap **RIGHT**.
4. At DONE press STOP. Download using the command below. Verify a summary and raw `.jsonl` exist.
   If log writing fails the benchmark stops with ERROR. Fix that before spending time on trials.
5. A wrong pipeline/Full 3-D/decode error in telemetry is a setup issue, not a reason to lower
   readiness thresholds. A trial with zero new frames cannot pass by showing an empty hit rate.

| Actual near cell | First ID to select |
| --- | --- |
| RED audience-side (primary start) | 34 |
| RED scoring-side (secondary start) | 30 |
| BLUE scoring-side (primary start) | 42 |
| BLUE audience-side (secondary start) | 38 |

The first-ID selection is deliberately explicit. We need known physical truth to judge the detector;
letting the detector choose the label would hide wrong-cell errors.

## Prepare comparable pipeline variants once

With the camera on the Mac's normal data connection, open `http://limelight.local:5801` (or the
camera's known address). Stop any robot mode that is switching pipelines. Before editing, export:

```bash
python3 tools/hive-pipeline-backup.py --out "$HOME/Desktop/hive-pipelines-before"
```

The helper is read-only and uses port 5807. `--host CAMERA_IP` is available. It saves all ten saved
pipelines plus available status, calibration and hardware metadata. It refuses an existing output
folder. The Mac must reach the **camera**, not merely the Control Hub Wi-Fi. Direct Mac connection
is the simplest route. If unavailable, use the download button beside each pipeline name and keep
the files. These exports are separate from the Hub logs; we need both.

Keep slot 0 as the working baseline. Back up any occupied slots before replacing them. Download
baseline and upload copies into spare slots using the web UI; change one variable per copy. The
benchmark runs a contiguous slot range; select the actual range in INIT if you use different slots.

| Slot / suggested name | Change from baseline | Purpose |
| --- | --- | --- |
| 0 `hive-baseline` | None | Reproduce current behavior |
| 1 `hive-exposure-short` | Fixed exposure about half baseline, within UI limits | Test blur resilience / darker image |
| 2 `hive-exposure-long` | Fixed exposure about twice baseline, within UI limits | Test whether light limits detection |
| 3 `hive-downscale-low` | Lower detector downscale; e.g. 2 to 1, if available | Give small/distant tags more detector pixels |
| 4 `hive-resolution-high` | Next higher supported capture resolution; same downscale | Test range and pose stability vs frame rate |

These are experiment suggestions, not calibrated defaults. If already at minimum downscale or
maximum resolution, omit that variant or replace it with a modest gain change and give it an
accurate name. If exposure is automatic, make a documented fixed-exposure baseline first so the
short/long variants are interpretable. Retain the original export. Use the UI's displayed units;
do not assume an old documentation example uses the same units as your firmware.

Every profile: **AprilTag 36h11, marker size 82.55 mm, Full 3-D on, Single target grouping**, same
ID filters, crop and camera geometry. Initially keep all hive tags available; do not crop out the
DOWN position or change filters between trials. These are BIOBUZZ tags, not the larger FRC tags
in some vendor examples. Keep source on **Camera**, not Snapshot. If you enabled **Ignore
NetworkTables Index** while editing, clear it before testing SDK switching.

Save changes, then run:

```bash
python3 tools/hive-pipeline-backup.py --out "$HOME/Desktop/hive-pipelines-tested"
```

Retain that export with the run. It records what the pipeline numbers mean without transcribing
settings. If you edit a profile again, use a new export folder and a new benchmark run. The saved
pipeline endpoint reads disk; transient unsaved REST overrides are not represented there. Do not
make those during this comparison. Move the camera back to the Hub and leave the web stream closed
for consistent load. The patch verifies reported pipeline IDs on each result.

## Automatic comparison: about 7 minutes 20 seconds for five profiles

INIT: **First = 0, Last = 4, Passes = 2, Seconds = 20**, correct cell, Scenario = normal. D-pad
up/down selects a row; left/right edits. Settings are local to this experiment; they are not saved
over the robot's calibration. START waits; it does not start a countdown you have to race.

1. Put our near cell **UP**. Let it settle. Tap **LEFT** once. Hold still while every pipeline runs.
2. At WAIT, tip that same cell **DOWN**. Let it settle. Tap **RIGHT** once. Hold still.
3. At the next WAIT repeat **UP / LEFT**, then **DOWN / RIGHT**.
4. At DONE press STOP. The next pass reverses pipeline order to reduce simple order effects.

Each trial excludes 2 seconds after switching, then records 20 seconds, starting with a fresh
readiness detector. Opposite-direction buttons are ignored at WAIT; label presses during a trial
are ignored. **Circle cancels a run if you moved the hive early or gave the wrong label.** STOP and
restart; the file is marked incomplete. Do not relabel a moving hive as already settled.

The mode returns the camera to the pipeline in the saved robot camera settings at DONE/STOP. It
reports a failed restore; if that happens select the saved pipeline manually before another mode.
It never promotes the winning settings or changes shot permission automatically.

For a faster screening run, use one pass and 10 seconds/profile (2 minutes for five). Then spend
longer on the best two. Compare usable frames/second and readiness, not hit percentage alone: a
slow high-resolution pipeline may find more tags per frame but still provide fewer useful updates.

## Download one bundle and send it back

From the repo root, after STOP, join the Control Hub Wi-Fi on the Mac. Use a new folder for every
pull; existing logs on the Hub are not deleted.

```bash
bash tools/hive-benchmark-download.sh "$HOME/Desktop/hive-session-01"
```

Default adb device is `192.168.43.1:5555`. Supply a second argument for a different device. The
script pulls `/sdcard/FIRST/pollen-hive`, saves available logcat and checkout identity, runs the
analyzer, and creates **hive-session-01.zip**. Upload that ZIP **and the tested pipeline export
folder/files**. No need to read or transcribe the telemetry. Pipeline exports need a separate
camera connection only when preparing/changing profiles, not for every run.

If a download was interrupted, use a new output folder. To analyze existing files again:

```bash
python3 tools/hive-benchmark-analyze.py "$HOME/Desktop/hive-session-01/pollen-hive" \
  --out "$HOME/Desktop/hive-analysis-02"
```

The raw JSONL keeps every sampled SDK result (including `v=0`, rejected, empty and repeated
results), camera timestamps, SDK age, latency fields, all reported tags/poses/quality fields,
requested and returned pipeline, physical label, elapsed time, saved mount values, decoded
heights/sideways positions and production state/reason. The log is flushed every second and at
trial boundaries. A crash may lose the last buffered portion; partial files are flagged.

Polling is capped at 50 Hz to match the existing camera adapter. **This measures frames observed
by the robot, not every sensor frame the camera produced.** The report separates fresh-frame hit
rate from useful frame rate and long dropouts. No video is captured; a single photo of the mount
and view is useful if the numeric results remain puzzling.

The CSV reports each trial separately: raw correct-cell hit %, usable %, fresh/usable FPS,
longest no-usable-frame gap (including start/end), sampled ready-time %, first-ready delay, DOWN
false-UP polls, height mean/SD, rejected polls and loop gaps. Ready-time is conservatively sampled:
a sample covers at most 50 ms, so long code stalls do not look like continuous successful tracking.
Here "usable" means decoded poses for our cell inside the sideways gate; it does not mean all
tags agree on UP or that readiness was granted. An incomplete run is not validation; height SD pools tags and is not a per-tag accuracy estimate.

## Spend the remaining hive time on the best candidates

- First choose candidates that improve **UP usable FPS and readiness availability**, with short
  gaps. Any confirmed UP while the same cell is physically settled DOWN is a failure to investigate.
- Re-run the winner at the secondary shooting start, selecting its correct cell and Scenario.
  Test modest left/right placement changes with the corresponding Scenario values. Keep each run
  fixed; do not move during a block. Use First = Last to select a single candidate.
- Finish calibration using TEST Hive camera and the selected pipeline. Change the camera pipeline
  row there and save only after checks pass. Revalidate geometry if resolution/crop changes.
- Use the existing continuous camera check for deliberate tipping/occlusion transitions before
  shooting tests. Static labels cannot automatically establish the exact instant a moving basket
  becomes shootable. This benchmark does not certify transition behavior or feeder timing.
- Once detection is reliable, the raw logs let us compare alternative confirmation rules offline.
  Do not lower the production gate just to make calibration pass. The present gate requires four
  fresh agreeing frames **and** at least 0.15 seconds of agreement, with no intervening unknown;
  high frame rates therefore need more than four successive hits. Calibration's hold-rate test
  is a separate condition.

No access to the hive again? Keep the raw logs, exports and measurements. They let us distinguish
optical misses from `v=0`, wrong IDs, pose/latency rejection, sideways filtering and confirmation
resets without asking Lev to repeat dozens of screen observations. Logs cannot test a camera
setting that was never tried; capture the meaningful variants while the field is available.

## Implementation / checks

`PollenHiveBenchmarkPlan` is a small hardware-independent schedule; `PollenHiveBenchmark` handles
SDK results and buffered logging. Existing `PollenHiveCamera.decode` and `IsHiveReadyForShot` are
used unchanged. The test never maps or commands a motor. `tools/test-pollen-sequence.sh` includes
schedule and analyzer edge cases; JUnit includes the Java schedule test.

Before first robot use, run a full Android build and the short log-download smoke test. Desktop
SDK compilation cannot confirm the camera's installed firmware, USB delivery, storage permissions
or real field performance.

Sources checked for this workflow:
- [Limelight REST API](https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api)
- [FTC Java API guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
- [AprilTag tuning](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags)
- [Pipeline setup and export](https://docs.limelightvision.io/docs/docs-limelight/getting-started/pipelines)
