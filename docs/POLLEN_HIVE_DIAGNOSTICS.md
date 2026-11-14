# Hive auto diagnostics and next tests - v7

Based on branch `pollen-hive-ready`, commit `c995222ec62bb8983197f3207426c1c7af8913b4`.
New build: `pollen-hive-v7`. These are experimental hive modes. Legacy Odo match modes are unchanged.

## Why wait for launcher speed before collection?

The intake also feeds the launcher. If it collects while the wheel is still spinning quickly,
it may feed newly collected pollen through the shooter. This is a collection/feed constraint;
it does not inherently require delaying chassis travel. A later optimization could travel with
the intake off while the launcher slows, then independently permit collection. We should measure
spin-down before changing that sequencing.

The current rule is **absolute measured speed <=100 ticks/s**, with at least **0.5 s** in COAST.
After **2 s**, an unsuccessful coast check skips the garden and parks. The threshold and timeout
are existing conservative settings, not validated physical limits. v7 leaves them unchanged.
With 28 encoder ticks/revolution, 100 ticks/s is about 214 RPM; “stopped” in the old message did
not mean exactly zero. Log actual speed and whether pollen can still exit at low speed before
relaxing this gate. Do not tune this by simply increasing the timeout: the complete cycle must
still fit in 27 seconds, including departure.

## What the first session establishes

- The two completed primary shooting runs skipped the garden on the coast timeout. They do not
  establish the wheel's actual decay curve: the old camera log stopped sampling speed in COAST.
- Run 2 physically moved and turned before planned departure, as observed by the team. The
  controller should command zero drive during FIRST. The old log lacks wheel commands,
  motor-port identities, encoder measurements and current, so it cannot establish the cause.
- The garden-only driver STOP was intentional. It is not a route failure or evidence that return
  is broken. A complete garden/return run is still needed.
- Some shot runs lost feed permission when launcher speed fell below the readiness fraction.
  That can lengthen or truncate a volley at the first-shot window. Do not change PIDF or speed
  before comparing unloaded and loaded traces.
- A mean tag height in the UP band does not guarantee readiness: all accepted tags must agree,
  odometry must be settled, and the camera evidence must be fresh. v7 shows the height range when
  accepted tags disagree or fall outside the bands; the classifier thresholds are unchanged.

## Vulnerabilities and evidence

| Question | New evidence / action |
| --- | --- |
| Were drive commands sent during shooting? | Route commands, applied FL/FR/BL/BR power commands, SDK power readback and drive encoders/velocities at 10 Hz plus state/feed transitions. |
| Does a mechanism name address a drive port? | Controller connection, port, direction, mode and motor type for all seven motors. Duplicate controller-port mappings block START. Physically swapped cables still need inspection. |
| Is the robot moving, or is localization moving? | Route pose and raw Pinpoint-localizer pose with readiness status; compare to a fixed-camera video and wheel encoders. Neither encoder values nor SDK power readbacks alone prove physical movement. |
| Why did feeding stop/start? | Measured and decision-input launcher speed, requested/applied outputs, pose settling, fresh hive state, per-tag positions/heights and camera status. |
| Why was garden/second shot skipped? | Evaluated phase, phase time, accumulated feed time, requested feed time, remaining time, garden estimate and park reserve at each decision. |
| Why did a route leg stall? | Node name, target, position/heading errors, elapsed node time, time without progress, motor outputs and odometry. |
| Did a power/configuration/reset problem occur? | Battery voltage, launcher current (when available), mapping report, saved settings, installed-app info, Robot Controller rotations, match logs and logcat. |
| Did an unsaved test setting disappear? | Effective START settings and unsaved counts. Execution always resets on INIT; shot tests and match modes share the corresponding saved route settings. |

New protective behavior: before the first commanded departure, a displacement **over 2 inches**
or wrapped heading error **over 10 degrees** stops all outputs and latches DONE. It does not park.
This catches a repeat of the reported motion, but cannot distinguish bad localization from actual
movement, does not diagnose wiring, and is not a substitute for driver STOP. Later path motion
continues to use the existing motion checks.

## Before installing

1. STOP the robot and download the current session with the existing downloader before an app
   restart can rotate additional evidence. Keep the ZIP already uploaded too.
2. Apply the supplied patch on the stated branch/base, run the tests, build and install using
   the instructions in the kit README. Check `pollen-hive-v7` on INIT.
3. Keep the successful camera mount/profile. Confirm pipeline **4**, current saved calibration,
   and for this primary position **Side RED**, cell **34-37**. Do not recalibrate just because
   this diagnostic build was installed. Use the existing calibrated height/tilt, not code defaults.
4. With robot power OFF, trace the seven motor cables to Hub ports. Compare the active robot
   configuration with the physical front-left/right, back-left/right, launcher, intake and agitator.
   Pay special attention to any wiring or active configuration change between runs 1 and 2.

## Next tests, in order

Every motor test starts at marked **S**, rear/launcher facing the hive, with Pinpoint READY.
Keep a person on STOP and a fixed video showing the whole robot, wheels and target. State the test
ID aloud at START. Reset placement between tests; never reset odometry in the middle of a run.
Record approximate start time, ball count, visible movement, outcome and whether STOP was intentional.

### A - Zero-output baseline

- Select **TEST Hive primary shot ONLY**. Confirm Execution **DRY**, Side **RED**, pipeline **4**.
- INIT with the robot stationary; confirm READY and cell 34-37. START; touch neither robot nor sticks.
- All seven motors must remain off. Let the run reach DONE, wait five seconds, then STOP.
- If any motor runs, STOP immediately and download. Do not proceed to LIVE. DRY can show virtual
  feed permission; that is not an applied motor command. Its trace records actual wheel speed too.

### B - Empty shot-only isolation and spin-down (two runs)

- Empty the robot. Keep the hive cell physically UP and the robot at S.
- Select **TEST Hive primary shot ONLY**, set Execution **LIVE**, first feed **1.5 s**, first-shot
  window **5 s**, launcher target **1180 ticks/s**, Side RED. Keep the established feed powers.
- START. The launcher and then feed mechanisms may run; the four drive wheels must remain still.
- On DONE, leave the OpMode running another **five seconds** before STOP. v7 continues measuring
  actual flywheel speed in DONE, so this measures decay without a subsequent drive route.
- Repeat once. Any wheel rotation, unexpected chassis motion or new mapping/odometry fault:
  STOP and download immediately; do not continue to garden. A timeout or no feed is also useful
  evidence; keep its trace rather than retrying until a successful run hides the problem.

### C - Loaded shot-only check (three runs)

- Only after B has no unexplained movement, use the same settings with **three pollen** each run.
  Shoot into the hive, with the cell restored UP before each run.
- Count loaded, expelled and remaining pollen. Video whether feed pauses coincide with speed dips,
  vibration or a blocked camera view. After DONE, again wait five seconds before STOP.
- Download A-C now. We can decide from these traces whether the 2 s coast wait is achievable,
  whether collection must be gated independently of travel, and whether speed sag needs tuning.
  Do not loosen speed/vision limits or increase the timeout merely to force a garden run.

### D - Complete garden route, without shooting

- After resolving any unexplained motion, select **TEST Hive garden + return**, Execution **DRIVE**,
  at S with no balls. This runs the path with intake/launcher/agitator off.
- Allow the entire route and return to finish; stop early only for a real problem. This replaces
  the intentionally interrupted test. Measure return error and heading relative to the tape at S.
- Pass: correct direction/turns, no contact with field objects, no timeout, return and settle at S.
  Download a failure immediately. Do not compensate for wrong motion by changing camera settings.
- Then repeat with Execution **LIVE** and a small counted group of pollen in the garden. Launcher
  and agitator stay off; the intake runs on collection legs. Count collected and retained pollen.

### E - Full primary cycle

Only after A-D pass and coast behavior is understood. Confirm the effective INIT values again:
Execution LIVE; Side RED; first feed 1.5 s; second feed 1.5 s; first window 5 s; target 1180 ticks/s;
current camera calibration. Square saves route/camera changes for the full mode, but not Execution.
Select **EXPERIMENTAL Hive Primary + Garden**. Keep its marked path clear.

1. Cell UP initially and still UP on return: first shot, acceptable coast, garden/return, second
   shot, park. A garden skip is not a collection pass; download its decision trace.
2. Cell UP for the first shot, then DOWN when the robot returns: no second feed while DOWN; the
   robot waits subject to the remaining-time reserve, then parks. Do not expect an immediate park
   solely because the cell is DOWN. Nobody should enter the robot's route to change hive state.
3. Cell DOWN from START: no shot; the first-shot window ends and the primary parks without garden.

## Download after each batch or any unexplained movement

Run from the repository root on the Mac, while connected to Hub Wi-Fi, after driver STOP:

```bash
bash tools/hive-benchmark-download.sh "$HOME/Desktop/auto-session-$(date +%Y%m%d-%H%M%S)"
```

Upload the resulting ZIP, the fixed-camera video of any unexpected movement and the short run
notes. No manual transcription of screen numbers is necessary. Include pipeline exports only
if changed. Photograph the active motor configuration and cable labels if motion remains unexplained.
The downloaded XML files list available configurations; they do not by themselves prove which was
active. The run's MAP records describe the devices this OpMode actually obtained.

If adb is offline, rejoin Hub Wi-Fi, then `adb disconnect 192.168.43.1:5555`,
`adb connect 192.168.43.1:5555`, and `adb devices -l`. Proceed only when it says **device**.
The downloader now prints its stage and finds the usual Mac SDK adb even when an interactive alias
is not inherited. It preserves the mandatory hive folder even if optional SDK logs are missing.

## Files, limits and interpretation

Each motor-mode run produces a compact `reports/*.txt` and matching `traces/*.txt.csv` under
`/sdcard/FIRST/pollen-hive`. Reports include settings, maps and transitions. Traces include all phases,
COAST and DONE. CSV columns `decision_inputs` describe the phase evaluated **before** a transition;
`phase` describes the resulting state. In DRY, `decision_launcher_tps` is virtual; the measured
column is physical. Commands describe SDK writes, not a guarantee that hardware accepted them.
After an output exception, FAULT/zero is best effort: readbacks and SDK errors matter.

Trace rows flush at transitions, STOP and about once per second; power loss can still lose the
last buffered data. Normal sampling is 10 Hz, with extra rows for phase/state/feed transitions.
New traces are limited to 2 MiB/run and a 64 MiB folder budget; creation also requires 32 MiB free
plus the 2 MiB reservation. A full budget disables tracing visibly rather than deleting evidence.
This is a trace budget, not a quota for every Android file: existing benchmark logs, small text
reports and other apps still consume disk. The ZIP includes `storage.txt`. Download and verify an
archive before any cleanup; do not delete `camera.properties`, `primary.properties` or
`secondary.properties`. No cleanup runs automatically.

Instrumentation adds sensor reads and file I/O. The existing 250 ms loop-stall stop still applies;
check trace timing and SDK logs on the first runs. These tests can establish software decisions,
not prove wiring, physical coast limits or a safe collection threshold without robot observation.
