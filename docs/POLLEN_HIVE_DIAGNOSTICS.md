**Current hardware tests run LIVE.** The DRY and DRIVE procedures below describe older builds.
For the current motor permissions and test order, use the [hive testing guide](POLLEN_HIVE_TESTING.md#fast-motor-testing-loop).

# v33 clear-out after each volley

Balls were left in the robot after the first volley, and collecting more then overloaded it. In the
Match autos and Hero, a finished volley now starts a clear-out: the agitator and launcher keep
running together for **Clear-out s after volley** (3 s) while the route continues. One time for both,
because the agitator meters balls into the launcher: a ball delivered after the launcher stopped could
jam. It is not a feed (the trace's `feed_requested` stays false; the decision column adds
`clear_out_s_left=`), it never touches the intake or wheels, it does not run in DRIVE, and the park
deadline cancels it. It costs no route time. Set the row to 0 for the v32 behavior. Calibration modes
and Out of the Way are unchanged. Field checks: count balls left in the robot at the garden or flower;
see whether balls fired while backing away land in the hive.

# v27 secondary startup: zero Pinpoint during INIT

On the Nov. 18 v26 red Secondary Shooter run, Pinpoint passed the READY check at START.
The next update reported `FAULT_BAD_READ` with the cached Y still at −215.26 inches, and the
auto stopped before requesting flywheel speed. The SDK rejects position jumps over 5,000 mm.
The START write to zero exceeded that threshold. Hive autos now reset position and recalibrate
the IMU during INIT while stationary. The SDK takes one new baseline reading for the intentional
reset, then resumes its usual read checks and verifies the origin. START reads the existing
pose. A Pinpoint fault still stops the auto.

# v9 garden route: keep the starting heading

Build **pollen-hive-v9** replaces the garden turns with a direct sideways approach.
From S: back clear of the end wall, strafe to intake alignment, drive front-first to collect,
back out, strafe back, and return to S. Launcher and agitator remain off during the garden.
Old saved nonzero transit/pickup headings are rejected; camera calibration is preserved.
The old turn-waypoint settings are retained for saved-file compatibility but no longer affect the path.
The CAD footprint checks cover the new straight corridor. First retest with an empty robot in
**Beta Test Primary Shooter + Garden / DRIVE**: garden, return, park. Then STOP and download.
Computer/geometry checks and Android tests/build passed on the local Mac. On 2026-11-15,
the operator reported that the empty v9 combined DRIVE trial on vinyl ran successfully.
Loaded pickup and hive-gated shooting with this route remain unverified.

# Hive auto diagnostics and next tests - v8

Base: `pollen-hive-ready`, commit `fbf82081b621bf6f0ff61041abec42d4dcdbef97` (v7 diagnostics).
New build: **pollen-hive-v8**. Legacy Odo match modes are unchanged.

## Collection no longer waits for flywheel spin-down

The team confirms that the agitator meters balls into the launcher and stops delivery when
commanded off. The previous assumption that collection required a stopped flywheel was incorrect
for this mechanism. v8 removes the COAST state, 100 ticks/s threshold, 0.5 s minimum wait and 2 s
timeout. A primary shot now transitions directly to the garden when the remaining-time budget
permits. The transition tick commands launcher, intake and agitator off; the next tick starts the
route. The intake runs on collection legs; the launcher and agitator remain commanded off throughout
the garden. No new delay or agitator-speed gate replaces the removed flywheel gate.

The measured launcher-speed requirement for deliberately SHOOTING remains, as do fresh UP evidence,
pose settling, odometry/reading validity checks, the 27 s cutoff, route checks and time reserved for
parking. Collection is never skipped because measured flywheel speed is still high. Too little
remaining time can still skip collection. Garden-only practice also starts without a coast wait.

Two mechanical edge cases are worth observing, without adding software delays: a ball already past
the agitator may complete its shot after the off command, and intake pressure or chassis motion may
move a queued ball if the stopped agitator is not a physical barrier. If the actual mechanism prevents
both, there is no reason to wait for flywheel spin-down. Motor mapping or wiring faults can also make
an off command act on the wrong device; v7's mapping records and unexpected-motion guard remain.

## What the previous runs mean

The completed primary runs that skipped the garden on the old coast timeout should no longer do so.
Their old CSVs/reports remain useful historical evidence. The analyzer still understands those old
COAST records. Run 2's physically unexpected movement is a separate unresolved issue: the controller
should command zero drive during FIRST. The garden-only driver STOP was intentional, not a route
failure. A completed garden-and-return run is still needed.

## Test order

Keep the successful camera mount/calibration, pipeline **4**, and for this primary start **Side RED**,
cell **34-37**. Check **pollen-hive-v8** in INIT. Every route starts at marked S, front toward the end
wall and launcher toward the hive, with Pinpoint READY. A person stays on STOP. Record test ID,
approximate start time, ball counts and whether any STOP was intentional. Video the full robot.

1. **Power-off mapping check.** Trace the four drive motor cables and launcher/intake/agitator to
   their Hub ports and compare to the active configuration. Software cannot detect swapped physical
   cables when the configured names and port assignments are otherwise distinct.
2. **Primary shot ONLY, DRY.** All motors must remain off. Then, if normal, two empty shot-only LIVE
   runs: mechanisms may run, but drive wheels must stay still. Any unexpected movement: STOP and
   download. The motion issue must be understood before full driving autos.
3. **Loaded shot-only, LIVE.** Three runs with a small counted batch, using target 1180 ticks/s,
   first feed 1.5 s and first-shot window 5 s. Observe whether any additional ball exits after the
   agitator stops, distinguishing a ball already committed to the flywheel from continued feeding.
   No five-second spin-down measurement is required. Note speed dips/feeding pauses if present.
4. **Garden + return, DRIVE.** With an empty robot, complete the whole route and return to S. No
   launcher, agitator or intake output in DRIVE. Check turns, clearance, return position and heading.
   Then repeat **LIVE** with a small counted group in the garden: intake collects; launcher and
   agitator stay off. Count collected and retained pollen.
5. **Beta Test Primary Shooter + Garden, LIVE.** Only once unexpected motion is resolved and the
   isolated tests pass. Check effective INIT settings again (practice and full modes share saved
   route values; Execution is never saved). With the cell UP throughout, expect first shot, direct
   garden departure, collection, return, second shot and park. Since v33 the clear-out fires leftover
   balls during the first 3 s of travel; after that, confirm no unwanted balls exit during
   travel/collection. Do not increase feed duration/window yet; keep the time budget comparable.
6. **Hive-state checks.** With the cell DOWN on return, no second feed while DOWN; it may wait until
   the parking reserve requires departure. With the cell DOWN from START, the primary still feeds
   the preload once pose and launcher speed are ready, then checks the garden budget. Nobody enters
   the robot route to tip it.

## Download after each batch or any unexplained movement

After driver STOP, from the repo root on Hub Wi-Fi:

```bash
bash tools/hive-benchmark-download.sh "$HOME/Desktop/auto-session-$(date +%Y%m%d-%H%M%S)"
```

Upload the ZIP, run notes and video of unexpected movement. Pipeline exports are needed only if
changed. The archive includes settings, reports, trace CSVs, available configuration XML, SDK rotated
logs/match logs, logcat, installed-app info, checkout information and storage usage. The XML files
show available configurations; the run's MAP lines identify the devices the OpMode actually obtained.

If adb is offline: rejoin Hub Wi-Fi; `adb disconnect 192.168.43.1:5555`,
`adb connect 192.168.43.1:5555`, then `adb devices -l`. Proceed only when the Hub says **device**.

## What is recorded and what it proves

- Motor-controller ports/directions/modes, requested/applied commands, SDK power readbacks, wheel
  encoder positions/velocities, launcher speed/current and battery voltage.
- Raw and route pose, odometry readiness, camera status and individual tag positions/heights.
- Evaluated phase, phase elapsed time, accumulated/requested feed time, gate results, remaining time,
  garden budget and parking reserve. Motion nodes add targets, errors and time without progress.
- START effective settings and unsaved counts, state/feed transitions, faults and intentional STOP.

`decision_inputs` describes the phase evaluated before transition; `phase` describes the resulting
state. In DRY, decision launcher speed is virtual; measured speed is physical. SDK commands/readbacks
are not independent proof of which physical motor moved. Compare them with encoders and video.
The pre-departure guard stops all outputs if displacement exceeds 2 in or wrapped heading exceeds
10 degrees; it cannot distinguish real movement from a localization fault. Later route motion uses
the existing path checks. Stop commands remain best effort if hardware communication fails.

Reports and matching traces live under `/sdcard/FIRST/pollen-hive`. Normal sampling is 10 Hz plus
phase/state/feed transitions, including DONE until STOP. Rows flush on transitions, STOP and about
once per second. Trace writes are capped at 2 MiB/run and a 64 MiB folder budget; creation requires
32 MiB free plus a 2 MiB reservation. A budget failure disables tracing visibly, without deleting
old evidence. This does not cap existing benchmark logs, small reports or other Android files.
Download and verify an archive before cleanup; preserve camera/primary/secondary properties.

Instrumentation adds sensor/file I/O; the 250 ms loop-stall check remains. A computer simulation
validates sequencing, not physical wiring, retention or pickup performance.
