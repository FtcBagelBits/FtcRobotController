# Agent and robot loop: observing the robot and iterating faster

Question: Can a Mac act as the Robot Controller so a coding agent can observe and interact
with the robot directly? What other ways of working would make each try-it-on-the-robot loop
shorter?

Started: 2026-11-14 by a Claude cloud session, at Ari's request. Based on branch
`pollen-hive-ready` at `84d8eaf` (build label `pollen-hive-v8`).

**Where this ran:** a cloud session. It read this repository and public documentation. It did
not reach the laptop, the Control Hub, the Driver Hub or the camera. Nothing below marked
`[doc]` or `[inferred]` has been tried on our hardware.

Labels (`[doc]`, `[repo]`, `[reported]`, `[inferred]`, `[tested]`) and the rules for editing
this note are in the [folder README](README.md). This note does not change any rule in
[AGENTS.md](../../AGENTS.md).

## Summary

- A Mac cannot usefully be the Robot Controller. Stop looking for that.
- The useful goal is different: let an agent on the Mac **see** what the robot does while it
  runs, and, for OpModes that move nothing, **start and stop** them.
- The laptop already has robot Wi-Fi and internet at the same time through a phone on USB
  (N1, in use). That was the biggest single blocker and it is solved.
- Cheapest next steps, in order: live log watching (O1), a ten-minute test of the Panels
  Limelight proxy (O3), reading the Driver Hub screen over USB (O4).
- Letting an agent press INIT/START (A1) needs a mentor decision and an AGENTS.md change
  first. Today's rule forbids it.

## What is known

### A Mac cannot be the Robot Controller

- `[doc]` A Control Hub contains two boards: an Android controller, and an Expansion Hub board
  that talks to the motors, servos and sensors. The Robot Controller app runs on the Android
  board. (REV Hardware Client documentation, source 1.)
- `[inferred]` The Mac has no connection to that inner Expansion Hub board. The Control Hub's
  USB-C port and Wi-Fi reach the Android board (ADB, web pages), not the motor board. An
  Android emulator on the Mac could run the app but would have no hardware behind it.
- `[doc]` REV's PC tool for driving a hub directly, REV Hub Interface, works only with a
  separate Expansion Hub and not with a Control Hub (source 2). A community edition runs on
  macOS (source 3). It is manual control of motors and sensors with no code, so it would not
  run our OpModes.
- `[inferred]` Even if it could be made to work, the robot at a match must run from the
  Control Hub, so code developed against a Mac stand-in would be tested on the wrong system.

### What the tools did at the time of the original review

`[repo]` Update (November 18, 2026, local Mac checkout). [AGENTS.md](../../AGENTS.md)
now treats an install or deploy request as authorization to proceed without checking OpMode status.

- `[repo]` `tools/ftc.sh` covers doctor, connect, test, build, deploy, install, logs and patch.
  "No command starts an OpMode." (`tools/ftc.sh`, `docs/DESKTOP_TOOLS.md`)
- `[repo]` AGENTS.md: "Scripts never press INIT/START or drive/feed/shoot. Physical tests are
  run by the operator." Install only after the operator has stopped the OpMode.
- `[repo]` Logs are collected after STOP. `tools/hive-benchmark-download.sh` takes a one-time
  `logcat -d` dump; nothing streams logs while an OpMode runs.
- `[repo]` Robot log tags in the code: `PollenHive` and `PollenAuto` (`PollenAutoLog.TAG`).
- `[repo]` Panels is already a dependency (`com.bylazar:fullpanels:1.0.12` in
  `TeamCode/build.gradle`), served at `http://192.168.43.1:8001`. Only
  `PedroLocalizationTest.java` uses it.
- `[repo]` The camera tools (`tools/hive-pipeline-backup.py`, `tools/hive-camera-record.py`)
  read the Limelight's HTTP API on port **5807** and assume the Mac can reach the camera
  itself (`limelight.local`). `docs/POLLEN_HIVE_BENCHMARK.md` says the Mac must reach the
  camera, "not merely the Control Hub Wi-Fi".
- `[repo]` Two hive OpModes map no motors: `TEST Hive camera (NO MOTORS)`
  (`PollenHiveCheck.java`) and `TEST Hive pipeline benchmark` (`PollenHiveBenchmark.java`,
  which maps only `limelight`). Every other hive OpMode can move the robot or shoot.
- `[repo]` The Dairy Maven repository is already listed in `TeamCode/build.gradle`.

### Where the time goes in one loop

`[inferred]` from the documented procedure, not measured: edit, build, operator stops the
OpMode, install (the Robot Controller app restarts), operator checks the Build line on the
Driver Hub, operator runs the OpMode, operator stops it, download logs, analyze. The agent is
blind between install and download, and a person is needed at four of those steps. Nobody has
timed these steps. Timing one real loop would show which option below is worth the most.

## Options

Status values: `Proposed`, `Trying`, `In use`, `Deferred`, `Rejected`.

| ID | Option | Status | Strongest evidence | Next step |
| --- | --- | --- | --- | --- |
| N1 | Robot Wi-Fi and internet at the same time (phone on USB) | In use | `[reported]` | Write the setup into Desktop tools |
| N2 | ADB to the Control Hub over a USB cable | Proposed, low priority | `[doc]` | Try a USB-A to USB-C cable once |
| O1 | Watch robot logs live while an OpMode runs | Proposed | `[repo]` | Add a read-only `watch` command |
| O2 | Panels telemetry and graphs for people | Proposed | `[doc]` | Decide whether hive OpModes should send to Panels |
| O3 | Reach the Limelight through the Control Hub (Panels Limelight proxy) | Proposed | `[doc]` | Ten-minute test on the robot |
| O4 | Read the Driver Hub screen from the Mac over USB | Proposed | `[inferred]` | Plug in, run two commands |
| O5 | A camera on a tripod that the agent can take pictures from | Proposed | `[inferred]` | Try once during a practice |
| A1 | Agent starts and stops OpModes that move nothing | Proposed, **blocked by AGENTS.md** | `[inferred]` | Mentor decision |
| A2 | Agent starts OpModes that drive or shoot | Rejected | n/a | None |
| L1 | Panels configurables for live tuning | Proposed, low priority | `[doc]` | Compare with the INIT menus first |
| L2 | Replay recorded robot logs through the logic in computer tests | Proposed | `[repo]` | Pick one small real session as a fixture |
| L3 | Commit the simulated-OpMode test harness | Proposed | `[reported]` | Port it into `TeamCode/src/test` |
| L4 | Hot reload with Sloth | Deferred | `[doc]` | Revisit after the next meet |
| P1 | One agent writes, the other reviews before deploy | Proposed | `[inferred]` | Try for one session |
| P2 | One experiment file per robot session | Proposed | `[inferred]` | Try for one session |

### N1. Robot Wi-Fi and internet at the same time

- `[reported]` Ari, 2026-11-15: the laptop already gets internet from a phone on USB while its
  Wi-Fi is joined to the robot.
- Why it matters `[inferred]`: Claude and Codex need the internet to think. Without this, an
  agent is offline whenever the laptop is on the robot's Wi-Fi, which is exactly when the robot
  can be reached.
- Follow-up: `docs/DESKTOP_TOOLS.md` ("Build on internet Wi-Fi, install on robot Wi-Fi") and
  `docs/ROBOT_CONNECTION.md` ("Your laptop has no internet now; that's expected") still
  describe switching networks. Someone at the laptop should write down the setup that works:
  which phone connection, the order of network services on the Mac, whether a VPN must be off,
  and that `bash tools/ftc.sh connect` and the agent both work at the same moment. A cloud
  session cannot check any of that, so it was not written here.

### N2. ADB to the Control Hub over a USB cable

- `[repo]` "Our team has never got a USB-C cable to the Control Hub working"
  (`docs/ROBOT_CONNECTION.md`).
- `[doc]` REV's instructions say to connect a Control Hub to a PC with a USB-A to USB-C cable
  (source 4).
- `[inferred]` If the failed attempts used a USB-C to USB-C cable from a Mac, the cable type
  may be the cause. Not verified, and it is not known which cables were tried.
- Value now is small because N1 works. It would let Wi-Fi stay on the internet and would allow
  `adb forward` of the Panels port, but a cable on a driving robot is a snag hazard.

### O1. Watch robot logs live

- Idea: a `bash tools/ftc.sh watch` command that runs `adb logcat` for the selected hub,
  filtered to `PollenHive` and `PollenAuto`, and writes to a new file that an agent can read
  while the OpMode runs.
- `[inferred]` Read-only: it does not start, stop or install anything, so it fits today's
  AGENTS.md rules. It must use the explicit serial like every other tool, must not clear the
  hub's log buffer, and must stop on its own after a time limit.
- `[inferred]` It lets the agent see `HIVE` per-frame lines and auto decisions during the run
  instead of after a download, and a tester can ask "what just happened?" right away.
- Needs a fake-ADB test in `tools/test-ftc-tools.py` like the other commands.

### O2. Panels telemetry and graphs

- `[doc]` Panels offers OpMode control, live configurables, a graph view, a capture
  (record and replay) feature and Limelight support (sources 5 and 6).
- `[inferred]` For people, graphs of launcher speed or tag height during a run may be easier
  than reading logs afterward. For agents, Panels is a web page: no documented way for a script
  to talk to it was found in this research, so an agent would have to drive a browser. That is
  slow and fragile compared with O1.
- Cost: every OpMode that should appear needs Panels telemetry calls. Keep the plain Driver
  Station telemetry as the main path so students are not confused.

### O3. Reach the Limelight through the Control Hub

- `[doc]` Panels has a Limelight Proxy plugin. Its documentation says the Limelight's own
  dashboard can be opened on port **5801** at the Robot Controller's address, and describes
  the plugin as a separate jar plus an OkHttp dependency
  (`com.squareup.okhttp3:okhttp:4.10.0`) (source 7).
- Unknown: whether `fullpanels:1.0.12` already includes that plugin. The cloud session could
  not reach the Panels Maven server. A local session can check with
  `bash tools/ftc.sh gradle :TeamCode:dependencies` and search the output for `limelight`.
- Unknown: whether the proxy also forwards port **5807**. Our scripts need 5807; the
  documentation only mentions 5801.
- Test (about ten minutes, moves nothing): robot on, laptop on robot Wi-Fi, then open
  `http://192.168.43.1:5801` in a browser. If the Limelight page appears, also try
  `python3 tools/hive-pipeline-backup.py --host 192.168.43.1 --out NEW_FOLDER`. Record both
  results here with the `[tested]` label.
- If it works, pipeline backups and camera checks no longer need the camera's own cable on the
  Mac, and `docs/POLLEN_HIVE_BENCHMARK.md` should say so.
- Rule to keep `[repo]`: the backup tool is read-only. Do not let an agent change camera
  pipelines through the proxy without the mentor asking for it.

### O4. Read the Driver Hub screen from the Mac

- `[doc]` The REV Driver Hub is an Android device that REV Hardware Client supports, along
  with generic Android devices over ADB (source 1).
- `[inferred]` With the Driver Hub on a USB cable to the Mac,
  `adb -s DRIVER_HUB_SERIAL exec-out screencap -p > screen.png` should save a picture of the
  Driver Station screen that an agent can read. That would let the agent confirm the Build
  line after an install, read telemetry, and see error banners. Not tried: the Driver Hub may
  need USB debugging switched on, and it is not known whether the cable affects its Wi-Fi link
  to the robot.
- `[repo]` Two ADB devices would then be attached. The tools already require an explicit
  serial for every device command, which is what makes this safe. A second variable (for
  example `FTC_DRIVER_SERIAL`) would be needed; never reuse `FTC_ROBOT_SERIAL` for it.
- Reading the screen is observation only and fits today's rules. Tapping the screen is A1.

### O5. A camera on a tripod

- `[inferred]` A webcam or phone pointed at the field, connected to the Mac, lets an agent
  save a still or short clip (for example with `ffmpeg`) and look at it. Useful for "what did
  the robot physically do?" questions, such as the unexplained Q6 run on 2026-11-07.
- `[inferred]` Not a measuring tool. Positions and timing should come from odometry and the
  logs. Record the clock time of each clip so it can be matched to the logs.

### A1. Agent starts and stops OpModes that move nothing

- **Blocked.** AGENTS.md says scripts never press INIT/START. Do not build or use this until a
  mentor decides and AGENTS.md is changed.
- Why consider it `[inferred]`: with A1, an agent could run the whole camera benchmark loop
  (install, run, download, analyze, adjust, repeat) while people work on something else.
- Candidates `[repo]`: only `TEST Hive camera (NO MOTORS)` and `TEST Hive pipeline benchmark`.
  If allowed, the allowed names should be an explicit list in the tool, checked against the
  code so an OpMode that gains a motor drops off the list.
- Ways to do it, none tried:
  1. Through the Driver Hub with `adb shell input tap` (O4 first). Uses the real Driver
     Station, and the physical STOP button stays in a person's reach. Fragile: screen positions
     can change with app versions.
  2. Through Panels' OpMode control `[doc]` (source 5) by driving a browser. No script
     interface was found.
  3. A small endpoint of our own inside TeamCode. Most code, most to maintain, and hardest for
     students to understand. Not recommended now.
- `[inferred]` The benchmark has INIT menu choices. The team's rule that INIT defaults must be
  correct without edits is what would make an unattended start sensible.
- Open safety questions for the mentor: must a person be in the room; who stops it if the
  agent hangs; does a camera-only OpMode started this way behave the same when the Driver Hub
  is not connected.

### A2. Agent starts OpModes that drive or shoot

- Rejected by the cloud session that wrote this, and not requested by anyone. A person presses
  START with a hand on STOP. Listed so that nobody proposes it by accident as "the next step
  after A1".

### L1. Panels configurables for live tuning

- `[doc]` Panels configurables change values while the robot runs, without installing again
  (sources 5 and 6).
- `[repo]` We already have INIT menus with saved settings and `tools/hive-promote-defaults.py`
  to move measured settings into code. AGENTS.md says to treat launcher speeds, PIDF gains and
  calibration as measured data and to call out changes.
- `[inferred]` So configurables partly repeat what the INIT menus do. Possible value is for
  values that must change during a run, not between runs. Any value tuned this way still has to
  go through the same promote-and-record step. Low priority.

### L2. Replay recorded logs in computer tests

- `[repo]` `PollenHiveDiagnosticsTest` replays hand-written cases of observed risks. The Hub
  also writes raw JSONL logs under `/sdcard/FIRST/pollen-hive`.
- Idea: keep a few small real recordings as test fixtures and feed the recorded camera frames
  through `IsHiveReadyForShot` and the other decision classes in a computer test. Every field
  session then becomes a permanent check, and an agent can try a rule change against real data
  with no robot.
- Care: raw session logs are large (the benchmark guard is 500 MiB). Fixtures must be trimmed
  to a few seconds each, and must record which build produced them.

### L3. Commit the simulated-OpMode test harness

- `[reported]` (Claude cloud sessions, project handoff notes): the hive patches were checked
  with a harness that is rebuilt in each cloud session and is not in the repository. It has
  stand-ins for `OpMode`, `Gamepad`, `Telemetry`, `HardwareMap`, `ElapsedTime` (a simulated
  clock) and `Limelight3A`/`LLResult` (a simulated camera with a frame rate and a detection
  chance per pipeline), and a driver that presses buttons by reading telemetry.
- `[inferred]` Now that local agents can run Gradle, this belongs in `TeamCode/src/test` so
  every agent runs the same checks and nobody rebuilds it. It tests menus, timing and log
  writing. It is not a robot test.

### L4. Hot reload with Sloth

- `[doc]` Sloth reloads TeamCode classes without reinstalling or restarting the Robot
  Controller app. It reloads only classes under `org.firstinspires.ftc.teamcode` (source 8).
- `[doc]` Panels has separate Sloth builds that cannot be mixed with the normal ones
  (source 9), and the Sloth documentation warns that projects started from the Pedro
  quickstart are a common source of setup problems (source 10).
- `[doc]` An older tool, FTC Fast Load, reported installs dropping from 89 s to 27 s on SDK 8.2
  (source 11). Not evaluated against SDK 12.
- `[inferred]` Deferred because: we use both Pedro and Panels, so the build would change in
  several places mid-season; "what is installed on the robot" becomes harder to prove, and our
  process depends on the Build label; and install time is probably not the slowest step (see
  "Where the time goes"). Revisit after the next meet, and only if a timed loop shows install
  is the bottleneck.

### P1. One agent writes, the other reviews

- `[repo]` AGENTS.md already says one writer at a time per checkout, and one session in
  control of deployment.
- Idea `[inferred]`: before each deploy, the agent that did not write the change reads the
  diff and the test output and lists concerns. Different models miss different things. Costs a
  few minutes; no robot time.

### P2. One experiment file per robot session

- Idea `[inferred]`: before a robot session, write a short file with the question, the exact
  OpMode and settings, what result would count as yes or no, and the build label. After the
  session, the agent adds the log folder name and the numbers. Any later agent can then pick
  up the thread without being told the story again.
- Open question: where these live (next to the downloaded logs, or a `sessions/` folder here).
  Logs themselves stay out of Git.

## Not verified

Nothing in this note has been tried on our robot except N1, which is `[reported]`. In
particular:

- Whether the Panels Limelight proxy is installed, and whether it forwards port 5807.
- Whether the Driver Hub accepts ADB over USB as we have it set up, and whether `screencap`
  works on it.
- Whether camera-only OpModes behave the same when started from anything other than the
  Driver Hub.
- How long each step of the loop really takes.
- Whether Sloth works with SDK 12, Pedro 3 and Panels 1.0.12 together.
- Why USB-C to the Control Hub failed before.

## Decision log

| Date | Who | Decision |
| --- | --- | --- |
| 2026-11-15 | Ari | N1 is already in use (phone on USB). |
| 2026-11-15 | Ari | Keep this research in the repository so several agents can work from it. |

## Sources

Read on 2026-11-14 and 2026-11-15.

1. REV Hardware Client documentation (Control Hub has two boards; supported devices):
   <https://docs.revrobotics.com/rev-hardware-client/>
2. REV Hub Interface software (Expansion Hub only, not Control Hub):
   <https://docs.revrobotics.com/duo-control/legacy/rev-hub-interface-software>
3. REV Hub Interface, Community Edition (macOS and Linux):
   <https://pypi.org/project/REVHubInterface/>
4. REV, connecting a Control Hub by USB:
   <https://docs.revrobotics.com/rev-hardware-client/duo/control-hub>
5. Panels introduction (feature list): <https://pedro-pathing.github.io/Docs/docs/panels>
6. Panels compared with FTC Dashboard:
   <https://pedro-pathing.github.io/Docs/docs/panels/core/overview>
7. Panels Limelight Proxy plugin:
   <https://pedro-pathing.github.io/Docs/docs/panels/plugins/limelight>
8. Sloth README, read from a fork (find the Dairy Foundation original before adopting):
   <https://github.com/SMERobotics/Sloth>
9. Dairy Foundation Gradle plugins (Sloth builds of Panels and Dashboard):
   <https://github.com/Dairy-Foundation/Plugins>
10. Dairy Foundation documentation source, Sloth page (pull request 14):
    <https://github.com/Dairy-Foundation/docs/pull/14>
11. FTC Fast Load:
    <https://gist.github.com/MatthewOates36/1e38452236dc9f145c3a6b2addfb418f>
