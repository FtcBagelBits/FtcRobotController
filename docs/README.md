# Pollen documentation

Use these documents with the code installed on the robot. New to the code?
Read [Start here](POLLEN_START_HERE.md) first.

| Document | Audience and purpose |
| --- | --- |
| [Desktop tools](DESKTOP_TOOLS.md) | Claude/Codex setup, ADB recovery, tests, build/install and diagnostics. |
| [Shared research notes](research/README.md) | Agents and mentors: research, options and decisions that are not yet rules or tested behavior. First note: [agent and robot loop](research/agent-robot-loop.md). |
| [Recordings and Control Hub storage](POLLEN_LOGGING.md) | Mentors and programmers: what every mode records, folder budgets, how to check free space, download-then-prune, and turning a move rehearsed in TeleOp into draft autonomous waypoints. |
| [Pollen code reference](POLLEN_CODE_REFERENCE.md) | Hardware names, facing transforms and established robot architecture. |
| [Automatic hive pipeline benchmark](POLLEN_HIVE_BENCHMARK.md) | Camera-only profile sweep, raw logs, four physical labels, download and analysis. |
| [Hive auto diagnostics](POLLEN_HIVE_DIAGNOSTICS.md) | v8 direct garden departure, motor/decision traces, unexpected-motion guard and staged retests. |
| [Garden pickup and flower drain practice](POLLEN_COLLECTION_PRACTICE.md) | Drivers and programmers tuning collection from a marked entry pose. |
| [Hive camera + garden testing](POLLEN_HIVE_TESTING.md) | Experimental: how the robot finds its hive cell and reads UP/DOWN from tag height, labeled calibration, isolated shot/garden tests, session reports, match modes, CAD garden route, validation. |
| [Start here](POLLEN_START_HERE.md) | Everyone new to the code: what runs, the three core ideas, a guided reading path, exercises, build commands. |
| [Set up, test, install, share](ROBOT_CONNECTION.md) | Students on Windows or Mac: Android Studio notes, running the computer tests, installing over the Control Hub's Wi-Fi, fixing connection problems, branches and pull requests. |
| [Operating manual](POLLEN_AUTONOMOUS.md) | Drivers and students: choose a routine, position the robot, use INIT controls (autonomous and PollenBot TeleOp), interpret warnings and record tested settings. |
| [Code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md) | Programmers: file map, hardware, sequencing, calibration, tests and deployment. |
| [How autonomous works](POLLEN_AUTONOMOUS_EXPLAINED.md) | Students: hardware names, lifecycle, timed movement, shooting and failure handling. |
| [Field layout](POLLEN_FIELD_LAYOUT.md) and [diagram](POLLEN_FIELD_LAYOUT.svg) | Programmers: permanent field geometry in one coordinate frame, tile names, heights, AprilTags, starting pieces, rules that constrain code. |
| [Robot tested behavior](POLLEN_ROBOT_BEHAVIOR.md) | Programmers: what the PollenBot has actually done on competition tiles, measured drive and shooter data, what is untested, next measurements. |
| [Field routes](POLLEN_FIELD_ROUTES.md) and [diagram](POLLEN_FIELD_ROUTES.svg) | Programmers and drive team: our start poses, route presets, clearances, parking margins. |
| [Pit sheet](POLLEN_PIT_SHEET.svg) | Alliance partners: lanes, parking slots, timing and setup rules on one printable page. |
| [Drive-team guides](drive-team/README.md) and [print PDFs](drive-team/print/) | Students at matches: Odo Shoot + Park, Odo Supporting Shooter, TeleOp; emergency and park-only pages. |
| [Odometry autos](POLLEN_ODOMETRY_AUTOS.md) and [baseline test](ODOMETRY_BASELINE_TEST.md) | Programmers: how the Odo modes drive, tuning knobs, Pinpoint checks. |

Menu controls belong in the operating manual; the drive-team guides are its short, student version,
so update both. Implementation details belong in
the code README. Field facts that do not depend on our robot belong in the field
layout; physical robot results belong in the tested-behavior record. Update documentation with behavior changes, and record the
installed commit with physical test results.
