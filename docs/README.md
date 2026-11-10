# Pollen documentation

Use these documents with the code installed on the robot. New to the code?
Read [Start here](POLLEN_START_HERE.md) first.

| Document | Audience and purpose |
| --- | --- |
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
