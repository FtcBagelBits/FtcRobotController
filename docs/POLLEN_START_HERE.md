# PollenBot code: start here

This is the one page to read first. It explains what the PollenBot code does,
how it is built, and the order to read it in, with questions to answer as you
go. Every other document and code file is linked from here.

Build: `pollen-odo-v9`. The INIT screen of every Pollen OpMode shows it.

**New to the project?** First get Android Studio working and the code building:
[set up, test, install, and share your changes](ROBOT_CONNECTION.md) (Windows and Mac).

## What the robot runs

| Driver Station name | File | What it does |
| --- | --- | --- |
| **PollenBot** (TeleOp) | [`BioBuzzStarterbotOpMode.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzStarterbotOpMode.java) | Mecanum driving, intake on the triggers, launcher on R1, L1 reverses the launcher to clear a jam |
| **Odo ROBOT Shoot + Park** (our main auto) | [`pollen/PollenOdoRobotShoot.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdoRobotShoot.java) | Squares against the wall, shoots the 4 pre-loads, then drives each leg to a measured end point with the Pinpoint odometry |
| **Odo Supporting Shooter Auto** | [`pollen/PollenOdoSupportingShooter.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdoSupportingShooter.java) | Waits for the partner to shoot first, shoots, then parks using odometry |
| **Odo ROBOT Park** | [`pollen/PollenOdoPark.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdoPark.java) | Optional start delay, then drives a set distance with odometry |
| EMERGENCY Timed Park / Shoot + Park / Supporting Shooter (A, B, C) | [`pollen/`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/) | **Emergency only.** The same three jobs with timed driving, for a match where the odometry sensor isn't working |

The drive team's one-page guides are in [drive-team/](drive-team/README.md). Everything else in
TeamCode, including the old Lox robot's code in `lox/`, is listed in the
[TeamCode file guide](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/readme.md).

**What has worked on the real robot:** the TeleOp; shooting the pre-loads in autonomous; timed
Park (A) with its defaults. At our first event, Odo Shoot + Park shot but did not reliably park;
from `pollen-odo-v6` on, the shoot autos write what they did to the robot log so the next run can
be diagnosed. Everything else has only been checked by the computer tests. See
[tested behavior](POLLEN_ROBOT_BEHAVIOR.md) and [the odometry autos](POLLEN_ODOMETRY_AUTOS.md).

## Three ideas that explain all of it

1. **The SDK calls your code; your code never waits.** An `OpMode` has five
   methods. The SDK runs `init()` once, then `init_loop()` over and over
   until START, then `start()` once, then `loop()` about 50 times a second
   until STOP, then `stop()`. Each `loop()` reads the controller, the clock and
   the sensors, decides, sets the motors and returns. Nothing in it sleeps.
2. **Read once, decide, then write each motor once.** A loop reads each sensor
   once, works out every command in a variable, then calls `setPower` once per
   motor at the end. Setting the same motor more than once per loop can make it
   behave erratically, and reading a sensor twice can give two different answers.
3. **"What should happen right now?"** Autonomous is not written as "drive,
   wait, shoot". Every loop it looks at the clock (and, in the Odo modes, where
   the robot is), works out which step it is in, and returns that step's motor commands.

## Reading path (about an hour)

Code paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

### 1. TeleOp: `BioBuzzStarterbotOpMode.java` (20 min)

- **`init()`**: each `hardwareMap.get(...)` name (`frontLeft`, `frontRight`,
  `backLeft`, `backRight`, `intake`, `agitator`, `pollenLauncher`) must match
  the Robot Controller configuration exactly. A typo compiles fine and fails
  on the robot.
- **`init_loop()`**: the INIT menu (launcher speed, spin-up time, agitator
  power, drive power). It only shows and edits settings; nothing moves.
- **`loop()`**: follow `intakePower` from the triggers, through `launch()`, to
  `intake.setPower(...)`. The launcher's measured speed is read once, near the
  top, into `launcherMeasuredVelocity`.
  *Question:* why does `launch()` have to run before the intake power is written?
- **`launch()`**: the feed rule. Holding R1 spins the launcher. Feeding starts
  when the launcher is up to speed (96% of target, about 1133 ticks/s) **or** R1 has been held for
  the spin-up time (3 s), whichever comes first, and stays on until R1 is
  released.
  *Questions:* if the encoder cable falls out, when does feeding start? Why
  doesn't feeding stop when the wheel slows as balls go through?
- **`mecanumDrive()`**: how forward, strafe and turn become four wheel powers,
  and why they are divided by the largest one when any is over 1.

### 2. An INIT menu: [`pollen/PollenTeleOpTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenTeleOpTuning.java) (10 min)

- **`update()`**: D-pad up/down selects a row, left/right changes it,
  Triangle resets everything, START locks it.
- **`launcherIsUpToSpeed()`** and **`spinUpTimeIsOver()`**: the two halves of
  the feed rule that `launch()` combines with `||`.
- *Question:* what is `previous[]` for? What would happen without it while you
  hold the D-pad down?

### 3. The autonomous engine: [`pollen/PollenSequence.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenSequence.java) (15 min)

- A route is a list of `Step`s. There are four kinds (`Kind`): `DRIVE` (fixed
  powers for a set time; the emergency timed modes, and waits), `SHOOT`
  (launcher alone for the spin-up time, then feed), `MOVE` (drive to an end
  point, in inches from where the odometry was zeroed) and `ZERO_ODOMETRY`.
- **`tick(now, x, y, heading)`** is called every loop with the clock and the
  robot's position, and returns an `Output`: what every motor should do right now.
- **`compute()`** does the work. Read it top to bottom: first the checks that
  apply to every step (clock, loop gap, time limit, start delay), then one
  line per kind of step. Each kind has its own short method. Read them in this
  order: `driveStep`, `zeroOdometryStep`, `shootStep`, then `moveStep`.
- **`moveStep()`** asks four questions in order: has the robot arrived, is the
  leg out of time, is the robot stuck, and otherwise what powers does the
  drive controller want?
- *Question:* each step's clock starts on the first loop that runs it
  (`beginStep`), not when the previous step ended. What goes wrong with the
  other choice if one loop is slow?
- *Question:* find every `stop(` call. What are the ways a run ends early?
- *Exercise:* on paper, add a fifth kind of step, "wait until the hive tips".
  What would its method return each loop, and what new input would `tick` need?
- This file never touches hardware, which is why the computer tests can run it.

### 4. Driving to a point: [`pollen/PollenOdometryDrive.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdometryDrive.java) (10 min)

- Read the constants at the top first: each has a comment saying what it does.
- **`command(...)`** turns "how far is the end point, and in which direction" into
  a `Command`: forward, strafe and turn powers, plus `inchesToGo`. It slows
  down as the robot gets close, and braking is based on where the robot will
  be a moment from now.
- *Question:* a leg ends when the robot is within `POSITION_TOLERANCE_IN` **and**
  has stopped. Why isn't being close enough on its own?
- *Question:* what does the robot do if a partner robot is in the way? (Hint:
  `NO_PROGRESS_SECONDS` and the leg's give-up time.)

### 5. The autonomous OpMode: [`pollen/PollenShooterOpMode.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenShooterOpMode.java) (10 min)

- The same five methods as TeleOp. **`loop()`** has four numbered parts:
  **read** the clock and the odometry, **decide** by calling
  `sequence.tick(...)`, **act** by writing each motor once, and **report** to
  the Driver Hub screen and the robot log.
- *Question:* why are the launcher, intake and agitator set before the wheels?
- The Odo modes and the timed B and C are 10-line files that extend this class
  and only say "supporting shooter or not" and "odometry or not".

### 6. Where a route comes from: [`pollen/PollenShootTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenShootTuning.java), `sequence()` (5 min)

- It builds the step list: wall square (Odo modes), shoot, then the legs.
- *Exercise:* on paper, write the Supporting Shooter timeline: start delay,
  3 s spin-up, 7.5 s of feeding, then its legs. When does it start driving?

### 7. Field numbers: [`pollen/PollenFieldGeometry.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenFieldGeometry.java) (skim)

Positions from the official field CAD, in inches from the field center. The
[field layout](POLLEN_FIELD_LAYOUT.md) explains them.

### Skip for now

- [`pollen/PollenDriveCalibration.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenDriveCalibration.java):
  turns a distance and angle into seconds and wheel powers for the emergency
  timed modes. It is frozen.
- [`pollen/PollenParkTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenParkTuning.java),
  [`pollen/PollenOdoParkTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenOdoParkTuning.java):
  the Park menus. Same ideas as the shoot menus, with fewer rows.
- [`pollen/PollenAutoLog.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenAutoLog.java):
  writes what each shoot auto did to the robot log.
- [`pollen/PollenConfig.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenConfig.java):
  shared constants, including `BUILD`. Look things up here when you need them.

## Try it without the robot

- **Run the tests:** in Android Studio, open
  [`PollenTests.java`](../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen/PollenTests.java)
  and click the green arrow next to the class. All seven should pass.
  (From a terminal: `./gradlew :TeamCode:testDebugUnitTest` on Mac,
  `gradlew.bat :TeamCode:testDebugUnitTest` on Windows.)
- **Break a test on purpose:** change one expected number in
  [`PollenTeleOpTuningTest.java`](../TeamCode/src/test/java/org/firstinspires/ftc/teamcode/pollen/PollenTeleOpTuningTest.java),
  rerun, read the error, then change it back.
- **Trace by hand:** walk through `launch()` at 0.5 s, 1.2 s and 3.0 s of
  holding R1, once with the launcher reading 1180 and once reading 0.
- **Explain it:** what happens if `loop()` freezes for half a second while the
  robot is driving in autonomous? (Hint: `MAX_DRIVE_LOOP_GAP_SECONDS`.)

## Build and install

Build → Make Project in Android Studio, then install over the Control Hub's Wi-Fi and check that
INIT shows the build above. The steps, for Windows and Mac, and what to do when the connection
won't work, are in [ROBOT_CONNECTION.md](ROBOT_CONNECTION.md). Changes go in through a pull
request; that page explains how.

## What comes next

The odometry autos replaced timed driving; the timed ones are kept for emergencies only. Next is getting
Odo Shoot + Park to park reliably in matches, then using Pedro Pathing and a Limelight so
autonomous can cycle: shoot, collect more pollen from a known spot, shoot again, and park before
time runs out. The feed rule, the "what now?" loop, the INIT menus and the field coordinates carry
over. See [what carries over](POLLEN_AUTONOMOUS_EXPLAINED.md#what-carries-over-to-odometry-and-pedro).

## Other documents

| Document | Read it when |
| --- | --- |
| [Set up, test, install, share](ROBOT_CONNECTION.md) | You are setting up a laptop, installing on the robot, or opening a pull request |
| [Drive team guides](drive-team/README.md) | You run the robot at a match |
| [Operating manual](POLLEN_AUTONOMOUS.md) | You want every menu row and control |
| [Odometry autos](POLLEN_ODOMETRY_AUTOS.md) | You work on the Odo modes: what changed from timed, wall squaring, defaults |
| [How autonomous works](POLLEN_AUTONOMOUS_EXPLAINED.md) | You want the reasons behind the design |
| [Code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md) | You are changing the code: file map, tests, deployment |
| [Tested behavior](POLLEN_ROBOT_BEHAVIOR.md) | You test on the robot: what has run, what to measure, the run log |
| [Field layout](POLLEN_FIELD_LAYOUT.md), [field routes](POLLEN_FIELD_ROUTES.md) | You work on routes and positions |
| [Pit sheet](POLLEN_PIT_SHEET.svg) | You talk to alliance partners |
