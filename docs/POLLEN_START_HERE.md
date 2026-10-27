# PollenBot code: start here

This is the one page to read first. It explains what the PollenBot code does,
how it is built, and the order to read it in, with questions to answer as you
go. Every other document and code file is linked from here.

Build: `pollen-mat-v9`. The INIT screen of every Pollen OpMode shows it.

## What the robot runs

| Driver Station name | File | What it does |
| --- | --- | --- |
| **PollenBot** (TeleOp) | [`BioBuzzStarterbotOpMode.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/BioBuzzStarterbotOpMode.java) | Mecanum driving, intake on the triggers, launcher on R1 |
| **Pollen ROBOT Park** (A) | [`PollenRobotPark.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenRobotPark.java) | Optional start delay, then drives forward 2 s at power 0.20 |
| **Pollen ROBOT Shoot + Park** (B) | [`PollenRobotShoot.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenRobotShoot.java) | Shoots the 4 pre-loads, then drives up to three legs to park |
| **Pollen Supporting Shooter Auto** (C) | [`PollenSupportingShooter.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenSupportingShooter.java) | Waits 12 s for a partner to shoot, shoots, then parks |

**What has worked on the real robot:** A with its defaults, and B with its
Triangle defaults. Everything else, including the TeleOp feed rule in this
build, has only been checked by computer tests. See
[tested behavior](POLLEN_ROBOT_BEHAVIOR.md) for the details.

## Three ideas that explain all of it

1. **The SDK calls your code; your code never waits.** An `OpMode` has five
   methods. The SDK runs `init()` once, then `init_loop()` over and over
   until START, then `start()` once, then `loop()` about 50 times a second
   until STOP, then `stop()`. Each `loop()` reads the controller and the clock,
   decides, sets the motors and returns. Nothing in it sleeps.
2. **Decide first, then write each motor once.** A loop works out every
   command in a variable, then calls `setPower` once per motor at the end.
   Setting the same motor more than once per loop can make it behave erratically.
3. **"What should happen right now?"** Autonomous is not written as "drive,
   wait, shoot". Every loop it looks at the clock, works out which step it is
   in, and returns that step's motor commands.

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
  `intake.setPower(...)`.
  *Question:* why does `launch()` have to run before the intake power is written?
- **`launch()`**: the feed rule. Holding R1 spins the launcher. Feeding starts
  when the launcher is up to speed (1200 ticks/s) **or** R1 has been held for
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

- A route is a list of `Step`s: a drive step (forward, strafe, turn for some
  seconds) or a shoot step (launcher alone for the spin-up time, then feed).
- **`tick(now)`** is called every loop with the clock and returns an `Output`:
  what every motor should do right now.
- *Question:* each step's clock starts on the first loop that runs it, not when
  the previous step ended. What goes wrong with the other choice if one loop
  is slow?
- *Question:* find every `stop(` call. What are the ways a run ends early?
- This file never touches hardware, which is why the computer tests can run it.

### 4. The autonomous OpMode: [`pollen/PollenShooterOpMode.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenShooterOpMode.java) (10 min)

- The same five methods as TeleOp. **`loop()`** calls `sequence.tick(now)`
  and sends the result to the motors.
- *Question:* why are the launcher, intake and agitator set before the wheels?
- B and C are 9-line files that extend this class and only choose which menu
  to use.

### 5. Where a route comes from: [`pollen/PollenShootTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenShootTuning.java), `sequence()` (5 min)

- It builds the step list: shoot, then legs 1 to 3, then a short hold.
- *Exercise:* on paper, write C's timeline: 12 s wait, 3 s spin-up, 7.5 s of
  feeding, then its legs. When does it start driving?

### 6. Field numbers: [`pollen/PollenFieldGeometry.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenFieldGeometry.java) (skim)

Positions from the official field CAD, in inches from the field center. The
[field layout](POLLEN_FIELD_LAYOUT.md) explains them.

### Skip for now

- [`pollen/PollenDriveCalibration.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenDriveCalibration.java):
  turns a distance and angle into seconds and wheel powers. It is frozen and
  will be replaced by odometry.
- [`pollen/PollenParkTuning.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenParkTuning.java)
  and [`pollen/PollenRobotPark.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenRobotPark.java):
  mode A. Same ideas as B and C, with simpler controls.
- [`pollen/PollenConfig.java`](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/PollenConfig.java):
  shared constants. Look things up here when you need them.

## Try it without the robot

- **Run the tests** (from the repository root; needs Java):
  `bash tools/test-pollen-sequence.sh`. Every line should say `PASS` or
  `checks passed`.
- **Break a test on purpose:** change one expected number in
  [`tools/pollen-tests/PollenTeleOpTuningTest.java`](../tools/pollen-tests/PollenTeleOpTuningTest.java),
  rerun, read the error, then change it back.
- **Trace by hand:** walk through `launch()` at 0.5 s, 1.2 s and 3.0 s of
  holding R1, once with the launcher reading 1250 and once reading 0.
- **Explain it:** what happens if `loop()` freezes for half a second while the
  robot is driving in autonomous? (Hint: `MAX_DRIVE_LOOP_GAP_SECONDS`.)

## Build and install

From the repository root on the team Mac:

```bash
export JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home"
export PATH="$JAVA_HOME/bin:$PATH"
bash tools/test-pollen-sequence.sh && ./gradlew :TeamCode:assembleDebug
```

The first command runs the computer tests; the second builds the robot app
only if they pass. Install it from Android Studio over the team's Wi-Fi
connection to the Control Hub, then check that INIT shows `pollen-mat-v9`.

## What comes next

Timed driving is a bridge. The plan is odometry with Pedro Pathing, so the
robot knows where it is, and a Limelight to see the field. Then autonomous can
cycle: shoot, collect more pollen from a known spot, shoot again, and repeat
until it is time to park or the hive has tipped. The feed rule, the
"what now?" loop, the INIT menus and the field coordinates carry over;
timed legs and drive calibration do not. See
[what carries over](POLLEN_AUTONOMOUS_EXPLAINED.md#what-carries-over-to-odometry-and-pedro).

## Other documents

| Document | Read it when |
| --- | --- |
| [Operating manual](POLLEN_AUTONOMOUS.md) | You are on the drive team: menus, controls, robot placement |
| [How autonomous works](POLLEN_AUTONOMOUS_EXPLAINED.md) | You want the reasons behind the design |
| [Code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md) | You are changing the code: file map, tests, deployment |
| [Tested behavior](POLLEN_ROBOT_BEHAVIOR.md) | You test on the robot: what has run, what to measure, the run log |
| [Field layout](POLLEN_FIELD_LAYOUT.md), [field routes](POLLEN_FIELD_ROUTES.md) | You work on routes and positions |
| [Pit sheet](POLLEN_PIT_SHEET.svg) | You talk to alliance partners |
