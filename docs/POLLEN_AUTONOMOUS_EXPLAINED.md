# How Pollen autonomous works

Pollen has three timed autonomous modes. A drives forward after a delay. B
shoots while stationary, optionally moves left to clear the hive frame, then
travels toward parking. C lets a partner shoot first, shoots from an opposite
end-wall pose, moves forward to clear a flower, then travels mostly right to park.
They do not measure position or detect arrival.

## INIT edits settings; START freezes them

`init()` maps devices and commands zero outputs. `init_loop()` reads controller
taps and displays selections without moving. `start()` locks settings and starts
a clock. `loop()` asks the hardware-independent sequencer for outputs and sends
them to motors. `stop()` latches the sequence stopped and attempts to zero every
mapped motor. A, B and C have separate process-local settings memory.

Use [the operating manual](POLLEN_AUTONOMOUS.md) for controls. In B, Triangle
restores the tested short baseline; Square loads the experimental three-leg field
preset. In C, both restore the supporting preset. Those presets are plans to
validate, not previously tested robot behavior.

## Field geometry and motor calibration solve different problems

Geometry chooses where the robot should go. The official CAD shows that an
18-inch robot on a direct primary-to-parking diagonal crosses the frame's
floor-level bar. Moving 10 cm off the wall, then left, before that diagonal
avoids it. The opposite shooting start sits flush and square to the wall,
shifted toward the midline to clear a flower, then moves forward before the
rightward departure. See
[the field analysis](POLLEN_FIELD_ROUTES.md) and [diagram](POLLEN_FIELD_ROUTES.svg).

Calibration estimates what motor commands achieve those displacements. The
reported 45-degree command traveled 54 cm forward and 31 cm sideways: its
physical direction was only 29.86 degrees. Running longer would preserve that
wrong direction. Both direction and duration need correction.

`PollenDriveCalibration` provides two profiles:

- **TESTED_BASELINE:** unchanged coefficient 129, time = distance/(129*power),
  with direct sine/cosine motor inputs. B still defaults to this profile.
- **MAT_TRIAL:** piecewise interpolation/inversion of the reported forward,
  diagonal and sideways results. It changes the commanded direction and time
  while preserving selected vector-power magnitude. The original shoot-test
  settings were uncertain; this profile assumes unchanged baseline code and
  comparable power. It is not a verified physical model.

At power 0.20, MAT_TRIAL predicts a +56.33-degree command for 3.534 seconds to
travel 75 cm at a physical +45 degrees. That is a prediction between the measured
points. Left/right symmetry and scaling to other powers also require testing.
A remains time-controlled; its new mat distance display uses the known-power
forward measurements without changing the existing motor command.

## Translation is not rotation

The selected angle is positive toward robot-left. The drive mixer uses positive
right strafe, so command-angle conversion is:

```text
forward = power * cos(command_angle)
strafe  = -power * sin(command_angle)
turn    = 0
```

With MAT_TRIAL, `command_angle` differs from the desired path angle. The four
wheel commands are forward +/- strafe +/- turn. If any exceeds magnitude one,
all are scaled by the same largest magnitude. The supported translation powers
stay below that saturation boundary. Zero turn commands do not correct drift.

Both shooters start flush and square to an end wall, so every leg angle is
measured from a heading parallel to the field axes. The supporting robot always
shoots straight forward; its centerline sits about 6.9 cm toward the midline
from the hive center. Nothing turns the chassis.

## Sequence timing and shooting

A sequence is a list of timed actions. `PollenSequence.tick()` returns commands
without blocking the OpMode loop. Each step's clock starts on the first loop
that runs it, so a slow loop between two steps cannot use up the next step's
time. The motors keep their last command while the program is paused, so a
pause of more than 0.25 s while the wheels are moving stops everything:
after that, the robot's position is not what the plan assumes.

Shooting uses fixed times, not measured speed. The launcher runs alone for the
spin-up time (3 s by default, an INIT row), then intake and agitator feed
continuously for the selected duration with the launcher still on. An earlier
version waited until the encoder read 96% of target speed; when the encoder
read low, the feeder never started, so that check was removed. B spins up right
after its start delay. C waits 12 s, spins up from 12 to 15 s, and feeds from
15 to 22.5 s, the slot agreed with a partner who shoots first. In TeleOp the
driver holds R1: feeding starts when the launcher reads 96% of its target or
after the spin-up time, whichever comes first, and stays on until R1 is
released. The speed check can only make feeding start sooner, never stop it.

A shooter INIT failure may permit the same mode's departure only if every mapped
shooter accepts zero power. C preserves its feed slot in that fallback too.
Explicitly selecting feed=0 removes the shooting slot instead. Each of the three
legs has its own distance and angle row; a zero-cm leg is skipped and the others
still run. All three must be zero for a stationary shooting test.

Mechanism commands are applied before drive commands, so launcher, intake and
agitator receive zero before departure. Invalid/backwards time, a long pause
while driving, STOP, the independent deadline and runtime
hardware errors stop outputs.
Electrical/mechanical stopping is not guaranteed merely by issuing commands.

## Time budgets and validation

Each budget is delay + spin-up (if feeding) + feed + all three drive legs +
0.5-second hold. Edits must keep the total below 26 seconds, leaving
transition margin before the independent 27-second cutoff.

Computer tests check arithmetic, transitions, locking, resets, bounds and fault
behavior. The geometry script checks nominal swept polygons and parking overlap.
Compiling against FTC checks API compatibility. A full Android build checks APK
integration. Only physical trials verify distance, heading, clearance, shooting
and partner timing. Neither CAD nor timed waiting confirms that a hive tipped.

## What carries over to odometry and Pedro

Timed driving is a bridge. Once odometry and Pedro Pathing work, the robot
drives to a position and knows when it got there, instead of guessing from
time. The plan after that is a cycle: shoot the pre-loads, collect more pollen
from a known spot (probably a flower), shoot again, and repeat until it is time
to park or the hive has tipped.

These parts go away then, so don't spend effort polishing them now:

- timed legs (distance, angle, power turned into seconds) and both
  `PollenDriveCalibration` profiles, including MAT_TRIAL;
- the leg rows in the INIT menu and the per-leg time budget arithmetic;
- the stall stop for timed driving (a path follower corrects its own position).

These carry over:

- **Field coordinates.** `PollenFieldGeometry` holds CAD positions in inches
  from the field center. They become the target positions for paths, after one
  conversion into Pedro's own field coordinates.
- **The launcher rule** from TeleOp: feed when the launcher is up to speed or
  after the backup timer, and keep feeding until done. A cycle cannot afford to
  wait the full timer every time.
- **"What now?" every loop.** `PollenSequence` never blocks: each loop it
  looks at the clock and returns what the motors should do. A cycle works the
  same way, except it chooses the next step while running. Before each new
  cycle it asks one question: is there time to finish this cycle and still
  park before the 27 s cutoff?
- **INIT menus with memory** for settings the drive team changes between
  matches, such as launcher speed and start delay.
- **Stop conditions:** time to park, or (with the Limelight) the hive has
  tipped.

The numbers that will decide how many cycles fit, worth measuring as soon as
the mechanisms exist: seconds to shoot a full load, seconds to collect a load,
and driving time between the shooting spot, the flower and the parking zone.
