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
restores the tested short baseline; Square loads the experimental two-leg field
preset. In C, both restore the supporting preset. Those presets are plans to
validate, not previously tested robot behavior.

## Field geometry and motor calibration solve different problems

Geometry chooses where the robot should go. The official CAD shows that an
18-inch robot on a direct primary-to-parking diagonal crosses the frame's
floor-level bar. Moving left before that diagonal avoids it. The opposite
shooting start needs an offset and small aiming angle to clear a flower, then
an initial forward move before the rightward departure. See
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

The supporting robot starts already aimed about three degrees off square to the
wall. Its two drive legs are measured relative to that heading. The autonomous
code does not execute that aiming turn. Humans place and aim it before INIT/START.

## Sequence timing and shooting

A sequence is a list of timed actions. `PollenSequence.tick()` returns commands
without blocking the OpMode loop. Shooting first qualifies encoder speed at 96%
of target for 0.25 s. Then intake and agitator feed continuously for the selected
duration. Normal speed dips do not pause the feed clock.

B's four-second spinup allowance happens after its start delay. Its timeout
skips the shot and proceeds along B's selected route. C spins the flywheel alone
during the final four seconds of its delay; intake, agitator and drive stay off.
At the default 15-second mark, a qualified launcher begins 7.5 seconds of feeding.
An unqualified launcher skips the shot but keeps the entire shooting time slot
stationary. This prevents a failure from sending C into its departure early.

A shooter INIT failure may permit the same mode's departure only if every mapped
shooter accepts zero power. C preserves its feed slot in that fallback too.
Explicitly selecting feed=0 removes the shooting slot instead. Clearance=0 skips
the first movement; distance=0 skips only the second/main movement. Both must be
zero for a stationary shooting test.

Mechanism commands are applied before drive commands, so launcher, intake and
agitator receive zero before departure. Nonfinite velocity, invalid/backwards
time, STOP, the independent deadline and runtime hardware errors stop outputs.
Electrical/mechanical stopping is not guaranteed merely by issuing commands.

## Time budgets and validation

B's budget includes delay + four-second spinup (if feeding) + feed + both drive
legs + 0.5-second hold. C overlaps spinup with the delay. Edits must keep the
worst case below 26 seconds, leaving transition margin before the independent
27-second cutoff. C also needs delay >=4 seconds while feeding is enabled.

Computer tests check arithmetic, transitions, locking, resets, bounds and fault
behavior. The geometry script checks nominal swept polygons and parking overlap.
Compiling against FTC checks API compatibility. A full Android build checks APK
integration. Only physical trials verify distance, heading, clearance, shooting
and partner timing. Neither CAD nor timed waiting confirms that a hive tipped.
