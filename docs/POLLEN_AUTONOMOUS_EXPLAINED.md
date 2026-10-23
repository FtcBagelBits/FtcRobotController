# Pollen autonomous: how it works

Pollen has two autonomous choices: **Pollen ROBOT Park** and
**Pollen ROBOT Shoot + Park**. Both execute timed actions. They do not measure
robot position or detect arrival at a parking zone.

Use the [operating manual](POLLEN_AUTONOMOUS.md) for controller instructions and
the [code README](../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pollen/README.md)
for hardware details, source files and build instructions.

## Hardware names connect configuration to Java

The team names each device in Configure Robot. The active configuration associates
that name with a hub, port and device type. Java retrieves the device by name:

```java
DcMotor motor = hardwareMap.get(DcMotor.class, "frontLeft");
```

Names are case-sensitive. `frontLeft` identifies a configured device; the name
does not determine its direction. Direction is set separately in Java to match
wheel mounting and gearing.

`PollenConfig` lists the four drive names and direction flags. The shooting
routine also retrieves `pollenLauncher`, `intake` and `agitator`. Its launcher
uses encoder velocity control; the intake and agitator use motor power.

## INIT selects settings; START runs the sequence

| Callback | Responsibility |
| --- | --- |
| `init()` | Map and configure devices, command zero outputs and load settings. |
| `init_loop()` | Read controller taps and display settings without moving motors. |
| `start()` | Lock settings, reset the clock and construct the selected sequence. |
| `loop()` | Ask the sequence for outputs, apply them to hardware and report telemetry. |
| `stop()` | Stop the sequence and attempt to zero each mapped motor. |

A button edit changes configuration, not motor output. Held buttons do not repeat.
Settings remain in app memory between runs, except start delay, which resets
every INIT. Restarting the Robot Controller app loses that memory. Triangle
restores code defaults. START locks settings for the run.

## A sequence is a list of timed actions

**Option A:** start delay → forward drive → zero-output hold → DONE.

**Option B:** start delay → launcher spinup → stationary feeding → angled drive
→ zero-output hold → DONE.

`PollenConfig.parkRoute()` builds A's drive steps.
`PollenShootTuning.sequence()` builds B's steps from the selected settings.
`PollenSequence.tick()` checks the clock and current step each loop, then returns
the requested outputs. It does not wait inside a long driving or shooting loop.

A drive step specifies seconds and a robot-relative movement vector:

```java
PollenSequence.Step.drive("Forward", 0.2, 0.3, 0, 0);
```

This requests 0.3 forward power for 0.2 seconds. Positive drive axes are forward,
right strafe and clockwise turn. Humans choose the starting pose and clear route.
A step name containing PARK does not establish that the robot is physically parked.

## Angle describes translation, not a chassis turn

B's menu angle uses positive degrees for left of robot-forward, negative for
right and zero for forward. `PollenShootTuning` converts it into movement inputs:

```text
forward = vector_power × cos(angle)
strafe  = -vector_power × sin(angle)
turn    = 0
```

The minus sign converts left-positive angle into right-positive strafe.
The chassis is commanded to keep its orientation, but there is no heading
feedback to correct drift.

The mixer calculates four wheel commands:

```text
frontLeft  = forward + strafe + turn
frontRight = forward - strafe - turn
backLeft   = forward - strafe + turn
backRight  = forward + strafe - turn
```

If any magnitude exceeds one, all four are divided by the largest magnitude.
This preserves their proportions while keeping commands within the allowed range.
Vector power is not the same as each wheel's individual power.

## Distance is converted into time

B calculates:

```text
drive_seconds = selected_distance_cm / (129 × selected_vector_power)
```

At the code defaults, 60.96 cm and power 0.20 request about 2.363 seconds of
driving at +25.87 degrees. The coefficient is fitted near that angle. The same
coefficient is used throughout the angle menu, so changing angle can change the
actual distance traveled even when the selected distance stays the same.

Battery, surface, load and wheel slip also affect travel. This is an open-loop
estimate: no sensor checks the endpoint. Keep the known calibration until repeat
measurements justify changing it. Hold power fixed, measure forward and sideways
displacement, adjust angle for direction and distance for the endpoint, then
repeat with shooting included. Check the complete robot footprint and partner
route before using a full-field path.

## Shooting qualifies speed once, then feeds continuously

B sets launcher velocity in encoder ticks per second. Once measured speed is at
least 96% of the selected target for 0.25 seconds, it runs the intake and agitator
for the selected feed duration while keeping the flywheel running. Normal speed
dips during feeding do not pause it. The displayed feed time measures commanded
motor time; it does not count balls or confirm scoring.

Spinup has a four-second timeout. A timeout skips feeding and continues the
selected departure. Nonfinite velocity data stops the sequence. The OpMode sends
zero shooter commands before its first driving command.

Zero feed duration skips shooting; zero travel distance skips driving. Both zero
leave delay and hold only. Hardware initialization still runs.

## Failure handling preserves the selected starting pose

If shooting initialization fails, B attempts to stop every mapped shooting motor.
If those commands succeed, it displays a warning and can run its own selected
departure without shooting. It does not substitute A's route, because A uses a
different starting pose. Drive initialization failures or failed shooter stop
commands block the run.

Runtime hardware exceptions attempt to stop all devices and request OpMode stop.
The sequencer also stops on STOP, invalid or backwards time, and its independent
27-second cutoff. Route editing reserves one second below that cutoff for
transitions. Software stop commands do not prove immediate mechanical stopping
or guarantee recovery from an electrical failure.

## Where students make changes

| File in the `pollen` package | Responsibility |
| --- | --- |
| `PollenRobotPark.java` | A's FTC lifecycle, controller input, telemetry and drive output. |
| `PollenRobotShoot.java` | B's lifecycle, shooting hardware, fallback and output. |
| `PollenConfig.java` | Shared drive mapping and A's permanent defaults. |
| `PollenParkTuning.java` | A's INIT settings, bounds and memory. |
| `PollenShootTuning.java` | B's settings, calibration and route construction. |
| `PollenSequence.java` | Timed execution, shooting readiness, motor mixing and stop conditions. |

Use INIT for routine tuning. Save validated permanent defaults in the appropriate
configuration class. Computer tests check sequencing and arithmetic; APK
compilation checks SDK integration; physical trials establish travel, shooting
and STOP behavior. Record results against the installed commit in the operating
manual's settings record.
