# Pollen autonomous code

Build identifier: **pollen-mat-v2** (`PollenConfig.BUILD`). Three autonomous OpModes preselect PollenBot:
`Pollen ROBOT Park`, `Pollen ROBOT Shoot + Park`, and
`Pollen Supporting Shooter Auto`.

Read the [operator manual](../../../../../../../../../docs/POLLEN_AUTONOMOUS.md),
[calibration analysis](../../../../../../../../../docs/POLLEN_MAT_CALIBRATION.md),
[field geometry](../../../../../../../../../docs/POLLEN_FIELD_ROUTES.md), and
[student explanation](../../../../../../../../../docs/POLLEN_AUTONOMOUS_EXPLAINED.md).
The experimental profiles/routes are not validated robot settings.

## Code map

| File | Responsibility |
| --- | --- |
| PollenRobotPark | Option A FTC lifecycle, four drives, INIT controls and distance estimate |
| PollenRobotShoot | Registered Option B wrapper |
| PollenSupportingShooter | Registered Option C wrapper |
| PollenShooterOpMode | Shared B/C hardware lifecycle, isolated memory, telemetry and faults |
| PollenConfig | Shared tested drive mapping; unchanged A default power/time |
| PollenParkTuning | A controller editing, bounds, delay budget and lock |
| PollenShootTuning | B/C menus (launcher, feed, power, three legs' cm/angle), defaults, field presets, budget and route construction |
| PollenDriveCalibration | Unchanged baseline; opt-in B / default C mat trial; A forward estimate |
| PollenFieldGeometry | Nominal CAD-derived start/parking coordinates and preset displacements |
| PollenSequence | Hardware-free timing, prespin, speed qualification, feed, STOP and mixer |

B and C use distinct static snapshots in the shared OpMode. Their INIT loads
never share settings. Start delay resets to the mode default; an invalid C
snapshot is reset before sequence creation. START locks every control, including
Triangle and Square. These are process-local snapshots, not persisted files.

## Hardware contract

| Name | Device | Direction / control |
| --- | --- | --- |
| frontLeft | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |
| frontRight | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| backLeft | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| backRight | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |
| pollenLauncher | DcMotorEx | REVERSE, RUN_USING_ENCODER; PIDF 40/0/0/12.5 |
| intake | DcMotor | FORWARD, RUN_WITHOUT_ENCODER, BRAKE |
| agitator | DcMotor | REVERSE, RUN_WITHOUT_ENCODER, BRAKE |

A maps only the four drives; B/C map all seven. No windmill feeder is used.
Launcher targets are encoder ticks/s, not RPM. The existing hardware setup and
motor directions are unchanged. The shared shooter lifecycle is extracted from
the tested B implementation so the modes do not duplicate motor/fault handling.

Drive axes are positive forward, right, clockwise. Translation has zero turn.
Wheel order is front-left, front-right, back-left, back-right; mixer commands are
`f+s+t`, `f-s-t`, `f-s+t`, `f+s-t`, normalized together if magnitude exceeds one.

## Routes and timing

- A: delay -> timed forward -> hold.
- B baseline: delay -> spinup -> feed -> leg 3 only (tested 60.96 cm) -> hold.
- B Square field preset: same stationary shot -> leg 1 forward 10 cm off the
  wall -> leg 2 left 63.5 cm -> leg 3 forward-left to park -> hold.
  Triangle restores the original short baseline.
- C: delay with flywheel-only prespin during its final four seconds -> feed ->
  leg 1 forward flower clearance -> (leg 2 unused) -> leg 3 mostly right -> hold.

`LEG1_CM`/`LEG1_ANGLE` through `LEG3_CM`/`LEG3_ANGLE` are independent controller
rows; every leg uses `DRIVE_POWER` and the selected calibration profile, and a
0-cm leg is skipped. All three must be zero to disable all travel. Feed=0 removes spinup
and feeding. If C's shooter is unavailable, a zero-output step occupies its feed
slot; if prespin times out, the sequencer holds the same slot. Explicit feed=0
instead departs after the chosen start delay. Invalid velocity stops the run.

The original PollenSequence constructors delegate with `spinupDuringDelay=false`
and retain existing behavior. The new overload permits overlap only when the
first step shoots and delay is at least the spinup timeout. It subtracts that
one overlapped allowance from worst-case duration; subsequent shots, if added,
still require their own spinup allowance. Only the flywheel runs during prespin.

Budget includes all three drive legs and hold. Selected worst case must be <26 s;
the independent cutoff is 27 s. Runtime hardware exceptions attempt every stop
and request OpMode stop. Shooter INIT fallback requires all mapped shooter stop
commands to succeed; drive failure blocks. B/C never substitute A's starting-pose
route. Hardware output order stops mechanisms before any drive command.

## Calibration and geometry

A still commands seconds/power directly; its display uses the measured forward
fit, with explicitly unverified interpolation/extrapolation. B defaults to the
original 129 model at +25.87 degrees, 60.96 cm and power 0.20. The mat trial uses
separate measured direction/distance knots, not a direction-independent scalar.
See the calibration document for assumptions and exact measurements.

Square loads complete nominal field settings. C's default start is flush and
square, centered between the flower and the midline (6.89 cm toward the midline
from the hive center); a hive-centered start overlaps the flower.
The field guide records the CAD geometry, tight initial margins and 180-degree
alliance transform. Neither these constants nor the timed routes use Pedro,
Pinpoint, AprilTags, voltage compensation or position/heading feedback.

## Tests and Android build

From the repository root on the team's Mac, use the existing `ftcgradle` helper:

```bash
PATH="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin:$PATH" \
  bash tools/test-pollen-sequence.sh
python3 tools/check-pollen-field-routes.py
ftcgradle :TeamCode:assembleDebug
```

The Java script uses Java 8 compatibility and covers original A/B behavior,
mat arithmetic, field-preset selection, C timing and skipped-shot slots, all three
movement legs, controller edits of each leg and launcher speed, budgets, memory and STOP. The standard-library Python script
checks nominal swept polygons, including the rejected direct route and rejected
centered support start. Neither establishes actual physical clearance.
`tools/draw-pollen-field-routes.py` and `tools/draw-pollen-pit-sheet.py` (matplotlib)
redraw the engineering diagram and the alliance pit sheet; they draw, not check.

`ftcgradle` is the team's existing shell helper; inspect `type ftcgradle` if it
is unavailable. The older `tools/pollen-patch.sh` still requires the historical
`pollen-auto-prep` branch: on this feature branch use the explicit checks above,
not that branch-locked helper.

Deploy a freshly built APK through the team's working Wi-Fi/ADB connection or
Android Studio TeamCode run configuration to the Control Hub. USB-C has not
worked. Verify `pollen-mat-v2` in INIT. Check the actual installed commit and all
settings before physical tests. Java changes require rebuild/reinstall; INIT
settings and documentation edits do not.
